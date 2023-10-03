#include "ground_removal.h"
#include "cluster.h"
#include "message_parsing.h"

using namespace Eigen;
using namespace std;

//RANSAC
const bool RANSAC = true;
const bool REATTACH = false;
const float PLANE_RANSAC_THRESHOLD = 0.005;
const float PLANE_RANSAC_EXTRACT_THRESHOLD = 0.03;
const float PLANE_RANSAC_ANGLE_THRESHOLD = 0.05;
const float PLANE_RANSAC_PROBABILITY = 0.99;
const int PLANE_RANSAC_MAX_ITERATIONS = 500;

//SPHERICAL CLUSTERING
float RADIAL_THRESHOLD = 0.3;
int MIN_SEGMENT_LENGTH = 2;
float D_MAX_BREAKPOINTS = 0.05;
float D_THRESHOLD = 0.02;
float EPSILON = 0.09;
float LAMBDA = 0.12;

float ALPHA = 0;
float ALPHA_RANGE = 0.03;

float BETA = 0;
float BETA_RANGE = 0.03;

int MIN_CLUSTER_SIZE = 4;
int AZIMUTH_INDEX_THRESHOLD = 2;

//GROUND_REMOVAL

extern pcl::PointCloud<pcl::PointXYZI> after_ransac_cloud;
extern pcl::PointCloud<pcl::PointXYZRGB> ground_segment_cloud;
extern pcl::PointCloud<pcl::PointXYZRGB> object_segment_cloud;
extern pcl::PointCloud<pcl::PointXYZRGB> azimuth_group_cloud;
extern pcl::PointCloud<pcl::PointXYZI>::Ptr possible_ground_points;
extern pcl::PointCloud<pcl::PointXYZRGB> cluster_cloud;
extern pcl::PointCloud<pcl::PointXYZRGB> cluster_with_reattached_ground_cloud;
extern visualization_msgs::MarkerArray markers;
extern float lidar_height;

//Rosparams
int n_bins;
int span;
int layers;
float valid_plane_angle_threshold; //Maximum angle between planes for them to be valid
float dist_to_plane_threshold; //The maximum distance from a point to a plane thats part of that plane

void read_rosparams()
{
    ros::param::param<int>("/perception/ground_removal/n_bins", n_bins, 6);
    ros::param::param<int>("/perception/ground_removal/span", span, 3);
    ros::param::param<int>("/perception/ground_removal/layers", layers, 7);
    ros::param::param<float>("/perception/ground_removal/valid_plane_angle_threshold", valid_plane_angle_threshold, 0.98);
    ros::param::param<float>("/perception/ground_removal/dist_to_plane_threshold", dist_to_plane_threshold, 0.03);
}

float average_radial_distance(vector<vector<float>>& r_matrix, vector<vector<float>>& theta_matrix, list<int>::iterator& it, int span, int i)
{
    int counter = 0;
    list<int>::iterator avg_it;
    float average, new_average;
    float theta = theta_matrix[i][*it];
    average = 0;
    counter = 0;
    for (avg_it = prev(it, span); avg_it != next(it, span + 1); avg_it++) {
        if (abs(theta - theta_matrix[i][*avg_it]) > span * 0.00344)
            continue;
        average += r_matrix[i][*avg_it];
        counter++;
    }
    average = average / counter;

    new_average = 0;
    counter = 0;
    for (avg_it = prev(it, span); avg_it != next(it, span + 1); avg_it++) {
        if (abs(theta - theta_matrix[i][*avg_it]) > span * 0.00344)
            continue;
        if (r_matrix[i][*avg_it] - average > average * 0.05)
            continue;
        new_average += r_matrix[i][*avg_it];
        counter++;
    }
    return new_average = new_average / counter;
}

void visulize_bins(int n_bins)
{
    Vector3f v1, v2, v3;
    vector<Vector3f> triangles;
    float theta;
    v1 << 0, 0, -lidar_height;
    for (int i = 0; i <= n_bins; i++) {
        theta = -FOV_LIDAR + i * (2 * FOV_LIDAR) / (n_bins);
        v2 << 10, theta, 0;
        v2 = polar_to_cartesian(v2);
        v2[2] = -lidar_height;
        triangles.push_back(v1);
        triangles.push_back(v2);
    }
    Vector3f rgb(1, 1, 0);
    markers.markers.push_back(line_markers(triangles, rgb, "bins"));
}

void setup_plane(Plane& plane, Plane& last_valid_plane,
    const float r1, const float r2, const float inc,
    const float theta1, const float theta2,
    vector<Vector3f>& triangles, vector<Vector3f>& non_valid_triangles)
{
    Vector3f p1, p2, p3;
    p1 << r1, theta1, inc;
    p2 << r2, theta2, inc;
    p1 = polar_to_cartesian(p1);
    p2 = polar_to_cartesian(p2);
    p3 = last_valid_plane.midpoint;
    plane.coeff = fit_plane_to_3_points(p1, p2, p3);
    plane.center = (p1 + p2) / 2;
    plane.midpoint = (p1 + p2 + p3) / 3;

    //Compare normal to previous plane

    float dot_product = 0;
    for (int a = 0; a < 3; a++)
        dot_product += plane.coeff[a] * last_valid_plane.coeff[a];
    plane.valid = dot_product > valid_plane_angle_threshold && plane.coeff[2] > 0.7;

    //Add triangles for visualization
    if (plane.valid) {
        triangles.push_back(p1);
        triangles.push_back(p2);
        triangles.push_back(p3);
    } else {
        non_valid_triangles.push_back(p1);
        non_valid_triangles.push_back(p2);
        non_valid_triangles.push_back(p3);
    }
}

void mesh_ground_removal(vector<vector<float>>& r_matrix,
    vector<vector<float>>& theta_matrix,
    vector<vector<Vector3f>>& cart_matrix,
    const vector<float>& phis,
    vector<pair<int, int>>& after_ransac_indices,
    vector<list<int>>& r_matrix_nonzero_indices)
{
    //Parameters
    int points_per_bin = 1824 / n_bins;
    int span = 3;
    int layers = 7;
    list<int> dummy_list;

    //Variables
    //vector<vector<list<int>::iterator>> lowest_bin_index(16, vector<list<int>::iterator>(n_bins, dummy_list.end()));
    vector<vector<int>> lowest_bin_index(16, vector<int>(n_bins, -1));
    vector<vector<float>> lowest_bin_radius(16, vector<float>(n_bins, -__FLT_MAX__));
    vector<vector<Vector3f>> lowest_point_in_bin(16, vector<Vector3f>(n_bins, Vector3f(0, 0, 0)));
    vector<vector<bool>> object_points(16, vector<bool>(1824, false));

    int bin_index;
    list<int>::iterator it, it_start, it_end;
    float average;
    visulize_bins(n_bins);
    Vector3f p;
    for (int i = 0; i < 16; i++) {
        it_start = next(r_matrix_nonzero_indices[i].begin(), span);
        it_end = prev(r_matrix_nonzero_indices[i].end(), span);
        for (it = it_start; it != next(it_end, 1); ++it) {
            object_points[i][*it] = true;
            average = average_radial_distance(r_matrix, theta_matrix, it, span, i);
            bin_index = (int)(((theta_matrix[i][*it] + FOV_LIDAR) / (2 * FOV_LIDAR)) * n_bins);
            int closest_bin_index = (int)round((((theta_matrix[i][*it] + FOV_LIDAR) / (2 * FOV_LIDAR)) * n_bins));
            float theta_edge = -FOV_LIDAR + 2 * FOV_LIDAR / n_bins * closest_bin_index;

            //If to close to bin seperators skip
            if (abs(theta_matrix[i][*it] - theta_edge) < FOV_LIDAR / (n_bins * 8))
                continue;

            if (average > lowest_bin_radius[i][bin_index]) {
                lowest_bin_radius[i][bin_index] = average;
                lowest_bin_index[i][bin_index] = *it;
                p << average, theta_matrix[i][*it], phis[i];
                lowest_point_in_bin[i][bin_index] = polar_to_cartesian(p);
            }
        }
    }

    int index1, index2, index1_prev, index2_prev;
    vector<Vector3f> triangles;
    for (int i = 0; i < 5; i++) { // Loop over all the channels of the Lidar
        for (int j = 0; j < n_bins - 1; j++) { //Loop over every bin
            index1 = lowest_bin_index[i][j];
            index2 = lowest_bin_index[i][j + 1];
            if (i == 0) { //If first channel create one triangle

                if (index1 == -1 || index2 == -1)
                    continue;
                Vector3f p1, p2, p3;
                p1 = lowest_point_in_bin[i][j];
                p2 = lowest_point_in_bin[i][j + 1];
                p3 << 0, 0, -lidar_height;

                triangles.push_back(p1);
                triangles.push_back(p2);
                triangles.push_back(p3);

            } else { //Else create two triangles
                index1_prev = lowest_bin_index[i - 1][j];
                index2_prev = lowest_bin_index[i - 1][j + 1];

                if (index1_prev == -1 || index2_prev == -1)
                    continue;

                Vector3f p1, p2, p3, p4;
                if (index1 != -1)
                    p1 = lowest_point_in_bin[i][j];
                if (index2 != -1)
                    p2 = lowest_point_in_bin[i][j + 1];
                if (index1_prev != -1)
                    p3 = lowest_point_in_bin[i - 1][j];
                if (index2_prev != -1)
                    p4 = lowest_point_in_bin[i - 1][j + 1];

                //First triangle
                if (index1 != -1 && index2 != -1 && index1_prev != -1)
                    triangles.push_back(p1);
                triangles.push_back(p2);
                triangles.push_back(p3);

                //Second triangle
                if (index2_prev != -1 && index2 != -1 && index1_prev != -1)
                    triangles.push_back(p3);
                triangles.push_back(p2);
                triangles.push_back(p4);
            }
        }
    }

    Vector3f rgb(0, 1, 0);
    markers.markers.push_back(triangle_markers(triangles, rgb, "triangle_mesh"));
}

vector<vector<Plane>> ground_removal2(vector<vector<float>> &r_matrix,
                                      vector<vector<float>> &theta_matrix,
                                      vector<vector<Vector3f>> &cart_matrix,
                                      const vector<float> &phis,
                                      vector<pair<int,int>> &after_ransac_indices,
                                      vector<list<int>> &r_matrix_nonzero_indices,
                                      vector<vector<float>> &intensity_matrix)
{
    //Parameters
    int points_per_bin = 1824 / n_bins;
    int span = 3;
    int layers = 7;
    int first_layer = -1;
    list<int> dummy_list;

    //Variables
    vector<vector<list<int>::iterator>> lowest_bin_index(16, vector<list<int>::iterator>(n_bins, dummy_list.end()));
    vector<vector<float>> lowest_bin_radius(16, vector<float>(n_bins, -__FLT_MAX__));
    vector<vector<bool>> object_points(16, vector<bool>(1824, false));
    Vector3f pos, color;
    int bin_index;
    list<int>::iterator it, it_start, it_end;
    float theta_bin_size, average;
    theta_bin_size = FOV_LIDAR / n_bins;
    visulize_bins(n_bins);
    for (int i = 0; i < 16; i++) {
        it_start = next(r_matrix_nonzero_indices[i].begin(), span);
        it_end = prev(r_matrix_nonzero_indices[i].end(), span);
        for (it = it_start; it != next(it_end, 1); ++it) {
            object_points[i][*it] = true;
            average = average_radial_distance(r_matrix, theta_matrix, it, span, i);
            bin_index = (int)(((theta_matrix[i][*it] + FOV_LIDAR) / (2 * FOV_LIDAR)) * n_bins);
            int closest_bin_index = (int)round((((theta_matrix[i][*it] + FOV_LIDAR) / (2 * FOV_LIDAR)) * n_bins));
            float theta_edge = -FOV_LIDAR + 2 * FOV_LIDAR / n_bins * closest_bin_index;

            //If to close to bin seperators skip
            if (abs(theta_matrix[i][*it] - theta_edge) < FOV_LIDAR / (n_bins * 8))
                continue;

            if (average > lowest_bin_radius[i][bin_index]) {
                lowest_bin_radius[i][bin_index] = average;
                lowest_bin_index[i][bin_index] = it;
            }
        }
    }

    ground_segment_cloud.clear();
    pcl::PointXYZRGB p;
    float r, r1, r2;
    Vector3f polar, cart, normal, p1, p2, p3;
    Vector4f plane;
    vector<Vector3f> triangles;
    vector<Vector3f> non_valid_triangles;
    vector<vector<Plane>> planes(layers, vector<Plane>(n_bins - 1));

    for (int i = 0; i < 16; i++) {
        if (i >= layers)
            continue;
        for (int j = 1; j < n_bins; j++) {
            // Create the plane
            Plane plane;

            it_start = lowest_bin_index[i][j - 1];
            it_end = lowest_bin_index[i][j];

            //Check if no valid points in either bin
            if (lowest_bin_radius[i][j - 1] == -__FLT_MAX__ || lowest_bin_radius[i][j] == -__FLT_MAX__ || it_start == dummy_list.end() || it_end == dummy_list.end() || r_matrix_nonzero_indices[i].size() == 0) {
                planes[i][j - 1] = plane;
                continue;
            }

            if (first_layer == -1)
                first_layer = i;

            //Find last valid plane
            Plane last_valid_plane;
            last_valid_plane.coeff << 0, 0, 1, lidar_height;
            last_valid_plane.midpoint << 0, 0, -lidar_height;
            int k = i;
            while (k > first_layer) {
                k--;
                if (planes[k][j - 1].valid) {
                    last_valid_plane = planes[k][j - 1];
                    break;
                }
            }

            setup_plane(plane, last_valid_plane, lowest_bin_radius[i][j - 1], lowest_bin_radius[i][j], phis[i], theta_matrix[i][*it_start], theta_matrix[i][*it_end], triangles, non_valid_triangles);
            planes[i][j - 1] = plane;

            if (j == 1)
                it_start = r_matrix_nonzero_indices[i].begin();

            if (j == n_bins - 1)
                it_end = r_matrix_nonzero_indices[i].end();

            for (it = it_start; it != it_end; ++it) {
                r = compute_radius(plane.coeff, theta_matrix[i][*it], phis[i]);

                //Verkar bara kolla avstånded till planet, men borde kolla om den ligger under
                object_points[i][*it] = abs(distance_to_plane(plane.coeff, cart_matrix[i][*it])) > dist_to_plane_threshold;
                polar << r, theta_matrix[i][*it], phis[i];
                cart = polar_to_cartesian(polar);
                p.x = cart.x();
                p.y = cart.y();
                p.z = cart.z();
                p.r = 255 * ((j + 1) % 2);
                p.g = 255 * ((j) % 2);
                p.b = 0;
                ground_segment_cloud.points.push_back(p);
            }
        }
    }

    Vector3f rgb(0, 0, 1);
    markers.markers.push_back(triangle_markers(triangles, rgb, "triangles"));
    rgb << 1, 0, 0;
    if (non_valid_triangles.size() > 0)
        markers.markers.push_back(triangle_markers(non_valid_triangles, rgb, "non_valid_triangles"));

    after_ransac_indices.clear();
    pcl::PointXYZI point;
    for (int j = 0; j < 1824; j++) {
        for (int i = 0; i < 16; i++) {
            if (object_points[i][j]) {
                after_ransac_indices.emplace_back(i, j);
                point.x = cart_matrix[i][j].x();
                point.y = cart_matrix[i][j].y();
                point.z = cart_matrix[i][j].z();
                point.intensity = intensity_matrix[i][j];
                
                after_ransac_cloud.push_back(point);
            }
        }
    }
    return planes;
}

vector<vector<Plane>> RANSAC_ground_removal(vector<vector<float>> &r_matrix,
                                      vector<vector<float>> &theta_matrix,
                                      vector<vector<Vector3f>> &cart_matrix,
                                      const vector<float> &phis,
                                      vector<pair<int,int>> &after_ransac_indices,
                                      vector<list<int>> &r_matrix_nonzero_indices,
                                      vector<vector<float>> &intensity_matrix)

{
    VectorXf plane_coeff;
    //Ransac
    pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZI>::Ptr
        model_p(new pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZI>(possible_ground_points));
    Eigen::Vector3f dir(0, 0, 1);
    model_p->setAxis(dir);
    model_p->setEpsAngle(PLANE_RANSAC_ANGLE_THRESHOLD);
    pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_p, PLANE_RANSAC_THRESHOLD);
    ransac.setMaxIterations(PLANE_RANSAC_MAX_ITERATIONS);
    ransac.setProbability(PLANE_RANSAC_PROBABILITY);
    ransac.computeModel();

    ransac.getModelCoefficients(plane_coeff);
    if (plane_coeff[2] < 0) {
        for (int i = 0; i < 4; i++)
            plane_coeff[i] = -plane_coeff[i];
    }

    //Visualize the Ransac plane
    markers.markers.push_back(visualize_ransac(plane_coeff[0], plane_coeff[1], plane_coeff[2], plane_coeff[3], PLANE_RANSAC_EXTRACT_THRESHOLD));

    //Extract points above plane
    after_ransac_indices.clear();
    after_ransac_cloud.clear();
    float temp = sqrt(plane_coeff[0] * plane_coeff[0] + plane_coeff[1] * plane_coeff[1] + plane_coeff[2] * plane_coeff[2]);
    pcl::PointXYZI p;

    vector<vector<bool>> object_points(16, vector<bool>(1824, false));
    for (size_t i = 0; i < r_matrix_nonzero_indices.size(); i++) {
        for (int j : r_matrix_nonzero_indices[i]) {
            p.x = cart_matrix[i][j].x();
            p.y = cart_matrix[i][j].y();
            p.z = cart_matrix[i][j].z();
            if (plane_coeff[0] * p.x + plane_coeff[1] * p.y + plane_coeff[2] * p.z + plane_coeff[3] < PLANE_RANSAC_EXTRACT_THRESHOLD * temp)
                continue;

            object_points[i][j] = true;
        }
    }

    pcl::PointXYZI point;
    for (int j = 0; j < 1824; j++) {
        for (int i = 0; i < 16; i++) {
            if (object_points[i][j]) {
                after_ransac_indices.emplace_back(i, j);
                point.x = cart_matrix[i][j].x();
                point.y = cart_matrix[i][j].y();
                point.z = cart_matrix[i][j].z();
                point.intensity = intensity_matrix[i][j];

                after_ransac_cloud.push_back(point);
            }
        }
    }

    Plane plane;
    plane.valid = true;
    plane.coeff = plane_coeff;
    plane.midpoint << 0, 0, 0;
    plane.center << 0, 0, 0;
    vector<vector<Plane>> planes(1, vector<Plane>(1, plane));

    return planes;
}

Vector3f alpha_beta(float w_s, float w_e, float p_s, float p_e, float h, float laser_angle)
{
    Eigen::Matrix2f A;
    A << cos(w_s), sin(w_s),
        cos(w_e), sin(w_e);

    Vector2f bb(h / (p_s * cos(laser_angle)) + tan(laser_angle), h / (p_e * cos(laser_angle)) + tan(laser_angle));

    Vector2f alpha_beta = (A.transpose() * A).ldlt().solve(A.transpose() * bb);
    float ab_den = sqrt(1 + alpha_beta[0] * alpha_beta[0] + alpha_beta[1] * alpha_beta[1]);
    float a = -alpha_beta[0] / ab_den;
    float b = -alpha_beta[1] / ab_den;
    float c = sqrt(1 - a * a - b * b);
    Vector3f abc(a, b, c);
    return abc;
}
