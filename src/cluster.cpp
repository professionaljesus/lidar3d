#include "cluster.h"
#include "ground_removal.h"

using namespace std;
using namespace Eigen;

//EUCLIDEAN CLUSTERING - ADD THESE TO DYNAMIC PARAMETER LIST IF FINE TUNING EUCLIDEAN CLUSTERING
int CLUSTER_SIZE_UPPER_LIMIT = 1000;
int CLUSTER_SIZE_LOWER_LIMIT = 8;
float CLUSTER_DISTANCE_THRESHOLD = 0.20;
float INDEX_THRESHOLD = 3;
//STRING CLUSTERING - ADD THIS TO DYNAMIC PARAMETER LIST IF FINE TUNING STRING CLUSTERING
float MAN_DIST_THRESHOLD = 0.15f;
float OUTSIDE_CAMERA_FOV = 70.0;
int MIN_NBR_OF_LAYERS = 2;
int MIN_NBR_OF_POINTS = 3;
float CONE_MODEL_HEIGHT_MARGIN = 0.05;
float MATCH_THRESHOLD = 0.5;

//CONE CLASSIFICATION - ADD THIS TO DYNAMIC PARAMETER LIST IF FINE TUNING CLASSIFICATION
double CONE_INLIER_THRESHOLD = 0.03;

extern pcl::PointCloud<pcl::PointXYZRGB> cluster_cloud;
extern pcl::PointCloud<pcl::PointXYZRGB> cluster_with_reattached_ground_cloud;
extern visualization_msgs::MarkerArray markers;
extern Cone cone_def;
extern float lidar_height;
extern bool reattach;

void update_params(int uCLUSTER_SIZE_UPPER_LIMIT, int uCLUSTER_SIZE_LOWER_LIMIT, float uCLUSTER_DISTANCE_THRESHOLD,
                            float uINDEX_THRESHOLD, float uMAN_DIST_THRESHOLD, float uOUTSIDE_CAMERA_FOV,
                            int uMIN_NBR_OF_LAYERS, int uMIN_NBR_OF_POINTS, float uCONE_MODEL_HEIGHT_MARGIN,
                            float uMATCH_THRESHOLD, double uCONE_INLIER_THRESHOLD) {
    CLUSTER_SIZE_UPPER_LIMIT = uCLUSTER_SIZE_UPPER_LIMIT;
    CLUSTER_SIZE_LOWER_LIMIT = uCLUSTER_SIZE_LOWER_LIMIT;
    CLUSTER_DISTANCE_THRESHOLD = uCLUSTER_DISTANCE_THRESHOLD;
    INDEX_THRESHOLD = uINDEX_THRESHOLD;
    MAN_DIST_THRESHOLD = uMAN_DIST_THRESHOLD;
    OUTSIDE_CAMERA_FOV = uOUTSIDE_CAMERA_FOV;
    MIN_NBR_OF_LAYERS = uMIN_NBR_OF_LAYERS;
    MIN_NBR_OF_POINTS = uMIN_NBR_OF_POINTS;
    CONE_MODEL_HEIGHT_MARGIN = uCONE_MODEL_HEIGHT_MARGIN;
    MATCH_THRESHOLD = uMATCH_THRESHOLD;
    CONE_INLIER_THRESHOLD = uCONE_INLIER_THRESHOLD;
}

Vector3f cone_apex_from_ransac_plane(Vector3f point, VectorXf plane_coeff, float cone_height)
{
    Vector3f origo(0, 0, -plane_coeff[3] / plane_coeff[2]);
    Vector3f v = point - origo;
    Vector3f normal;
    for (int i = 0; i < 3; i++)
        normal[i] = plane_coeff[i];
    float dist = normal.dot(v);
    Vector3f projected_point = point - dist * normal;
    return projected_point + normal * cone_height;
}

Vector3f project_point_on_plane(Vector3f point, VectorXf plane_coeff)
{
    Vector3f origo(0, 0, -plane_coeff[3] / plane_coeff[2]);
    Vector3f v = point - origo;
    Vector3f normal;
    for (int i = 0; i < 3; i++)
        normal[i] = plane_coeff[i];
    float dist = normal.dot(v);
    Vector3f projected_point = point - dist * normal;
    return projected_point;
}

size_t calc_cluster_size(Cluster* cluster)
{
    size_t cluster_size = 0;

    for (const list<int>& l : cluster->points) {
        cluster_size += l.size();
    }
    return cluster_size;
}

bool valid_cluster(Cluster* cluster, vector<vector<Vector3f>>& cart_matrix, vector<vector<float>>& theta_matrix, int counter)
{
    static int cluster_counter = 0;
    bool big_cone = false;
    pcl::PointXYZRGB p;
    int i, j;
    for (size_t i = 0; i < cluster->points.size(); i++) {
        for (int j : cluster->points[i]) {

            Vector3f cart = cart_matrix[i][j];
            p.x = cart.x();
            p.y = cart.y();
            p.z = cart.z();
            p.r = 0;
            p.g = 0;
            p.b = 0;
            if (cluster_counter % 3 == 0)
                p.r = (cluster_counter * 16) % 200 + 55;
            if (cluster_counter % 3 == 1)
                p.g = (cluster_counter * 16) % 200 + 55;
            if (cluster_counter % 3 == 2)
                p.b = (cluster_counter * 16) % 200 + 55;
            cluster_cloud.points.push_back(p);

            pair<int, int> right = cluster->right;
            p.x = cart_matrix[right.first][right.second].x();
            p.y = cart_matrix[right.first][right.second].y();
            p.z = cart_matrix[right.first][right.second].z();
            p.r = 0;
            p.g = 0;
            p.b = 0;
            cluster_cloud.points.push_back(p);
        }
    }
    cluster_counter++;

    markers.markers.push_back(text_marker(cluster->center.x(), cluster->center.y(), cluster->center.z() + lidar_height, to_string(counter), 0.1, "cluster_number"));

    string text;
    if (cluster->lines < MIN_NBR_OF_LAYERS && abs(cluster->theta_center) > M_PI * OUTSIDE_CAMERA_FOV / 180.0) {
        text = "cluster->lines < 2 && abs(cluster->theta_center) > M_PI * 65.0 / 180.0";
        markers.markers.push_back(text_marker(cluster->center.x(), cluster->center.y(), cluster->center.z() + lidar_height, text, 0.1, "valid_cluster_debug"));
        return false;
    }

    size_t cluster_size = calc_cluster_size(cluster);

    if (cluster_size < 2) {
        text = "cluster->points.size() < 2";
        markers.markers.push_back(text_marker(cluster->center.x(), cluster->center.y(), cluster->center.z() + lidar_height, text, 0.1, "valid_cluster_debug"));
        return false;
    }

    //Not within camera field of view (70 degrees in either direction of center line)
    if (cluster_size < MIN_NBR_OF_POINTS && abs(cluster->theta_center) > M_PI * OUTSIDE_CAMERA_FOV / 180.0) {
        text = "cluster->points.size() < 3";
        markers.markers.push_back(text_marker(cluster->center.x(), cluster->center.y(), cluster->center.z() + lidar_height, text, 0.1, "valid_cluster_debug"));
        return false;
    }

    if (cluster->center.z() + lidar_height > cone_def.big_h + CONE_MODEL_HEIGHT_MARGIN || cluster->center.z() + lidar_height < -0.1) {
        text = "cluster->center.z() + lidar_height > cone_def.big_h + 0.05 || cluster->center.z() + lidar_height < -0.1";
        markers.markers.push_back(text_marker(cluster->center.x(), cluster->center.y(), cluster->center.z() + lidar_height, text, 0.1, "valid_cluster_debug"));
        return false;
    }

    pair<int, int>& top_point = cluster->top;
    pair<int, int>& bottom_point = cluster->bottom;
    pair<int, int>& left_point = cluster->left;
    pair<int, int>& right_point = cluster->right;

    float cluster_height = abs(cart_matrix[top_point.first][top_point.second].z() - cart_matrix[bottom_point.first][bottom_point.second].z());

    if (cluster_height > cone_def.h + CONE_MODEL_HEIGHT_MARGIN)
    {
        if (cluster_height < cone_def.big_h + CONE_MODEL_HEIGHT_MARGIN)
        {
            big_cone = true;
        } else {
            text = "cluster height >= cone_def.big_h + 0.05";
            markers.markers.push_back(text_marker(cluster->center.x(), cluster->center.y(), cluster->center.z() + lidar_height, text, 0.1, "valid_cluster_debug"));
            return false;
        }
    }

    //Maximum width of the large cones are 290mm
    if ((cart_matrix[left_point.first][left_point.second].head(2) - cart_matrix[right_point.first][right_point.second].head(2)).norm() > 0.305) {
        text = "(cart_matrix[left_point.first][left_point.second].head(2) - cart_matrix[right_point.first][right_point.second].head(2)).norm() > 0.305 \n"
            + to_string((cart_matrix[left_point.first][left_point.second].head(2) - cart_matrix[right_point.first][right_point.second].head(2)).norm());
        markers.markers.push_back(text_marker(cluster->center.x(), cluster->center.y(), cluster->center.z() + lidar_height, text, 0.1, "valid_cluster_debug"));
        return false;
    }

    int exp;
    if (big_cone) {
        exp = expected_points(cluster->radial_center, cone_def.big_d, cone_def.big_h);
    } else {
        exp = expected_points(cluster->radial_center, cone_def.d, cone_def.h);
    }

    if (exp == 0) {
        text = "exp = 0";
        markers.markers.push_back(text_marker(cluster->center.x(), cluster->center.y(), cluster->center.z() + lidar_height, text, 0.1, "valid_cluster_debug"));
        return false;
    }

    pair<float, float> expected;
    if (big_cone) {
        expected = expected_bounds(cluster->radial_center, cone_def.big_d, cone_def.big_h);
    } else {
        expected = expected_bounds(cluster->radial_center, cone_def.d, cone_def.h);
    }

    if ((float)cluster_size > 1.5 * expected.second) {
        text = "(float)cluster->points.size() > exp*1.5; expected = " + to_string(expected.second) + " true = " + to_string(cluster_size);
        markers.markers.push_back(text_marker(cluster->center.x(), cluster->center.y(), cluster->center.z() + lidar_height, text, 0.1, "valid_cluster_debug"));
        return false;
    }

    // What does this mean?? Changed it from 0.5 to 0.4...
    if ((float)cluster_size < 0.4 * expected.first) {
        text = "(float)cluster->points.size() < exp*0.4; expected = " + to_string(expected.first) + " true = " + to_string(cluster_size);
        markers.markers.push_back(text_marker(cluster->center.x(), cluster->center.y(), cluster->center.z() + lidar_height, text, 0.1, "valid_cluster_debug"));
        return false;
    }

    cluster->big_cone = big_cone;
    return true;
}

bool possible_cone_cluster(Cluster* cluster)
{
    //Check if centre is valid
    if (cluster->center.z() + lidar_height > cone_def.big_h + 0.05 || cluster->center.z() + lidar_height < -0.1) {
        string text = "Not possible cone cluster, cluster->center.z() + lidar_height > cone_def.big_h + 0.025|| cluster->center.z() + lidar_height < -0.1";
        markers.markers.push_back(text_marker(cluster->center.x(), cluster->center.y(), cluster->center.z() + 0.19, text, 0.1, "valid_cluster_debug"));
        return false;
    }

    return true;
}

vector<Cluster> string_clustering(vector<pair<int, int>>& after_ransac_indices,
    vector<list<int>>& r_matrix_nonzero_indices,
    vector<vector<float>>& r_matrix,
    vector<vector<float>>& theta_matrix,
    vector<vector<Vector3f>>& cart_matrix,
    vector<vector<Plane>> planes)
{
    vector<Cluster> clusters;
    cluster_with_reattached_ground_cloud.clear();
    list<Cluster> ongoing_clusters;

    int a, i, j, c;
    float r_diff, index_diff, azimuth_diff, alpha;
    float min_man_dist, manhattan_diff, min_r_diff, min_az_diff;
    float dist_on_plane;
    Cluster* closest_cluster;
    int closest_cluster_index;

    cluster_cloud.clear();

    int counter = 0;

    list<Cluster>::iterator ongoing_it;

    int ccc = 0;
    //Sorted updown first, very important
    for (pair<int, int> point : after_ransac_indices) {
        i = point.first;
        j = point.second;
        dist_on_plane = r_matrix[i][j] * cos(phis[i]);

        min_man_dist = __FLT_MAX__;
        min_r_diff = __FLT_MAX__;
        min_az_diff = __FLT_MAX__;
        if (ccc % 20 == 0)
            markers.markers.push_back(text_marker(cart_matrix[i][j].x(), cart_matrix[i][j].y(), cart_matrix[i][j].z() + lidar_height, to_string(ccc), 0.1, "index"));
        ccc++;

        for (ongoing_it = ongoing_clusters.begin(); ongoing_it != ongoing_clusters.end();) {
            //r_diff = abs(ongoing_it->radial_center - dist_on_plane);

            pair<int, int>& right_idx = ongoing_it->right;
            r_diff = abs(r_matrix[right_idx.first][right_idx.second] - dist_on_plane);
            azimuth_diff = abs(theta_matrix[i][right_idx.second] - theta_matrix[i][j]) * ongoing_it->radial_center;
            manhattan_diff = max(r_diff, azimuth_diff);

            if (manhattan_diff < min_man_dist) {
                min_man_dist = manhattan_diff;
                min_r_diff = r_diff;
                min_az_diff = azimuth_diff;
                closest_cluster = &(*ongoing_it);
            // If current point are to far away from the given cluster, mark this cluster as done, and continue. (Later points can't be part of this cluster either, as the points are sorted)
            } else if (azimuth_diff > 0.15) {       //Add as dynamic parameter if fine tuning clustering.
                reattach_ground(&(*ongoing_it), r_matrix, theta_matrix, cart_matrix, r_matrix_nonzero_indices);
                if (valid_cluster(&(*ongoing_it), cart_matrix, theta_matrix, counter++)) {
                    clusters.push_back(*ongoing_it);
                }
                ongoing_it = ongoing_clusters.erase(ongoing_it);

                continue;
            }

            ongoing_it++;
        }

        if (min_man_dist < MAN_DIST_THRESHOLD) {        //This should be added as dynamic param if debug/fine tune clustering necessary
            //Add point to the closest cluster and update center
            closest_cluster->points[i].push_back(j);
            size_t cluster_size = calc_cluster_size(closest_cluster);
            alpha = 1.0 / (float)cluster_size;
            closest_cluster->radial_center = (1 - alpha) * closest_cluster->radial_center + alpha * r_matrix[i][j];
            closest_cluster->theta_center = (1 - alpha) * closest_cluster->theta_center + alpha * theta_matrix[i][j];
            closest_cluster->center = (1 - alpha) * closest_cluster->center + alpha * cart_matrix[i][j];

            pair<int, int>& top_point = closest_cluster->top;
            pair<int, int>& bottom_point = closest_cluster->bottom;
            if (cart_matrix[i][j].z() > cart_matrix[top_point.first][top_point.second].z()) {
                closest_cluster->top = point;
            }

            if (bottom_point.first > point.first) {
                closest_cluster->bottom = point;
            }

            if (j > closest_cluster->right.second) {
                closest_cluster->right = point;
            }
            closest_cluster->lines = (closest_cluster->top.first - closest_cluster->bottom.first) + 1;
        
        // Quick check to see that the points arent to far away from the lidar plane
        } else if (cart_matrix[i][j].z() < 0.6 && cart_matrix[i][j].z() > -0.3) //Could be added to dynamic parameter list, if clustering is fine tuned.        
        { //Create new cluster
            Cluster cluster;
            vector<list<int>> p = vector<list<int>>(16);
            cluster.points = p;
            cluster.radial_center = dist_on_plane;
            cluster.theta_center = theta_matrix[i][j];
            cluster.center = cart_matrix[i][j];
            cluster.right = point;
            cluster.left = point;
            cluster.top = point;
            cluster.plane = &planes[0][0];
            cluster.bottom = point;
            cluster.lines = 1;
            cluster.points[i].push_back(j);
            cluster.color = -1;
            string text = to_string(min_man_dist) + "\n" + to_string(min_r_diff) + "\n" + to_string(min_az_diff);
            markers.markers.push_back(text_marker(cart_matrix[i][j].x(), cart_matrix[i][j].y(), cart_matrix[i][j].z() + lidar_height, text, 0.1, "cluster_created"));

            ongoing_clusters.push_back(cluster);
        }
    }
    for (ongoing_it = ongoing_clusters.begin(); ongoing_it != ongoing_clusters.end();) {
        reattach_ground(&(*ongoing_it), r_matrix, theta_matrix, cart_matrix, r_matrix_nonzero_indices);
        if (valid_cluster(&(*ongoing_it), cart_matrix, theta_matrix, counter++)) {
            clusters.push_back(*ongoing_it);
        }
        ongoing_it = ongoing_clusters.erase(ongoing_it);
    }

    cout << "Clusters: " << clusters.size() << endl;

    //Visualize the clusters
    Vector3f cart;
    int cluster_counter = 0;
    pcl::PointXYZRGB p;

    for (vector<Cluster>::iterator it = clusters.begin(); it != clusters.end();) {
        //Plane *plane;
        float dist;
        float min_dist = __FLT_MAX__;
        for (int i = 0; i < planes.size(); i++) {
            for (int j = 0; j < planes[i].size(); j++) {
                if (!planes[i][j].valid)
                    continue;
                dist = abs(planes[i][j].center.x() - it->center.x()) * abs(planes[i][j].center.x() - it->center.x()) + abs(planes[i][j].center.y() - it->center.y()) * abs(planes[i][j].center.y() - it->center.y()) + abs(planes[i][j].center.z() - it->center.z()) * abs(planes[i][j].center.z() - it->center.z());
                if (dist < min_dist) {
                    min_dist = dist;
                    it->plane = &planes[i][j];
                }
            }
        }
        VectorXf coeffs(4);
        for (int i = 0; i < 4; i++) {
            coeffs[i] = it->plane->coeff[i];
        }
        float match = classify_cluster(&(*it), cart_matrix, coeffs);

        if ((!it->big_cone && match < MATCH_THRESHOLD) || (it->big_cone && match < MATCH_THRESHOLD)) {
            clusters.erase(it);
        } else {
            it->probability = match;
            ++it;
        }
    }
    return clusters;
}

void reattach_ground(Cluster* cluster,
    vector<vector<float>>& r_matrix,
    vector<vector<float>>& theta_matrix,
    vector<vector<Vector3f>>& cart_matrix,
    vector<list<int>>& r_matrix_nonzero_indices)
{
    int i;

    if (cluster->bottom.first == 0 || reattach == false) {
        return;
    }

    i = cluster->bottom.first;
    vector<int> new_indices;
    int n_indices;
    float average_r = 0;
    float average_z = 0;
    bool attached_new_layer = true;

    float dr_dz = (cone_def.d / 2.0) / (cone_def.h + cone_def.apex_diff);
    float cluster_size = (float)calc_cluster_size(cluster);
    Vector3f old_center = cluster->center * cluster_size;
    Vector3f center(0, 0, 0);

    float min_angle = cluster->theta_center - 0.075 / cluster->radial_center;
    float max_angle = cluster->theta_center + 0.075 / cluster->radial_center;

    //0.075 = r*theta
    //0.075/r = theta

    while (i > 0 && attached_new_layer) {
        new_indices.clear();
        attached_new_layer = false;
        average_r = 0;
        average_z = 0;

        for (int j : cluster->points[i]) {
            average_r += r_matrix[i][j];
            average_z += cart_matrix[i][j].z();
        }

        float n_points = (float)cluster->points[i].size();
        average_r = average_r / n_points;
        average_z = average_z / n_points;

        n_indices = ((int)n_points + 4) / 2;

        for (int j : r_matrix_nonzero_indices[i - 1]) {
            //if (j < center_index - n_indices || j > center_index + n_indices)
            if (theta_matrix[i - 1][j] < min_angle || theta_matrix[i - 1][j] > max_angle)
                continue;

            float z_diff = average_z - cart_matrix[i - 1][j].z();
            float model_r = average_r - z_diff * dr_dz;

            if (abs(model_r - r_matrix[i - 1][j]) < 0.03) {
                cluster->points[i - 1].push_back(j);
                center += cart_matrix[i - 1][j];
                if (j < cluster->left.second) {
                    cluster->left = make_pair(i - 1, j);
                }
                if (j > cluster->right.second) {
                    cluster->right = make_pair(i - 1, j);
                }
                attached_new_layer = true;
            }
        }

        if (attached_new_layer) {
            cluster->bottom = make_pair(i - 1, cluster->points[i - 1].front());
            //Visualize
            pcl::PointXYZRGB p;
            for (int j : cluster->points[i - 1]) {
                p.x = cart_matrix[i - 1][j].x();
                p.y = cart_matrix[i - 1][j].y();
                p.z = cart_matrix[i - 1][j].z();
                p.r = 255;
                p.g = 255;
                p.b = 255;
                cluster_with_reattached_ground_cloud.points.push_back(p);
            }
            i--;
        }
    }
    float new_cluster_size = (float)calc_cluster_size(cluster);
    cluster->center = (old_center + center) / new_cluster_size;
    cluster->lines = (cluster->top.first - cluster->bottom.first) + 1;
}

float classify_cluster(Cluster* cluster, vector<vector<Vector3f>>& cart_matrix, VectorXf plane_coeff)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointXYZI p;
    for (size_t i = 0; i < cluster->points.size(); i++) {
        for (int j : cluster->points[i]) {
            p.x = cart_matrix[i][j].x();
            p.y = cart_matrix[i][j].y();
            p.z = cart_matrix[i][j].z();
            cloud->push_back(p);
        }
    }

    size_t n_points = calc_cluster_size(cluster);

    pcl::SampleConsensusModelCone<pcl::PointXYZI, pcl::Normal>::Ptr
        cone_model(new pcl::SampleConsensusModelCone<pcl::PointXYZI, pcl::Normal>(cloud));

    //Add random normals
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    for (size_t i = 0; i < n_points; i++) {
        cloud_normals->push_back(pcl::Normal(rand(), rand(), rand()));
    }
    cone_model->setInputNormals(cloud_normals);

    // Add 1/4th of cone diameter to find true midpoint of cone
    Vector2d mu(cluster->center.x(), cluster->center.y());

    float norm = mu.norm();
    float one_fourth_diameter = cone_def.d / 4.0;
    float cone_height = cone_def.h + cone_def.apex_diff;

    if (cluster->big_cone) {
        cone_height = cone_def.big_h + cone_def.big_apex_diff;
        one_fourth_diameter = cone_def.big_d / 4.0;
    }
    mu += one_fourth_diameter * mu / norm;

    cluster->center[0] = mu.x();
    cluster->center[1] = mu.y();

    Vector3f apex = cone_apex_from_ransac_plane(cluster->center, plane_coeff, cone_height);
    VectorXf model_coefficients(7);
    model_coefficients[0] = apex.x();
    model_coefficients[1] = apex.y();
    model_coefficients[2] = apex.z();
    model_coefficients[3] = -plane_coeff[0];
    model_coefficients[4] = -plane_coeff[1];
    model_coefficients[5] = -plane_coeff[2];
    model_coefficients[6] = 0.2;

    //Add point on plane and cone normal for later vizualisation
    Vector3f normal;
    for (int i = 0; i < 3; i++)
        normal[i] = plane_coeff[i];
    cluster->normal = normal;
    cluster->center_on_plane = project_point_on_plane(cluster->center, plane_coeff);

    //Get inliers
    pcl::IndicesPtr inliers(new vector<int>());
    cone_model->selectWithinDistance(model_coefficients, (float)CONE_INLIER_THRESHOLD, *inliers);

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud);

    int n_inliers = inliers->size();
    //float rate = (float)n_inliers / (float)n_points;

    vector<double> distances(cloud->size());
    cone_model->getDistancesToModel(model_coefficients, distances);

    double distance_sum = 0;
    int n = 0;
    for (const double d : distances) {
        distance_sum += min(d * d, CONE_INLIER_THRESHOLD * CONE_INLIER_THRESHOLD) / (CONE_INLIER_THRESHOLD * CONE_INLIER_THRESHOLD);
        n++;
    }
    distance_sum = distance_sum / n;
    double rate = 1.0 - distance_sum;

    Vector3f rgb(1 - rate, rate, 0);
    Vector3f p1, p2;
    p2 << plane_coeff[0], plane_coeff[1], plane_coeff[2];
    p2 = apex - cone_height * p2;
    markers.markers.push_back(arrow_marker(apex, p2, rgb));

    /*
    cone_model->setAxis(normal);
    cone_model->setEpsAngle(0);
    cone_model->computeModelCoefficients(*inliers,model_coefficients);

    rgb << 1,1,0;
    apex << model_coefficients[0],model_coefficients[1],model_coefficients[2];
    p2 << model_coefficients[3],model_coefficients[4],model_coefficients[5];;
    p2 = apex + cone_height * p2;
    markers.markers.push_back(arrow_marker(apex, p2, rgb, "arrows2"));
    */

    string num_text = to_string(rate);
    markers.markers.push_back(text_marker(model_coefficients[0], model_coefficients[1], model_coefficients[2] + 0.1, num_text, 0.05, "cone_probability"));
    return (float)rate;
}

vector<Cluster> euclidean_clustering(vector<pair<int, int>>& after_ransac_indices,
    vector<list<int>>& r_matrix_nonzero_indices,
    vector<vector<float>>& r_matrix,
    vector<vector<float>>& theta_matrix,
    vector<vector<Vector3f>>& cart_matrix,
    vector<vector<Plane>> planes)
{
    //CLUSTERING PARAMETERS
    int CLUSTER_SIZE_UPPER_LIMIT = 1000;
    int CLUSTER_SIZE_LOWER_LIMIT = 8;
    float CLUSTER_DISTANCE_THRESHOLD = 0.20;

    //Set up the point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointXYZI p;
    int i, j;
    for (pair<int, int> index : after_ransac_indices) {
        i = index.first;
        j = index.second;
        p.x = cart_matrix[i][j].x();
        p.y = cart_matrix[i][j].y();
        p.z = cart_matrix[i][j].z();
        cloud->push_back(p);
    }

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;

    ec.setClusterTolerance(CLUSTER_DISTANCE_THRESHOLD);
    ec.setMinClusterSize(CLUSTER_SIZE_LOWER_LIMIT);
    ec.setMaxClusterSize(CLUSTER_SIZE_UPPER_LIMIT);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);

    ec.extract(cluster_indices);

    vector<Cluster> clusters;
    Cluster cluster;
    Vector3f center;
    Vector3f polar_center;
    int cluster_size, lowest_i, highest_i, lowest_j, highest_j;
    //Loop over every list of cluster indices
    int counter = 0;
    for (const pcl::PointIndices current_cluster_indices : cluster_indices) {
        cluster_size = current_cluster_indices.indices.size();
        center << 0, 0, 0;
        cluster.points = vector<list<int>>(16);
        //Loop over the indices of the current cluster
        lowest_i = 10000;
        highest_i = -1;
        lowest_j = 10000;
        highest_j = -1;
        bool first_time = true;
        for (int a : current_cluster_indices.indices) {
            pair<int, int> index = after_ransac_indices[a];
            i = index.first;
            j = index.second;
            lowest_i = min(i, lowest_i);
            lowest_j = min(j, lowest_j);
            highest_i = max(i, highest_i);
            highest_j = max(j, highest_j);

            if (first_time) {
                cluster.right = index;
                cluster.left = index;
                cluster.top = index;
                cluster.bottom = index;
                first_time = false;
            }

            if (cart_matrix[i][j].z() > cart_matrix[cluster.top.first][cluster.top.second].z()) {
                cluster.top = index;
            }

            if (cart_matrix[i][j].z() < cart_matrix[cluster.bottom.first][cluster.bottom.second].z()) {
                cluster.bottom = index;
            }

            if (j > cluster.right.second) {
                cluster.right = index;
            }

            cluster.points[i].push_back(j);
            center += cart_matrix[i][j] / cluster_size;
        }
        //Create new cluster
        polar_center = cartesian_to_polar(center);
        cluster.radial_center = polar_center.x();
        cluster.theta_center = polar_center.y();
        cluster.center = center;
        cluster.big_cone = false;
        cluster.lines = highest_i - lowest_i + 1;
        cluster.points[i].push_back(j);
        cluster.color = -1;

        reattach_ground(&(cluster), r_matrix, theta_matrix, cart_matrix, r_matrix_nonzero_indices);
        if (valid_cluster(&cluster, cart_matrix, theta_matrix, counter++)) {
            clusters.push_back(cluster);
        }
    }

    for (vector<Cluster>::iterator it = clusters.begin(); it != clusters.end();) {
        //Plane *plane;
        float dist;
        float min_dist = __FLT_MAX__;
        for (int i = 0; i < planes.size(); i++) {
            for (int j = 0; j < planes[i].size(); j++) {
                if (!planes[i][j].valid)
                    continue;
                dist = abs(planes[i][j].center.x() - it->center.x()) * abs(planes[i][j].center.x() - it->center.x()) + abs(planes[i][j].center.y() - it->center.y()) * abs(planes[i][j].center.y() - it->center.y()) + abs(planes[i][j].center.z() - it->center.z()) * abs(planes[i][j].center.z() - it->center.z());
                if (dist < min_dist) {
                    min_dist = dist;
                    it->plane = &planes[i][j];
                }
            }
        }
        VectorXf coeffs(4);
        for (int i = 0; i < 4; i++) {
            coeffs[i] = it->plane->coeff[i];
        }
        float match = classify_cluster(&(*it), cart_matrix, coeffs);

        if ((!it->big_cone && match < 0.7) || (it->big_cone && match < 0.6)) {
            clusters.erase(it);
        } else {
            it->probability = match;
            ++it;
        }
    }
    return clusters;
}
