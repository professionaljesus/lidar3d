#ifndef CLUSTER_H
#define CLUSTER_H

#include "ground_removal.h"
#include "utils.h"
#include "visuals.h"

struct Cluster {
    float radial_center;
    float theta_center;
    std::vector<std::list<int>> points;
    std::pair<int, int> top;
    std::pair<int, int> bottom;
    std::pair<int, int> left;
    std::pair<int, int> right;
    int lines;
    float probability;
    Eigen::Vector3f center;
    Eigen::Vector3f center_on_plane;
    Eigen::Vector3f normal;
    Eigen::Vector3f keypoints[3];
    bool big_cone;
    int color;
    Plane* plane;
};

struct Cone {
    float h;
    float d;
    float apex_diff;
    float big_h;
    float big_d;
    float big_apex_diff;
};

void update_params(int uCLUSTER_SIZE_UPPER_LIMIT, int uCLUSTER_SIZE_LOWER_LIMIT, float uCLUSTER_DISTANCE_THRESHOLD,
                            float uINDEX_THRESHOLD, float uMAN_DIST_THRESHOLD, float uOUTSIDE_CAMERA_FOV,
                            int uMIN_NBR_OF_LAYERS, int uMIN_NBR_OF_POINTS, float uCONE_MODEL_HEIGHT_MARGIN,
                            float uMATCH_THRESHOLD, double uCONE_INLIER_THRESHOLD);

std::size_t calc_cluster_size(Cluster* cluster);

Eigen::Vector3f cone_apex_from_ransac_plane(Eigen::Vector3f point, Eigen::VectorXf plane_coeff, float cone_height);

Eigen::Vector3f project_point_on_plane(Eigen::Vector3f point, Eigen::VectorXf plane_coeff);

float classify_cluster(Cluster* cluster,
    std::vector<std::vector<Eigen::Vector3f>>& cart_matrix,
    Eigen::VectorXf plane_coeff);

void reattach_ground(Cluster* cluster,
    std::vector<std::vector<float>>& r_matrix,
    std::vector<std::vector<float>>& theta_matrix,
    std::vector<std::vector<Eigen::Vector3f>>& cart_matrix,
    std::vector<std::list<int>>& r_matrix_nonzero_indices);

bool valid_cluster(Cluster* cluster, std::vector<std::vector<Eigen::Vector3f>>& cart_matrix, std::vector<std::vector<float>>& theta_matrix, int counter);

bool possible_cone_cluster(Cluster* cluster);

std::vector<Cluster> string_clustering(std::vector<std::pair<int, int>>& after_ransac_indices,
    std::vector<std::list<int>>& r_matrix_nonzero_indices,
    std::vector<std::vector<float>>& r_matrix,
    std::vector<std::vector<float>>& theta_matrix,
    std::vector<std::vector<Eigen::Vector3f>>& cart_matrix,
    std::vector<std::vector<Plane>> planes);

std::vector<Cluster> euclidean_clustering(std::vector<std::pair<int, int>>& after_ransac_indices,
    std::vector<std::list<int>>& r_matrix_nonzero_indices,
    std::vector<std::vector<float>>& r_matrix,
    std::vector<std::vector<float>>& theta_matrix,
    std::vector<std::vector<Eigen::Vector3f>>& cart_matrix,
    std::vector<std::vector<Plane>> planes);

void color_classification(std::vector<Cluster>& clusters,
    std::vector<std::vector<Eigen::Vector3f>>& cart_matrix,
    std::vector<std::vector<float>>& intensity_matrix);
#endif
