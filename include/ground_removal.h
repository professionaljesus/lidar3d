#ifndef GROUND_H
#define GROUND_H

#include "utils.h"
#include "visuals.h"

//STRUCTS
struct Line_segment
{
    int start;
    int end;
    Eigen::Vector3f normal;
    bool object;
};

struct Plane
{
    Eigen::Vector4f coeff;
    float azimuth_index_start;
    float azimuth_index_end;
    float max_rad;
    float min_rad;
    float theta_start;
    float theta_end;
    bool valid = false;
    Eigen::Vector3f center;
    Eigen::Vector3f midpoint;
};

void read_rosparams();

Eigen::VectorXf best_plane_from_points(const std::list<Eigen::Vector3f> &c);

std::vector<std::pair<int, int>> amz_ground_removal(std::vector<std::vector<float>> &r_matrix, std::vector<std::vector<float>> &theta_matrix, std::vector<std::vector<Eigen::Vector3f>> &cart_matrix);

Eigen::Vector3f alpha_beta(float w_s, float w_e, float p_s, float p_e, float h, float laser_angle);

void visulize_bins(int n_bins);

void setup_plane(Plane& plane, Plane& last_valid_plane, 
    const float r1, const float r2, const float inc,
    const float theta1, const float theta2,
    std::vector<Eigen::Vector3f>& triangles, std::vector<Eigen::Vector3f>& non_valid_triangles);

Eigen::VectorXf ground_removal(std::vector<std::vector<float>> &r_matrix, std::vector<std::vector<Eigen::Vector3f>> &cart_matrix, std::vector<std::pair<int, int>> &after_ransac_indices, std::vector<std::list<int>> &r_matrix_nonzero_indices);

float average_radial_distancest(std::vector<std::vector<float>> &r_matrix, std::vector<std::vector<float>> &theta_matrix, std::list<int>::iterator &it, int span, int i);

void mesh_ground_removal(std::vector<std::vector<float>> &r_matrix,
                                                std::vector<std::vector<float>> &theta_matrix,
                                                std::vector<std::vector<Eigen::Vector3f>> &cart_matrix,
                                                const std::vector<float> &phis,
                                                std::vector<std::pair<int,int>> &after_ransac_indices,
                                                std::vector<std::list<int>> &r_matrix_nonzero_indices);

std::vector<std::vector<Plane>> ground_removal2(std::vector<std::vector<float>> &r_matrix,
                                                std::vector<std::vector<float>> &theta_matrix,
                                                std::vector<std::vector<Eigen::Vector3f>> &cart_matrix,
                                                const std::vector<float> &phis,
                                                std::vector<std::pair<int,int>> &after_ransac_indices,
                                                std::vector<std::list<int>> &r_matrix_nonzero_indices,
                                                std::vector<std::vector<float>> &intensity_matrix );

std::vector<std::vector<Plane>> RANSAC_ground_removal(std::vector<std::vector<float>> &r_matrix,
                                                std::vector<std::vector<float>> &theta_matrix,
                                                std::vector<std::vector<Eigen::Vector3f>> &cart_matrix,
                                                const std::vector<float> &phis,
                                                std::vector<std::pair<int,int>> &after_ransac_indices,
                                                std::vector<std::list<int>> &r_matrix_nonzero_indices,
                                                std::vector<std::vector<float>> &intensity_matrix );


void spherical_ground_removal(std::vector<std::vector<float>> &r_matrix, std::vector<std::vector<float>> &theta_matrix, std::vector<std::vector<int>> &r_matrix_nonzero_indices, std::vector<std::vector<Eigen::Vector3f>> &cart_matrix, float h);

#endif
