#ifndef MSG_PARSE_H
#define MSG_PARSE_H

#include "utils.h"

static const float FOV_LIDAR = M_PI * (140.0 / 180.0);

void read_sensor_message(const sensor_msgs::PointCloud2::ConstPtr& msg,
    std::vector<std::vector<float>>& r_matrix,
    std::vector<std::vector<float>>& theta_matrix,
    std::vector<std::list<int>>& r_matrix_nonzero_indices,
    std::vector<std::vector<Eigen::Vector3f>>& cart_matrix,
    std::vector<std::vector<float>>& p_trig);

void read_rawdata(const velodyne_msgs::VelodyneScan::ConstPtr& msg,
    std::vector<std::vector<float>>& r_matrix,
    std::vector<std::vector<float>>& theta_matrix,
    std::vector<std::list<int>>& r_matrix_nonzero_indices,
    std::vector<std::vector<Eigen::Vector3f>>& cart_matrix,
    std::vector<std::vector<float>>& intensity_matrix,
    std::vector<std::vector<float>>& p_trig);

bool build_timings();

#endif
