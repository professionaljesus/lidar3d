#ifndef UTILS_H
#define UTILS_H

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <Eigen/Dense>
#include <angles/angles.h>
#include <chrono>
#include <cmath>
#include <geometry_msgs/Point32.h>
#include <iostream>
#include <lfs_msgs/cone.h>
#include <lfs_msgs/cone_array.h>
#include <lfs_msgs/filter_parameters.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_cone.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <signal.h>
#include <std_msgs/Header.h>
#include <velodyne_msgs/VelodynePacket.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <velodyne_pointcloud/point_types.h>

#define print_vector(var)                                      \
    std::cout << #var << " length = " << (var.size()) << ": "; \
    for (auto a : var) {                                       \
        std::cout << fixed << a << ", ";                       \
    }                                                          \
    std::cout << endl                                          \
              << std::flush;

#define p(x) cout << #x << " = " << x << endl \
                  << std::flush;

#define print_list(var)                                        \
    std::cout << #var << " length = " << (var.size()) << ": "; \
    for (const auto& a : var) {                                \
        std::cout << fixed << a << ", ";                       \
    }                                                          \
    std::cout << endl                                          \
              << std::flush;
/**
 * Raw Velodyne packet constants and structures.
 */
static const int SIZE_BLOCK = 100;
static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

static const float ROTATION_RESOLUTION = 0.01f; // [deg]
static const uint16_t ROTATION_MAX_UNITS = 36000u; // [deg/100]

/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;

/** Special Defines for VLP16 support **/
static const int VLP16_FIRINGS_PER_BLOCK = 2;
static const int VLP16_SCANS_PER_FIRING = 16;
static const float VLP16_BLOCK_TDURATION = 110.592f; // [µs]
static const float VLP16_DSR_TOFFSET = 2.304f; // [µs]
static const float VLP16_FIRING_TOFFSET = 55.296f; // [µs]

static const int PACKET_SIZE = 1206;
static const int BLOCKS_PER_PACKET = 12;
static const int PACKET_STATUS_SIZE = 4;
static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

static const std::vector<float> phis = { -0.2617993877991494,
    -0.22689280275926285,
    -0.19198621771937624,
    -0.15707963267948966,
    -0.12217304763960307,
    -0.08726646259971647,
    -0.05235987755982989,
    -0.017453292519943295,
    0.017453292519943295,
    0.05235987755982989,
    0.08726646259971647,
    0.12217304763960307,
    0.15707963267948966,
    0.19198621771937624,
    0.22689280275926285,
    0.2617993877991494 };
/** \brief Raw Velodyne data block.
 *
 *  Each block contains data from either the upper or lower laser
 *  bank.  The device returns three times as many upper bank blocks.
 *
 *  use stdint.h types, so things work with both 64 and 32-bit machines
 */
typedef struct raw_block {
    uint16_t header; ///< UPPER_BANK or LOWER_BANK
    uint16_t rotation; ///< 0-35999, divide by 100 to get degrees
    uint8_t data[BLOCK_DATA_SIZE];
} raw_block_t;

/** used for unpacking the first two data bytes in a block
 *
 *  They are packed into the actual data stream misaligned.  I doubt
 *  this works on big endian machines.
 */
union two_bytes {
    uint16_t uint;
    uint8_t bytes[2];
};

/** \brief Raw Velodyne packet.
 *
 *  revolution is described in the device manual as incrementing
 *    (mod 65536) for each physical turn of the device.  Our device
 *    seems to alternate between two different values every third
 *    packet.  One value increases, the other decreases.
 *
 *  \todo figure out if revolution is only present for one of the
 *  two types of status fields
 *
 *  status has either a temperature encoding or the microcode level
 */
typedef struct raw_packet {
    raw_block_t blocks[BLOCKS_PER_PACKET];
    uint16_t revolution;
    uint8_t status[PACKET_STATUS_SIZE];
} raw_packet_t;

Eigen::Vector3f cartesian_to_polar(Eigen::Vector3f vec);

Eigen::Vector3f polar_to_cartesian(Eigen::Vector3f vec);

int points_at_distance(float dist, float width, float rad);

int expected_points(float dist, float w = 0.15, float h = 0.275, float h_r = 2.0 * M_PI / 180.0, float v_r = 0.2 * M_PI / 180.0);

std::pair<float, float> expected_bounds(float dist, float w = 0.15, float h = 0.275, float h_r = 2.0 * M_PI / 180.0, float v_r = 0.2 * M_PI / 180.0);

int expected_points2(float dist, float w = 0.065, float h = 0.275, float inc_rad = 0.035, float azi_rad = 0.00344);

int expected_lines(float dist, float h = 0.275, float h_r = 2.0 * M_PI / 180.0);

float expected_lines2(float dist, float h = 0.275, float inc_rad = 0.035);

float compute_radius(Eigen::Vector3f abc, float w, float h, float laser_angle);

float compute_radius(Eigen::Vector4f plane, float w, float laser_angle);

float expected_height(Eigen::Vector3f abc, float w, float laser_angle, float p);

Eigen::Vector4f fit_plane_to_3_points(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, const Eigen::Vector3f& p3);

float distance_to_plane(const Eigen::Vector4f& plane, const Eigen::Vector3f point);

#endif
