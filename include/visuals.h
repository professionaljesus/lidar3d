#ifndef VISUALS_H
#define VISUALS_H

#include "utils.h"


visualization_msgs::Marker marker(float x, float y, float z, std::string ns = "cones");


visualization_msgs::Marker spehere_marker(Eigen::Vector3f pos, float size, Eigen::Vector3f rgb, std::string ns);


visualization_msgs::Marker cone_marker(Eigen::Vector3f pos, Eigen::Vector3f normal, Eigen::Vector3f rgb, std::string ns);


visualization_msgs::Marker text_marker(float x, float y, float z, std::string text, float scale, std::string ns);

visualization_msgs::Marker text_marker(Eigen::Vector3f p, std::string text, float scale, std::string ns);


visualization_msgs::Marker arrow_marker(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f rgb, std::string ns = "arrows");


visualization_msgs::Marker line_markers(const std::vector<Eigen::Vector3f> &lines, Eigen::Vector3f rgb, std::string ns);


visualization_msgs::Marker triangle_markers(const std::vector<Eigen::Vector3f> &triangles, Eigen::Vector3f rgb, std::string ns);


visualization_msgs::Marker visualize_ransac(float a, float b, float c, float d, float threshold, std::string ns = "ransac");


visualization_msgs::MarkerArray delete_markers();

#endif
