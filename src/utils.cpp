#include "utils.h"

float compute_radius(Eigen::Vector3f abc, float w, float h, float laser_angle)
{
    return -abc[2] * h / (std::cos(laser_angle) * (abc[0] * std::cos(w) + abc[1] * std::sin(w) + abc[2] * std::tan(laser_angle)));
}

float compute_radius(Eigen::Vector4f plane, float w, float laser_angle)
{
    return -plane[2] * -plane[3] / (std::cos(laser_angle) * -(plane[0] * std::cos(w) + plane[1] * std::sin(w) + plane[2] * std::tan(laser_angle)));
}

float expected_height(Eigen::Vector3f abc, float w, float laser_angle, float p)
{
    return p * (std::cos(laser_angle) * (abc[0] * std::cos(w) + abc[1] * std::sin(w) + abc[2] * std::tan(laser_angle))) / (-abc[2]);
}

int points_at_distance(float dist, float width, float rad)
{
    return (int)(2 * std::atan2(width / 2, dist) / rad);
}

int expected_points(float dist, float w, float h, float h_r, float v_r)
{
    float tanhr = 2 * dist * std::tan(h_r / 2);
    float tanvr = 2 * dist * std::tan(v_r / 2);
    return (int)(0.5 * (h / tanhr) * (w / tanvr));
}

std::pair<float, float> expected_bounds(float dist, float w, float h, float h_r, float v_r)
{
    float tanhr = 2 * dist * std::tan(h_r / 2);
    float tanvr = 2 * dist * std::tan(v_r / 2);
    return std::make_pair((0.5 * (h / tanhr) * (w / tanvr)), (0.5 * ((h / tanhr) + 1) * (w / tanvr)));
}

int expected_points2(float dist, float w, float h, float inc_rad, float azi_rad)
{
    float area_at_1m = w * h / (dist * dist);
    return (int)(area_at_1m / (inc_rad * azi_rad));
}

int expected_lines(float dist, float h, float h_r)
{
    float tanhr = 2 * dist * std::tan(h_r / 2);
    return (int)(h / tanhr);
}

float expected_lines2(float dist, float h, float h_r)
{
    return (int)(h / (dist * h_r));
}

Eigen::Vector4f fit_plane_to_3_points(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, const Eigen::Vector3f& p3)
{
    Eigen::Vector3f v1, v2;
    v1 = p2 - p1;
    v2 = p3 - p1;
    Eigen::Vector3f normal = v1.cross(v2);
    normal.normalize();
    float d = -(normal.dot(p1));
    Eigen::Vector4f plane(normal.x(), normal.y(), normal.z(), d);
    //if(normal.z()<0) for(int i = 0; i<4; i++) plane[i] = -plane[i];
    return plane;
}

float distance_to_plane(const Eigen::Vector4f& plane, const Eigen::Vector3f point)
{
    return (plane[0] * point[0] + plane[1] * point[1] + plane[2] * point[2] + plane[3]) / sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);
}

Eigen::Vector3f cartesian_to_polar(Eigen::Vector3f vec)
{
    Eigen::Vector3f polar;
    polar << vec.norm(), std::atan2(vec.y(), vec.x()), std::asin(vec.z() / (vec.norm()));
    return polar;
}

Eigen::Vector3f polar_to_cartesian(Eigen::Vector3f vec)
{
    Eigen::Vector3f cart;
    cart << vec.x() * std::cos(vec.z()) * std::cos(vec.y()), vec.x() * std::cos(vec.z()) * std::sin(vec.y()), vec.x() * std::sin(vec.z());
    return cart;
}
