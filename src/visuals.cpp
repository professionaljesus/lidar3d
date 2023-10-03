#include "visuals.h"
#include "utils.h"

using namespace std;

int marker_id = 0;
visualization_msgs::Marker marker(float x, float y, float z, string ns)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = marker_id++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 1.0; // - (float)(marker_id%3)/2;
    marker.color.g = 0.0;
    marker.color.b = 0; //(float)(marker_id%3)/2;
    marker.color.a = 0.7;
    marker.lifetime = ros::Duration(0);

    return marker;
}

visualization_msgs::Marker spehere_marker(Eigen::Vector3f pos, float size, Eigen::Vector3f rgb, string ns)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = marker_id++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pos.x();
    marker.pose.position.y = pos.y();
    marker.pose.position.z = pos.z();
    marker.pose.orientation.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;
    marker.color.r = rgb.x();
    marker.color.g = rgb.y();
    marker.color.b = rgb.z();
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0);

    return marker;
}

visualization_msgs::Marker cone_marker(Eigen::Vector3f pos, Eigen::Vector3f normal, Eigen::Vector3f rgb, string ns)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = marker_id++;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.mesh_resource = "package://lfs_msgs/meshes/cone.dae";
    marker.pose.position.x = pos.x();
    marker.pose.position.y = pos.y();
    marker.pose.position.z = pos.z();
    marker.pose.orientation.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.r = rgb.x();
    marker.color.g = rgb.y();
    marker.color.b = rgb.z();
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0);

    return marker;
}

visualization_msgs::Marker text_marker(Eigen::Vector3f p, string text, float scale, string ns)
{
    return text_marker(p.x(), p.y(), p.z(), text, scale, ns);
}

visualization_msgs::Marker text_marker(float x, float y, float z, string text, float scale, string ns)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = marker_id++;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.text = text;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0;
    marker.scale.y = 0;
    marker.scale.z = scale;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0);

    return marker;
}

visualization_msgs::Marker arrow_marker(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f rgb, string ns)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = marker_id++;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    geometry_msgs::Point p;
    p.x = p2.x();
    p.y = p2.y();
    p.z = p2.z();
    marker.points.push_back(p);
    p.x = p1.x();
    p.y = p1.y();
    p.z = p1.z();
    marker.points.push_back(p);

    marker.pose.orientation.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.02;
    marker.scale.y = 0.13;
    marker.scale.z = 0.323;
    marker.color.r = rgb.x();
    marker.color.g = rgb.y();
    marker.color.b = rgb.z();
    marker.color.a = 0.5;
    marker.lifetime = ros::Duration(0);

    return marker;
}

visualization_msgs::Marker line_markers(const std::vector<Eigen::Vector3f>& lines, Eigen::Vector3f rgb, string ns)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = marker_id++;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    for (int i = 0; i < lines.size(); i++) {
        geometry_msgs::Point p;
        p.x = lines[i].x();
        p.y = lines[i].y();
        p.z = lines[i].z();
        marker.points.push_back(p);
    }

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;

    marker.pose.orientation.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.02;

    marker.color.r = rgb.x();
    marker.color.g = rgb.y();
    marker.color.b = rgb.z();
    marker.color.a = 0.8;

    marker.lifetime = ros::Duration(0);
    return marker;
}

visualization_msgs::Marker triangle_markers(const std::vector<Eigen::Vector3f>& triangles, Eigen::Vector3f rgb, string ns)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = marker_id++;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    for (int i = 0; i < triangles.size(); i++) {
        geometry_msgs::Point p;
        p.x = triangles[i].x();
        p.y = triangles[i].y();
        p.z = triangles[i].z();
        marker.points.push_back(p);
    }

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;

    marker.pose.orientation.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;

    marker.color.r = rgb.x();
    marker.color.g = rgb.y();
    marker.color.b = rgb.z();
    marker.color.a = 0.8;

    marker.lifetime = ros::Duration(0);
    return marker;
}

visualization_msgs::Marker visualize_ransac(float a, float b, float c, float d, float threshold, string ns)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = marker_id++;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    float temp = sqrt(a * a + b * b + c * c);
    float nx = a / temp;
    float ny = b / temp;
    float nz = c / temp;

    float cx = 0 * nz - 0 * ny;
    float cy = 0 * nx - 1 * nz;
    float cz = 1 * ny - 0 * nx;

    Eigen::Vector3f from(0, 0, 1);
    Eigen::Vector3f to(nx, ny, nz);
    Eigen::Quaternionf q = Eigen::Quaternionf::FromTwoVectors(from, to);

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = -d / c;
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = 15;
    marker.scale.y = 15;
    marker.scale.z = threshold * 2;
    marker.color.r = 1.0; // - (float)(marker_id%3)/2;
    marker.color.g = 0.0;
    marker.color.b = 1.0; //(float)(marker_id%3)/2;
    marker.color.a = 0.7;
    marker.lifetime = ros::Duration(0);

    return marker;
}

visualization_msgs::MarkerArray delete_markers()
{
    visualization_msgs::MarkerArray markers2;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.action = 3;
    markers2.markers.push_back(marker);
    return markers2;
}
