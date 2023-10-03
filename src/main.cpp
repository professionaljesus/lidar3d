#include "cluster.h"
#include "geometry_msgs/TransformStamped.h"
#include "ground_removal.h"
#include "message_parsing.h"
#include "tf2_ros/transform_listener.h"
#include "utils.h"
#include "visuals.h"
#include <fstream>
#include <iostream>
#include <ros/package.h>

using namespace std;
using namespace Eigen;

//*** PARAMETERS ***//

//SPHERICAL CLUSTERING
extern int MIN_SEGMENT_LENGTH;
extern float D_MAX_BREAKPOINTS;
extern float D_THRESHOLD;
extern float EPSILON;
extern float LAMBDA;
extern float ALPHA;
extern float ALPHA_RANGE;
extern float BETA;
extern float BETA_RANGE;
extern int MIN_CLUSTER_SIZE;
extern int AZIMUTH_INDEX_THRESHOLD;

lfs_msgs::cone_array cones;
ros::Publisher cone_publisher;

//VISUALIZATION PUBLISHERS
ros::Publisher vis_pub;
visualization_msgs::MarkerArray markers;

ros::Publisher raw_cloud_publisher;
ros::Publisher raw_cloud_xyzir_publisher;
ros::Publisher after_ransac_publisher;
ros::Publisher ground_segment_publisher;
ros::Publisher object_segment_publisher;
ros::Publisher azimuth_group_publisher;
ros::Publisher cluster_publisher;
ros::Publisher cluster_with_reattached_ground_publisher;

//GLOBAL VARIABLES
vector<vector<float>> r_matrix(16, vector<float>(1824));
vector<list<int>> r_matrix_nonzero_indices(16);
vector<vector<float>> theta_matrix(16, vector<float>(1824));
vector<vector<Vector3f>> cart_matrix(16, vector<Vector3f>(1824));
vector<vector<float>> intensity_matrix(16, vector<float>(1824));
vector<vector<float>> p_trig(3, vector<float>(1824));
vector<pair<int, int>> after_ransac_indices;
float h = 0.19;
Cone cone_def;
float lidar_height;
bool reattach = true;
bool euclidean = false;
bool use_RANSAC = false;
bool enable_logging = false;

// Ano other way to find ws path? :)))
string ws_path = ros::package::getPath("sensor_fusion") + "/../../../";
string log_filepath = ws_path + "logs/lidar_1.log";

//POINTCLOUDS FOR VIZUALIZATION
pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr raw_cloud_xyzir(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
pcl::PointCloud<pcl::PointXYZI>::Ptr possible_ground_points(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI> after_ransac_cloud;
pcl::PointCloud<pcl::PointXYZRGB> ground_segment_cloud;
pcl::PointCloud<pcl::PointXYZRGB> object_segment_cloud;
pcl::PointCloud<pcl::PointXYZRGB> azimuth_group_cloud;
pcl::PointCloud<pcl::PointXYZRGB> cluster_cloud;
pcl::PointCloud<pcl::PointXYZRGB> cluster_with_reattached_ground_cloud;
extern int marker_id;

void publish_cones(const vector<Cluster>& clusters)
{
    for (vector<Cluster>::const_iterator it = clusters.begin(); it != clusters.end(); it++) {
        lfs_msgs::cone cone;
        vector<geometry_msgs::Point32> keypoints(3);

        const pair<int, int>& left_idx = it->left;
        Vector3f& left_point = cart_matrix[left_idx.first][left_idx.second];

        const pair<int, int>& right_idx = it->right;
        Vector3f& right_point = cart_matrix[right_idx.first][right_idx.second];

        const pair<int, int>& top_idx = it->top;
        Vector3f& top_point = cart_matrix[top_idx.first][top_idx.second];

        const pair<int, int>& bottom_idx = it->bottom;
        Vector3f& bottom_point = cart_matrix[bottom_idx.first][bottom_idx.second];

        keypoints[0].x = left_point.x();
        keypoints[0].y = left_point.y();
        keypoints[0].z = bottom_point.z();

        keypoints[1].x = right_point.x();
        keypoints[1].y = right_point.y();
        keypoints[1].z = bottom_point.z();

        keypoints[2].x = it->center.x();
        keypoints[2].y = it->center.y();
        keypoints[2].z = top_point.z();

        for (int i = 0; i < 3; i++) {
            markers.markers.push_back(marker(keypoints[i].x, keypoints[i].y, keypoints[i].z, "keypoints"));
        }

        cone.point.x = it->center.x();
        cone.point.y = it->center.y();
        cone.point.z = it->center.z();
        cone.keypoints = keypoints;
        cone.cone_confidence = it->probability;
        cone.cone_class_prob.cone_classes = { 0.25, 0.25, 0.25, 0.25 };

        if (it->big_cone) {
            cone.cone_class_prob.cone_classes = { 0.1, 0.1, 0.1, 0.7 };
        } else if (it->color != -1) {
            float conf = max(0.7, 0.3 + (it->lines - 3) * 0.1);
            float inv_conf = (1.0 - conf) / 3.0;

            cone.cone_class_prob.cone_classes = { inv_conf, inv_conf, inv_conf, inv_conf };
            cone.cone_class_prob.cone_classes[it->color] = conf;
        }

        Vector3f rgb(0, 1, 0);
        if (it->color == 0) {
            rgb << 0, 0, 1;
        } else if (it->color == 1) {
            rgb << 1, 1, 0;
        } else if (it->big_cone) {
            rgb << 0.5, 1, 0;
        }

        markers.markers.push_back(cone_marker(it->center_on_plane, it->normal, rgb, "cones"));

        cones.cones.push_back(cone);
    }
    cone_publisher.publish(cones);
}

void publish_visualization_clouds()
{
    cluster_with_reattached_ground_publisher.publish(cluster_with_reattached_ground_cloud);
    cluster_publisher.publish(cluster_cloud);
    azimuth_group_publisher.publish(azimuth_group_cloud);
    raw_cloud_publisher.publish(*raw_cloud);
    raw_cloud_xyzir_publisher.publish(*raw_cloud_xyzir);
    ground_segment_publisher.publish(ground_segment_cloud);
    object_segment_publisher.publish(object_segment_cloud);
    after_ransac_publisher.publish(after_ransac_cloud);
}

void main_algorithm()
{

    //Ground removal with full mesh
    //mesh_ground_removal(r_matrix, theta_matrix, cart_matrix, phis, after_ransac_indices, r_matrix_nonzero_indices);

    vector<vector<Plane>> planes;
    if (use_RANSAC) {
        planes = RANSAC_ground_removal(r_matrix, theta_matrix, cart_matrix, phis, after_ransac_indices, r_matrix_nonzero_indices, intensity_matrix);
    } else {
        planes = ground_removal2(r_matrix, theta_matrix, cart_matrix, phis, after_ransac_indices, r_matrix_nonzero_indices, intensity_matrix);
    }

    vector<Cluster> clusters;
    if (euclidean) {
        clusters = euclidean_clustering(after_ransac_indices, r_matrix_nonzero_indices, r_matrix, theta_matrix, cart_matrix, planes);
        cout << "euclid" << endl;
    } else {
        clusters = string_clustering(after_ransac_indices, r_matrix_nonzero_indices, r_matrix, theta_matrix, cart_matrix, planes);
    }

    color_classification(clusters, cart_matrix, intensity_matrix);
    publish_cones(clusters);
    publish_visualization_clouds();

    visualization_msgs::MarkerArray del_markers = delete_markers();
    vis_pub.publish(del_markers);

    vis_pub.publish(markers);
}

void preperation(const std_msgs::Header& header)
{

    markers.markers.clear();
    marker_id = 0;

    for (list<int>& l : r_matrix_nonzero_indices) {
        l.clear();
    }
    after_ransac_indices.clear();

    cluster_with_reattached_ground_cloud.clear();
    after_ransac_cloud.clear();
    ground_segment_cloud.clear();
    object_segment_cloud.clear();
    azimuth_group_cloud.clear();
    cluster_cloud.clear();
    raw_cloud->clear();
    raw_cloud_xyzir->clear();
    possible_ground_points->clear();

    cones.header = header;
    cones.cones.clear();

    pcl_conversions::toPCL(header, after_ransac_cloud.header);
    pcl_conversions::toPCL(header, raw_cloud->header);
    pcl_conversions::toPCL(header, raw_cloud_xyzir->header);
    pcl_conversions::toPCL(header, ground_segment_cloud.header);
    pcl_conversions::toPCL(header, object_segment_cloud.header);
    pcl_conversions::toPCL(header, azimuth_group_cloud.header);
    pcl_conversions::toPCL(header, cluster_cloud.header);
    pcl_conversions::toPCL(header, cluster_with_reattached_ground_cloud.header);
}

chrono::time_point<chrono::high_resolution_clock> start;
void callback(const velodyne_msgs::VelodyneScan::ConstPtr& msg)
{
    // Log the time we acquired from VelodyneScan
    if (enable_logging) {
        uint64_t now = ros::Time::now().toNSec();
        ofstream ofs;
        ofs.open(log_filepath, std::ios::out | std::ios::app);
        ofs << now << "\t" << msg->header.stamp.toNSec() << std::endl;
        ofs.close();
    }
    // Show downtime and execution timings
    auto stop = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    cout << "Downtime: " << duration.count() / 1000 << " ms" << endl;
    start = stop;

    preperation(msg->header);
    read_rawdata(msg,
        r_matrix,
        theta_matrix,
        r_matrix_nonzero_indices,
        cart_matrix,
        intensity_matrix,
        p_trig);

    main_algorithm();

    // Show execution time
    stop = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    cout << "Execution time: " << duration.count() / 1000 << " ms" << endl;
    start = stop;
}

void sim_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // Show downtime and execution timings
    auto stop = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    cout << "Downtime: " << duration.count() / 1000 << " ms" << endl;
    start = stop;

    preperation(msg->header);

    read_sensor_message(msg,
        r_matrix,
        theta_matrix,
        r_matrix_nonzero_indices,
        cart_matrix,
        p_trig);
    main_algorithm();

    // Show execution time
    stop = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    cout << "Execution time: " << duration.count() / 1000 << " ms" << endl;
    start = stop;
}

void parameter_callback(const lfs_msgs::filter_parameters::ConstPtr& params)
{
    MIN_SEGMENT_LENGTH = params->MIN_SEGMENT_LENGTH;
    D_MAX_BREAKPOINTS = params->D_MAX_BREAKPOINTS;
    D_THRESHOLD = params->D_THRESHOLD;

    EPSILON = params->EPSILON;
    LAMBDA = params->LAMBDA;

    ALPHA = params->ALPHA;
    BETA = params->BETA;

    ALPHA_RANGE = params->ALPHA_RANGE;
    BETA_RANGE = params->BETA_RANGE;
}

void update_cluster_callback(const ros::TimerEvent& event) {
    int CLUSTER_SIZE_UPPER_LIMIT, CLUSTER_SIZE_LOWER_LIMIT, MIN_NBR_OF_LAYERS, MIN_NBR_OF_POINTS;
    float CLUSTER_DISTANCE_THRESHOLD, INDEX_THRESHOLD, MAN_DIST_THRESHOLD, OUTSIDE_CAMERA_FOV, CONE_MODEL_HEIGHT_MARGIN, MATCH_THRESHOLD, CONE_INLIER_THRESHOLD;

    std::string srv_namespace = "/parameter_server/lidar3d_cluster";


    // Fetch cameratranslate parameters. Throw error if any fails to fetch. Do we want to throw differently?
    if (ros::param::get(srv_namespace + "/CLUSTER_SIZE_UPPER_LIMIT", CLUSTER_SIZE_UPPER_LIMIT) &&
        ros::param::get(srv_namespace + "/CLUSTER_SIZE_LOWER_LIMIT", CLUSTER_SIZE_LOWER_LIMIT) &&
        ros::param::get(srv_namespace + "/MIN_NBR_OF_LAYERS", MIN_NBR_OF_LAYERS) &&
        ros::param::get(srv_namespace + "/MIN_NBR_OF_POINTS", MIN_NBR_OF_POINTS) &&
        ros::param::get(srv_namespace + "/CLUSTER_DISTANCE_THRESHOLD", CLUSTER_DISTANCE_THRESHOLD) &&
        ros::param::get(srv_namespace + "/INDEX_THRESHOLD", INDEX_THRESHOLD) &&
        ros::param::get(srv_namespace + "/MAN_DIST_THRESHOLD", MAN_DIST_THRESHOLD) &&
        ros::param::get(srv_namespace + "/OUTSIDE_CAMERA_FOV", OUTSIDE_CAMERA_FOV) &&
        ros::param::get(srv_namespace + "/CONE_MODEL_HEIGHT_MARGIN", CONE_MODEL_HEIGHT_MARGIN) &&
        ros::param::get(srv_namespace + "/MATCH_THRESHOLD", MATCH_THRESHOLD) &&
        ros::param::get(srv_namespace + "/CONE_INLIER_THRESHOLD", CONE_INLIER_THRESHOLD)){
    } else {
        ROS_ERROR("Lidar3d_cluster parameter for %s could not be loaded from parameter server", srv_namespace);
        throw std::invalid_argument("Missing intrinsic parameters");
    }

    //ROS_INFO("Updating cluster params");
    update_params(CLUSTER_SIZE_UPPER_LIMIT, CLUSTER_SIZE_LOWER_LIMIT, CLUSTER_DISTANCE_THRESHOLD, INDEX_THRESHOLD, MAN_DIST_THRESHOLD, OUTSIDE_CAMERA_FOV, MIN_NBR_OF_LAYERS, MIN_NBR_OF_POINTS, CONE_MODEL_HEIGHT_MARGIN, MATCH_THRESHOLD, CONE_INLIER_THRESHOLD);
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "raw_interp");
    ros::NodeHandle nh;

    build_timings();
    if (nh.getParam("/cone/h", cone_def.h)) {
        ROS_INFO("Cone height loaded from parameter server to %f", cone_def.h);
    } else {
        cone_def.h = 0.305;
    }

    if (nh.getParam("/cone/d", cone_def.d)) {
        ROS_INFO("Cone diameter loaded from parameter server to %f", cone_def.d);
    } else {
        cone_def.d = 0.16;
    }

    if (nh.getParam("/cone/apex_diff", cone_def.apex_diff)) {
        ROS_INFO("Cone apex height loaded from parameter server to %f", cone_def.apex_diff);
    } else {
        cone_def.apex_diff = 0.10;
    }

    if (nh.getParam("/cone/big_h", cone_def.big_h)) {
        ROS_INFO("Big cone height loaded from parameter server to %f", cone_def.big_h);
    } else {
        cone_def.big_h = 0.505;
    }

    if (nh.getParam("/cone/big_d", cone_def.big_d)) {
        ROS_INFO("Big cone diameter loaded from parameter server to %f", cone_def.big_d);
    } else {
        cone_def.big_d = 0.22;
    }

    if (nh.getParam("/cone/big_apex_diff", cone_def.big_apex_diff)) {
        ROS_INFO("Big cone apex diff loaded from parameter server to %f", cone_def.big_apex_diff);
    } else {
        cone_def.big_apex_diff = 0.16;
    }

    if (nh.getParam("/lidar/height", lidar_height)) {
        ROS_INFO("Lidar height loaded from parameter server to %f", lidar_height);
    } else {
        lidar_height = 0.2;
        ROS_INFO("Lidar height loaded from harcoded value to %f", lidar_height);
    }

    if (nh.getParam("/reattach", reattach)) {
        ROS_INFO("Reattach set to: %s", reattach ? "True" : "False");
    }

    if (nh.getParam("/euclidean", euclidean)) {
        ROS_INFO("Euclidean set to: %s", euclidean ? "True" : "False");
    }

    if (nh.getParam("/use_RANSAC", use_RANSAC)) {
        ROS_INFO("use_RANSAC set to: %s", use_RANSAC ? "True" : "False");
    }
    if (nh.getParam("/logging/enabled", enable_logging)) {
        ROS_INFO("Enable_logging set to: %s", enable_logging ? "True" : "False");
    }

    read_rosparams();

    // Create/clear file for logging.
    ROS_INFO("Clearing files for logging.. ");
    ofstream ofs;
    ofs.open(log_filepath, std::ofstream::out | std::ofstream::trunc);
    ofs << "Timestamp now\tLiDAR" << std::endl;
    ofs.close();
    /*
    try{
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transform = tfBuffer.lookupTransform("base_link", "velodyne",ros::Time());
        lidar_height = transform.transform.translation.y;
        ROS_INFO("Lidar height loaded from tf_tree to %f", lidar_height);
    }
    catch(tf2::TransformException &ex){
        ROS_WARN("%s",ex.what());
        lidar_height = 0.2;
        ROS_INFO("Lidar height loaded from harcoded value to %f", lidar_height);
    }
 */
    //ros::Subscriber sub = nh.subscribe<velodyne_msgs::VelodyneScan>("/velodyne_synced", 1, callback);
    ros::Subscriber sub = nh.subscribe<velodyne_msgs::VelodyneScan>("/velodyne_packets", 1, callback);
    ros::Subscriber sub_simulator = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, sim_callback);
    ros::Subscriber parameters_sub = nh.subscribe<lfs_msgs::filter_parameters>("/filter_parameters", 1, parameter_callback);

    ros::Timer timer;

    ROS_INFO("Bind update cluster..");
    timer = nh.createTimer(ros::Duration(0.5), boost::bind(update_cluster_callback, _1));

    raw_cloud_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/spherical/pointcloud_raw", 1);
    raw_cloud_xyzir_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/spherical/pointcloud_raw_xyzir", 1);
    after_ransac_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/spherical/after_ransac", 1);

    ground_segment_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/spherical/ground_segments", 1);
    object_segment_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/spherical/object_segments", 1);
    azimuth_group_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/spherical/azimuth_groups", 1);

    cluster_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/spherical/cluster_cloud", 1);
    cluster_with_reattached_ground_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/spherical/clusters_with_reattached_ground", 1);

    vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/lidar3d/debug", 1);
    cone_publisher = nh.advertise<lfs_msgs::cone_array>("/lidar3d/cones", 1);
    ros::spin();
}
