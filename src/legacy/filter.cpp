#include <ros/ros.h>
#include <chrono> 
#include <iostream>
#include <ros/package.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_cone.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <velodyne_pointcloud/point_types.h>
#include <cmath>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <geometry_msgs/Point32.h>

#include "visuals.h"
//Visualization
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"


//ROS
#include <lfs_msgs/filter_parameters.h>
#include <lfs_msgs/cone.h>
#include <lfs_msgs/cone_array.h>


ros::Publisher cloud_with_ground_removed_publisher;
ros::Publisher cloud_after_ransac_publisher;
ros::Publisher cone_publisher;
ros::Publisher cones_in_cloud_publisher;
ros::Publisher cluster_with_inliers_publisher;
ros::Publisher vis_pub;

using namespace std;
using namespace Eigen;

visualization_msgs::MarkerArray markers;
extern int marker_id;

//*** PARAMETERS ***//

//RANSAC
bool RANSAC = true;
bool REATTACH = false;
float PLANE_RANSAC_TRESHHOLD = 0.005;
float PLANE_RANSAC_EXTRACT_TRESHHOLD = 0.03;
float PLANE_RANSAC_ANGLE_TRESHHOLD = 0.1;
float PLANE_RANSAC_PROBABILITY = 0.99;
int PLANE_RANSAC_MAX_ITERATIONS = 100;

//CLUSTERING
int CLUSTER_SIZE_UPPER_LIMIT = 1000;
int CLUSTER_SIZE_LOWER_LIMIT = 8;
float CLUSTER_DISTANCE_THRESHHOLD = 0.20;


int points_at_distance(float dist, float width, float rad)
{
    return (int)(2*atan2(width/2,dist)/rad);
}

bool verify_cone_model(VectorXf coeff){
    return (coeff[2]>0.1 && coeff[2]<0.23&& coeff[5]<0);
}

float classify_cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster, Eigen::Vector3f apex, visualization_msgs::MarkerArray& markers){
    cout << "Received cluster with size "<< cluster->size() << endl;

    int n_points = cluster->size();

    pcl::SampleConsensusModelCone<pcl::PointXYZI,pcl::Normal>::Ptr
        cone_model (new pcl::SampleConsensusModelCone<pcl::PointXYZI,pcl::Normal> (cluster));
    

    //Add random normals
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    for(int i = 0; i<n_points; i++){
        cloud_normals->push_back (pcl::Normal (rand (), rand (), rand ()));
    }
    cone_model->setInputNormals(cloud_normals);

    Eigen::Vector3f normal(0,0,-1);
    cone_model->setAxis(normal);
    cone_model->setEpsAngle(0.1);

    Eigen::VectorXf model_coefficients(7);
    model_coefficients[0] = apex.x();
    model_coefficients[1] = apex.y();
    model_coefficients[2] = apex.z();
    model_coefficients[3] = 0;
    model_coefficients[4] = 0;
    model_coefficients[5] = -1;
    model_coefficients[6] = 0.2;

    if(!verify_cone_model(model_coefficients)){
        cluster->clear();
        return 0;
    }

    //Get inliers
    pcl::IndicesPtr inliers(new vector<int>());
    cone_model->selectWithinDistance(model_coefficients, 0.02, *inliers);

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cluster);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cluster);

    
    //cone_model->selectWithinDistance(model_coefficients, 0.02, *inliers);
    int n_inliers = inliers->size();



    cout << "Found "<< n_inliers << " inliers" <<endl;
    float rate = (float)n_inliers / (float)n_points;
    cout << "Rate: "<< rate << endl << endl;

    Eigen::Vector3f rgb1(0,0,1);
    Eigen::Vector3f rgb2(1-rate,rate,0);
    Eigen::Vector3f p2(apex.x(),apex.y(),apex.z()-0.34);
    
    /*
    Eigen::VectorXf optimized_model_coefficients(7);
    cone_model->optimizeModelCoefficients(*inliers, model_coefficients, optimized_model_coefficients);
    Eigen::Vector3f p3(optimized_model_coefficients[0],optimized_model_coefficients[1],optimized_model_coefficients[2]);
    float sign = 0.34;
    if(optimized_model_coefficients[5]>0){
        sign = -sign;
    }
    Eigen::Vector3f p4(optimized_model_coefficients[0]+optimized_model_coefficients[3]*sign,
        optimized_model_coefficients[1]+optimized_model_coefficients[4]*sign,optimized_model_coefficients[2]+optimized_model_coefficients[5]*sign);
    */

    markers.markers.push_back(arrow_marker(apex,p2,rgb2));
    //markers.markers.push_back(arrow_marker(p3,p4,rgb2));
    string num_text = to_string(rate);
    markers.markers.push_back(text_marker(model_coefficients[0],model_coefficients[1],model_coefficients[2] + 0.1, num_text.substr(0,num_text.find(".")+3), 0.05, "cone_probability" ));
    return rate;
}

void ground_removal(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud){
    if(RANSAC){
        pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZI>::Ptr
            model_p (new pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZI> (cloud));
        Eigen::Vector3f dir(0,0,1);
        model_p->setAxis(dir);
        model_p->setEpsAngle(PLANE_RANSAC_ANGLE_TRESHHOLD);
        pcl::RandomSampleConsensus<pcl::PointXYZI> ransac (model_p, PLANE_RANSAC_TRESHHOLD);
        ransac.setMaxIterations(PLANE_RANSAC_MAX_ITERATIONS);
        ransac.setProbability(PLANE_RANSAC_PROBABILITY);
        ransac.computeModel();

        Eigen::VectorXf plane_coeff;
        ransac.getModelCoefficients(plane_coeff);
        markers.markers.push_back(visualize_ransac(plane_coeff[0],plane_coeff[1],plane_coeff[2],plane_coeff[3], PLANE_RANSAC_EXTRACT_TRESHHOLD));
        cout << "z: " << plane_coeff[2] << endl;
        if(plane_coeff[2]<0){
            for(int i = 0; i<4; i++) plane_coeff[i] = - plane_coeff[i];
        }
        pcl::IndicesPtr inliers(new vector<int>());
        model_p->selectWithinDistance (plane_coeff, PLANE_RANSAC_EXTRACT_TRESHHOLD, *inliers);
        if(!REATTACH){
            /*
            pcl::ExtractIndices<pcl::PointXYZI> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*cloud);
            cloud_after_ransac_publisher.publish(*cloud);
            */
            
            pcl::PointCloud<pcl::PointXYZI>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZI>);
            new_cloud->header = cloud->header;
            float temp = sqrt(plane_coeff[0]*plane_coeff[0]+plane_coeff[1]*plane_coeff[1]+plane_coeff[2]*plane_coeff[2]);
            for(pcl::PointXYZI p : cloud->points){
                if(plane_coeff[0]*p.x + plane_coeff[1]*p.y + plane_coeff[2]*p.z + plane_coeff[3]> PLANE_RANSAC_EXTRACT_TRESHHOLD * temp){
                    new_cloud->push_back(p);
                }
            }
            cloud = new_cloud;
            cloud_after_ransac_publisher.publish(*cloud);
        }else{
            pcl::IndicesPtr outliers(new vector<int>());
            int j;
            int k = 0;
            for(int i = 0; i<cloud->size(); i++){
                j = inliers->at(k);
                if(i == j){
                    k++;
                }else{
                    outliers->push_back(i);
                }
                
            }
            pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
            tree->setInputCloud(cloud);
            double radius = 0.2;
            vector<vector<int>> pointIdxRadiusSearch;
            vector<vector<float>> pointRadiusSquaredDistance;
            tree->radiusSearch(*cloud,*outliers,radius,pointIdxRadiusSearch,pointRadiusSquaredDistance);
            
            set<int> res;
            for(int i = 0; i<inliers->size(); i++){
                res.insert(inliers->at(i));
            }
            for(vector<int> pointIndices: pointIdxRadiusSearch){
                for(int i = 0; i<pointIndices.size(); i++){
                    res.erase(pointIndices[i]);
                }
            }
            inliers->clear();
            for(int i : res){
                inliers->push_back(i);
            }

            //pcl::PointCloud<pcl::PointXYZI>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::ExtractIndices<pcl::PointXYZI> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*cloud);
        }
        /*
        vector<float> pointRadiusSquaredDistance;
        pcl::PointIndices indices;
        float radius = 0.2;
        float temp = sqrt(plane_coeff[0]*plane_coeff[0]+plane_coeff[1]*plane_coeff[1]+plane_coeff[2]*plane_coeff[2]);
        for(pcl::PointXYZI p : new_cloud->points){
            //If the distance to the plane is larger than a treshold dont consider this point
            if(plane_coeff[0]*p.x + plane_coeff[1]*p.y + plane_coeff[2]*p.z + plane_coeff[3]> 0.2 * temp){
                continue;
            }
            if(tree->radiusSearch(p, radius, indices, pointRadiusSquaredDistance)>0){
                for (int i : indices.indices){

                }
            }

        }
        */
        //cloud = new_cloud;
    }else{
        pcl::PointCloud<pcl::PointXYZI>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZI>);
        new_cloud->header = cloud->header;
        new_cloud->height = 1;
        new_cloud->is_dense = true;
        int width = 0;

        for(pcl::PointXYZI pt1 : cloud->points)
        {
            if( pt1.z> -0.16)
            {
                new_cloud->push_back(pt1);
                width++;
            }
        }
        cloud = new_cloud;
        cloud->width = width;
        cloud_after_ransac_publisher.publish(*cloud);
    }

}

void reattach_ground(pcl::PointCloud<pcl::PointXYZI>::Ptr& raw, pcl::PointCloud<pcl::PointXYZI>::Ptr& ransac, float radius){
    pcl::PointCloud<pcl::PointXYZI>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    new_cloud->header = raw->header;
    int width = 0;
    new_cloud->height = 1;
    new_cloud->is_dense = true;

    for(pcl::PointXYZI pt1 : raw->points)
    {
        for(pcl::PointXYZI pt2 : ransac->points)
        {
            if( (pt1.x - pt2.x)*(pt1.x - pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y) < radius*radius)
            {
                new_cloud->push_back(pt1);
                width++;
                break;
            }
        }
    }    
    ransac = new_cloud;
    ransac->width = width;
    cloud_with_ground_removed_publisher.publish(*new_cloud);
    
}

void clean_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud){
    pcl::PointCloud<pcl::PointXYZI>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    new_cloud->header = cloud->header;
    int width = 0;
    new_cloud->height = cloud->height;
    for(const pcl::PointXYZI& pt : cloud->points)
    {
        if(pt.z < 0.2 && pt.x > -0.5 && abs(pt.y) < 3) {
            new_cloud->points.push_back(pt);
            width++;
        }
    }
    new_cloud->width = width;
    cloud = new_cloud;
}

vector<float> color_estimation()
{
    vector<float> color(5);




    return color;
}


void callback(const sensor_msgs::PointCloud2::ConstPtr& sensor_msg)
{
    markers.markers.clear();
    marker_id = 0;

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*sensor_msg,pcl_pc2);
    
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud2(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud2);
    

    
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>),cloud_with_clusters (new pcl::PointCloud<pcl::PointXYZI>)
        ,cluster_with_inliers (new pcl::PointCloud<pcl::PointXYZI>),current_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

    lfs_msgs::cone_array cones;
    cones.header = sensor_msg->header;
    
    cout << endl <<"---New Callback---"<< endl;
    cout << "height " << sensor_msg->height << endl;
    cout << "width " << sensor_msg->width << endl;
    cout << "point_step " << sensor_msg->point_step << endl;
    cout << "row_step " << sensor_msg->row_step << endl;
    cout << "data->size " << sensor_msg->data.size() << endl;
    for(int i = 0; i<sensor_msg->fields.size(); i++) cout << "Name: " << sensor_msg->fields[i].name << " datatype: " << sensor_msg->fields[i].datatype << " offset: " << sensor_msg->fields[i].offset << endl;
    

    //*** REMOVE UNNESECCARY POINTS ***//
    clean_cloud(cloud);

    cout << "Cloud size: " << cloud->size() << endl;

    //*** GROUND REMOVAL ***//
    auto start = std::chrono::high_resolution_clock::now(); 
  
    ground_removal(cloud);

    auto stop = std::chrono::high_resolution_clock::now(); 
  
    auto duration = std::chrono::duration_cast<chrono::microseconds>(stop - start);
    cout << "ground_removal_time " << duration.count() << " microseconds" << endl;
    //*** REATTACH GROUND POINTS ***//
    //reattach_ground(msg,cloud,0.03);

    start = std::chrono::high_resolution_clock::now(); 
    //Euclidian Cluster Extraction
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>());
    tree->setInputCloud(cloud);

    stop = std::chrono::high_resolution_clock::now(); 
  
    duration = std::chrono::duration_cast<chrono::microseconds>(stop - start);

    cout << "kd_tree_making? " << duration.count() << " microseconds" << endl;
    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;

    ec.setClusterTolerance(CLUSTER_DISTANCE_THRESHHOLD);
    ec.setMinClusterSize(CLUSTER_SIZE_LOWER_LIMIT);
    ec.setMaxClusterSize(CLUSTER_SIZE_UPPER_LIMIT);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    start = std::chrono::high_resolution_clock::now(); 
    ec.extract(cluster_indices);

    stop = std::chrono::high_resolution_clock::now(); 
    duration = std::chrono::duration_cast<chrono::microseconds>(stop - start);
    cout << "euclidean clustering " << duration.count() << " microseconds" << endl;

    float n_clusters = (float) cluster_indices.size();
    cout << "Clusters found " << n_clusters << endl;

    int cluster_counter = 0;
    pcl::PointXYZI *p;
    for (const pcl::PointIndices cluster : cluster_indices)
    {
        current_cluster->clear();
        //Calculate midpoint of cluster
        float x = 0,y = 0;
        float cluster_size = (float) cluster.indices.size();

        float min_z = __FLT_MAX__, max_z = -__FLT_MAX__;
        vector<float> lines;
        vector<float> avg_line_intensity;
        vector<int> n_avg;

        for(int i : cluster.indices)
        {
            p = &((*cloud)[i]);

            x += (p->x)/cluster_size;
            y += (p->y)/cluster_size;

            min_z = min(min_z, p->z);
            max_z = max(max_z, p->z);

            current_cluster->push_back( (*cloud)[i] );

            //Keeping track of the number of lines, could be done easier?
            bool add = true;
            for(size_t j = 0; j < lines.size(); j++)
            {
                if(abs(lines[j] - p->z) < 0.03)
                {
                    add = false;
                    lines[j] = (lines[j]*n_avg[j] + p->z)/(n_avg[j] + 1);
                    avg_line_intensity[j] = (avg_line_intensity[j]*n_avg[j] + p->intensity)/(n_avg[j] + 1);
                    n_avg[j] += 1;
                }
            }
            if(add)
            {
                lines.push_back(p->z);
                avg_line_intensity.push_back(p->intensity);
                n_avg.push_back(1);
            }
        }

        string text = "size: " + to_string(cluster.indices.size());
        text += "\ntop:" + to_string(max_z);
        markers.markers.push_back(text_marker(x,y, max_z + 0.05,text, 0.05, "cluster_info"));

        float cluster_height = max_z - min_z;

        // Add 1/4th of cone diameter to find true midpoint of cone
        Eigen::Vector2d mu(x,y);
        float norm = mu.norm();
        float one_fourth_diameter = 0.15/4.0;
        mu += one_fourth_diameter*mu/norm;

        for(pcl::PointXYZI point : current_cluster->points) cloud_with_clusters->push_back(point);

        // Finding out bounding box of cone
        Eigen::Vector2d perp(-y/norm, x/norm);
        Eigen::Vector2d temp; 
        float leftmost = -__FLT_MAX__, rightmost = __FLT_MAX__;
        for(const pcl::PointXYZI &p : current_cluster->points)
        {
           temp << p.x, p.y;
           float scalar = perp.dot(temp);
           leftmost = max(leftmost, scalar);
           rightmost = min(rightmost, scalar);
        }
        Eigen::Vector2d left = mu + leftmost*perp; 
        Eigen::Vector2d right = mu + rightmost*perp; 

        float cluster_width = leftmost - rightmost; 

        //*** CLASSIFICATION ***//
        Eigen::Vector3f apex(mu.x(),mu.y(),max_z + 0.08);
        start = std::chrono::high_resolution_clock::now(); 
        float probability = classify_cluster(current_cluster, apex, markers);
        stop = std::chrono::high_resolution_clock::now(); 
        duration = std::chrono::duration_cast<chrono::microseconds>(stop - start);
        cout << "classify clusters " << duration.count() << " microseconds" << endl;

        for(pcl::PointXYZI point : current_cluster->points) cluster_with_inliers->push_back(point);

        if(cluster_width > cluster_height || probability < 0.26) continue;


        vector<float> cone_class = color_estimation();
        string cone_class_debug;
        for(float f : cone_class)
        {
            //cone_class_debug += format("{:.2f} ", f);
        }
        //markers.markers.push_back(text_marker(mu.x(), mu.y(), max_z + 0.3, cone_class_debug));

        float mid_z = (max_z + min_z)/2.0;
        //min_z = mid_z - 0.265/2.0;
        //max_z = mid_z + 0.265/2.0;

        vector<geometry_msgs::Point32> keypoints(3);


        keypoints[0].x = left.x();
        keypoints[0].y = left.y();
        keypoints[0].z = min_z; 

        keypoints[1].x = right.x();
        keypoints[1].y = right.y();
        keypoints[1].z = min_z; 

        keypoints[2].x = mu.x();
        keypoints[2].y = mu.y();
        keypoints[2].z = max_z; 

        markers.markers.push_back(marker(keypoints[0].x, keypoints[0].y, keypoints[0].z,"max"));
        markers.markers.push_back(marker(keypoints[1].x, keypoints[1].y, keypoints[0].z,"max"));
        markers.markers.push_back(marker(keypoints[2].x, keypoints[2].y, keypoints[2].z));

        lfs_msgs::cone cone;
        cone.point.x = mu.x();
        cone.point.y = mu.y();
        cone.point.z = (max_z + min_z)/2.0;
        cone.keypoints = keypoints;
        cone.cone_confidence = probability;

        if(cluster_height > 0.35)
        {
            cone.cone_class[3] = 1.0; 
        }

        cones.cones.push_back(cone);
        cluster_counter++;
    }

    delete_markers();
    vis_pub.publish(markers);
    cone_publisher.publish(cones);
    cones_in_cloud_publisher.publish(cloud_with_clusters);
    cluster_with_inliers_publisher.publish(cluster_with_inliers);
}

void parameter_callback(const lfs_msgs::filter_parameters::ConstPtr& params){
    //RANSAC
    RANSAC = params->RANSAC;
    REATTACH = params->REATTACH;
    PLANE_RANSAC_TRESHHOLD = params->PLANE_RANSAC_TRESHHOLD;
    PLANE_RANSAC_EXTRACT_TRESHHOLD = params->PLANE_RANSAC_EXTRACT_TRESHHOLD;
    PLANE_RANSAC_ANGLE_TRESHHOLD = params->PLANE_RANSAC_ANGLE_TRESHHOLD;
    PLANE_RANSAC_PROBABILITY = params->PLANE_RANSAC_PROBABILITY;
    PLANE_RANSAC_MAX_ITERATIONS = params->PLANE_RANSAC_MAX_ITERATIONS;

    //CLUSTERING
    CLUSTER_SIZE_UPPER_LIMIT = params->CLUSTER_SIZE_UPPER_LIMIT;
    CLUSTER_SIZE_LOWER_LIMIT = params->CLUSTER_SIZE_LOWER_LIMIT;
    CLUSTER_DISTANCE_THRESHHOLD = params->CLUSTER_DISTANCE_THRESHHOLD;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "sub_pcl");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, callback);
    ros::Subscriber sub_spher = nh.subscribe<sensor_msgs::PointCloud2>("/spherical/pointcloud_raw", 1, callback);
    ros::Subscriber parameters_sub = nh.subscribe<lfs_msgs::filter_parameters>("/filter_parameters", 1, parameter_callback);
    ros::Subscriber sub2 = nh.subscribe<sensor_msgs::PointCloud2>("/filtered_points", 1, callback);
    cloud_after_ransac_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI>> ("/lidar3d/pointclouds/points_after_RANSAC", 1);
    cloud_with_ground_removed_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI>> ("/lidar3d/pointclouds/points_with_ground_removed", 1);
    cones_in_cloud_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI>> ("/lidar3d/pointclouds/cones_in_cloud", 1);
    cluster_with_inliers_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI>> ("/lidar3d/pointclouds/cluster_with_inliers", 1);
    vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/lidar3d/debug", 1);
    cone_publisher = nh.advertise<lfs_msgs::cone_array>("/lidar3d/cones", 1);
    ros::spin();
}

