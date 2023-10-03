#include "message_parsing.h"

using namespace std;
using namespace Eigen;

extern pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud;
extern pcl::PointCloud<pcl::PointXYZI>::Ptr possible_ground_points;
// XYZIR format used in excentric calibration
extern pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr raw_cloud_xyzir;

std::vector<std::vector<float>> timing_offsets;

bool build_timings()
{
    std::vector<std::vector<float>> timing_offsets;
    // vlp16
    // timing table calculation, from velodyne user manual
    timing_offsets.resize(12);
    for (size_t i = 0; i < timing_offsets.size(); ++i) {
        timing_offsets[i].resize(32);
    }
    // constants
    double full_firing_cycle = 55.296 * 1e-6; // seconds
    double single_firing = 2.304 * 1e-6; // seconds
    double dataBlockIndex, dataPointIndex;
    bool dual_mode = false;
    // compute timing offsets
    for (size_t x = 0; x < timing_offsets.size(); ++x) {
        for (size_t y = 0; y < timing_offsets[x].size(); ++y) {
            if (dual_mode) {
                dataBlockIndex = (x - (x % 2)) + (y / 16);
            } else {
                dataBlockIndex = (x * 2) + (y / 16);
            }
            dataPointIndex = y % 16;
            //timing_offsets[block][firing]
            timing_offsets[x][y] = (full_firing_cycle * dataBlockIndex) + (single_firing * dataPointIndex);
        }
    }

    if (timing_offsets.size()) {
        /*
        // ROS_INFO("VELODYNE TIMING TABLE:");
        for (size_t x = 0; x < timing_offsets.size(); ++x)
        {
            for (size_t y = 0; y < timing_offsets[x].size(); ++y)
            {
                printf("%04.3f ", timing_offsets[x][y] * 1e6);
            }
            printf("\n");
        }*/
        return true;
    } else {
        ROS_WARN("NO TIMING OFFSETS CALCULATED. ARE YOU USING A SUPPORTED VELODYNE SENSOR?");
    }
    return false;
}

void read_rawdata(const velodyne_msgs::VelodyneScan::ConstPtr& msg,
    std::vector<std::vector<float>>& r_matrix,
    std::vector<std::vector<float>>& theta_matrix,
    std::vector<std::list<int>>& r_matrix_nonzero_indices,
    std::vector<std::vector<Eigen::Vector3f>>& cart_matrix,
    std::vector<std::vector<float>>& intensity_matrix,
    std::vector<std::vector<float>>& p_trig)
{
    //clear indices;
    for (int i = 0; i < 16; i++)
        r_matrix_nonzero_indices[i].clear();

    size_t n_lasers = r_matrix_nonzero_indices.size();

    vector<list<int>> start(n_lasers), end(n_lasers);
    raw_cloud->points.clear();
    pcl_conversions::toPCL(msg->header, raw_cloud->header);

    raw_cloud_xyzir->points.clear();
    pcl_conversions::toPCL(msg->header, raw_cloud_xyzir->header);

    int azimuth_index = 0;
    for (const velodyne_msgs::VelodynePacket& pkt : msg->packets) {
        float azimuth;
        float azimuth_diff;
        int raw_azimuth_diff;
        float last_azimuth_diff = 0;
        float azimuth_corrected_f;
        int azimuth_corrected;
        float intensity;

        float time_diff_start_to_this_packet = (pkt.stamp - msg->header.stamp).toSec();

        const raw_packet_t* raw = (const raw_packet_t*)&pkt.data[0];

        for (int block = 0; block < BLOCKS_PER_PACKET; block++) {

            // ignore packets with mangled or otherwise different contents
            if (UPPER_BANK != raw->blocks[block].header) {
                // Do not flood the log with messages, only issue at most one
                // of these warnings per minute.
                ROS_WARN_STREAM_THROTTLE(60, "skipping invalid VLP-16 packet: block " << block << " header value is " << raw->blocks[block].header);
                return; // bad packet: skip the rest
            }

            // Calculate difference between current and next block's azimuth angle.
            azimuth = (float)(raw->blocks[block].rotation);
            if (block < (BLOCKS_PER_PACKET - 1)) {
                raw_azimuth_diff = raw->blocks[block + 1].rotation - raw->blocks[block].rotation;
                azimuth_diff = (float)((36000 + raw_azimuth_diff) % 36000);
                // some packets contain an angle overflow where azimuth_diff < 0
                if (raw_azimuth_diff < 0) //raw->blocks[block+1].rotation - raw->blocks[block].rotation < 0)
                {
                    ROS_WARN_STREAM_THROTTLE(60, "Packet containing angle overflow, first angle: " << raw->blocks[block].rotation << " second angle: " << raw->blocks[block + 1].rotation);
                    // if last_azimuth_diff was not zero, we can assume that the velodyne's speed did not change very much and use the same difference
                    if (last_azimuth_diff > 0) {
                        azimuth_diff = last_azimuth_diff;
                    }
                    // otherwise we are not able to use this data
                    // TODO: we might just not use the second 16 firings
                    else {
                        continue;
                    }
                }
                last_azimuth_diff = azimuth_diff;
            } else {
                azimuth_diff = last_azimuth_diff;
            }

            for (int firing = 0, k = 0; firing < VLP16_FIRINGS_PER_BLOCK; firing++) {

                for (int dsr = 0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k += RAW_SCAN_SIZE) {

                    /** Position Calculation */
                    union two_bytes tmp;
                    tmp.bytes[0] = raw->blocks[block].data[k];
                    tmp.bytes[1] = raw->blocks[block].data[k + 1];

                    /** correct for the laser rotation as a function of timing during the firings **/
                    azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr * VLP16_DSR_TOFFSET) + (firing * VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION);
                    azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;
                    float theta = angles::from_degrees(ROTATION_RESOLUTION * azimuth_corrected);

                    /*condition added to avoid calculating points which are not
                      in the interesting defined area (min_angle < area < max_angle)*/

                    // convert polar coordinates to Euclidean XYZ
                    float r = tmp.uint * 0.002;
                    /** Intensity Calculation */
                    float min_intensity = 0;
                    float max_intensity = 1024;

                    intensity = raw->blocks[block].data[k + 2];

                    intensity = (intensity < min_intensity) ? min_intensity : intensity;
                    intensity = (intensity > max_intensity) ? max_intensity : intensity;

                    float time = 0;
                    if (timing_offsets.size())
                        time = timing_offsets[block][firing * 16 + dsr] + time_diff_start_to_this_packet;

                    int laser_index = dsr / 2;
                    if (dsr % 2 != 0) {
                        laser_index += 8;
                    }

                    r_matrix[laser_index][azimuth_index] = r;

                    theta = (2 * M_PI - theta);
                    if (theta > M_PI)
                        theta = theta - 2 * M_PI;
                    theta_matrix[laser_index][azimuth_index] = theta;

                    if (laser_index == 0) {
                        p_trig[0][azimuth_index] = sin(theta);
                        p_trig[1][azimuth_index] = cos(theta);
                        p_trig[2][azimuth_index] = tan(theta);
                    }

                    pcl::PointXYZI p;
                    velodyne_pointcloud::PointXYZIR p_xyzir;

                    float phi = phis[laser_index];
                    Vector3f polar(r, theta, phi);
                    Vector3f cart = polar_to_cartesian(polar);

                    p.x = cart.x();
                    p.y = cart.y();
                    p.z = cart.z();
                    p.intensity = intensity;

                    p_xyzir.x = p.x;
                    p_xyzir.y = p.y;
                    p_xyzir.z = p.z;
                    p_xyzir.intensity = intensity;
                    p_xyzir.ring = laser_index;

                    if (r > 0) {

                        if (abs(theta) < FOV_LIDAR) {
                            if ((start[laser_index].size() == 0 || theta < theta_matrix[laser_index][start[laser_index].front()])) {
                                start[laser_index].push_front(azimuth_index);
                            } else {
                                end[laser_index].push_front(azimuth_index);
                            }
                        }
                        cart_matrix[laser_index][azimuth_index] = cart;
                        intensity_matrix[laser_index][azimuth_index] = intensity;
                        raw_cloud->push_back(p);
                        raw_cloud_xyzir->push_back(p_xyzir);
                        if (laser_index < 4 && abs(theta) < 2.0) {
                            possible_ground_points->push_back(p);
                        }
                    }
                }
                azimuth_index++;
            }
        }
    }
    list<int>::iterator it;
    for (size_t i = 0; i < r_matrix_nonzero_indices.size(); i++) {

        //Remove overlap, maybe use reverse iterator?
        it = start[i].end();
        it--;
        while (it != start[i].begin() && end[i].size() > 0 && theta_matrix[i][*it] > theta_matrix[i][end[i].front()]) {
            it = start[i].erase(it);
            it--;
        }

        start[i].splice(start[i].end(), end[i]);
        r_matrix_nonzero_indices[i] = start[i];
    }
}

void read_sensor_message(const sensor_msgs::PointCloud2::ConstPtr& msg,
    std::vector<std::vector<float>>& r_matrix,
    std::vector<std::vector<float>>& theta_matrix,
    std::vector<std::list<int>>& r_matrix_nonzero_indices,
    std::vector<std::vector<Eigen::Vector3f>>& cart_matrix,
    std::vector<std::vector<float>>& p_trig)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud2(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud2);

    size_t n_lasers = r_matrix_nonzero_indices.size();

    for (size_t i = 0; i < r_matrix.size(); i++) {
        for (size_t j = 0; j < r_matrix[i].size(); j++) {
            r_matrix[i][j] = 0;
        }
    }

    vector<list<int>> start(n_lasers), end(n_lasers);
    float r, theta, phi;
    int laser_index = 0;
    int azimuth_index = 0;
    pcl::PointXYZI new_p;
    for (const velodyne_pointcloud::PointXYZIR& p : cloud2->points) {
        /*
        if (p.ring < laser_index)
        {
            azimuth_index++;
        }
        */

        laser_index = p.ring;

        r = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        theta = atan2(p.y, p.x);
        azimuth_index = (theta + M_PI) * r_matrix[0].size() / (2 * M_PI);
        phi = phis[laser_index];

        r_matrix[laser_index][azimuth_index] = r;
        theta_matrix[laser_index][azimuth_index] = theta;

        Vector3f cart(p.x, p.y, p.z);
        cart_matrix[laser_index][azimuth_index] = cart;

        if (laser_index == 0) {
            p_trig[0][azimuth_index] = sin(theta);
            p_trig[1][azimuth_index] = cos(theta);
            p_trig[2][azimuth_index] = tan(theta);
        }

        if (r > 0 && abs(theta_matrix[laser_index][azimuth_index]) < FOV_LIDAR) {
            r_matrix_nonzero_indices[laser_index].push_back(azimuth_index);
            new_p.x = p.x;
            new_p.y = p.y;
            new_p.z = p.z;
            new_p.intensity = p.intensity;

            raw_cloud->push_back(new_p);
            if (laser_index < 4 && abs(theta) < 2.0) {
                possible_ground_points->push_back(new_p);
            }
        }
    }
}
