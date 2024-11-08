#define PCL_NO_PRECOMPILE
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/register_point_struct.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tins/tins.h>
#include <deque>
#include <mutex>
#include <iostream>
#include <fstream>
#include <string>
#include <filesystem>

namespace livox_ros {
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;     
        PCL_ADD_INTENSITY;
        uint tag;  
        double timestamp;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    };    
}


POINT_CLOUD_REGISTER_POINT_STRUCT (livox_ros::Point,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (uint, tag, tag)
                                   (double, timestamp, timestamp))

// 定义PointCloud类型
typedef livox_ros::Point PointType;
typedef pcl::PointCloud<PointType> PointCloudLivox;
typedef PointCloudLivox::Ptr PointCloudLivoxPtr;

void read_mid360_packets(const std::string &pcap_filename, rosbag::Bag &rosbag)
{
    Tins::FileSniffer sniffer(pcap_filename);

    int imu_payload_size = 60, lidar_payload_size = 1380;

    uint16_t time_interval, dot_num, point_time_interval;
    uint64_t timestamp;

    PointCloudLivoxPtr cloud(new PointCloudLivox());

    // int cloud_offset = 36;
    
    while (Tins::Packet packet = sniffer.next_packet())
    {
        auto &pdu = *packet.pdu();
        const Tins::UDP *udp = pdu.find_pdu<Tins::UDP>();
        if (!udp) continue;
        const Tins::RawPDU *raw = pdu.find_pdu<Tins::RawPDU>();
        if (!raw) continue;

        const Tins::RawPDU::payload_type &payload = raw->payload();
        if (payload.size() != imu_payload_size && payload.size() != lidar_payload_size)
            continue;

        memcpy(&time_interval, payload.data() + 3, 2);
        memcpy(&dot_num, payload.data() + 5, 2);
        memcpy(&timestamp, payload.data() + 28, 8);

        int cloud_offset = 36;

        // Lidar数据解析
        if (payload.size() == lidar_payload_size)
        {
            
            int point_nums = 0;
            point_time_interval = time_interval / (dot_num - 1);
            PointType point;

            while (cloud_offset < lidar_payload_size)
            {
                uint8_t reflectivity, tag;
                point_nums++;

                if (payload[10] == 1)
                {
                    int32_t x, y, z;
                    memcpy(&x, payload.data() + cloud_offset + 0, 4);
                    memcpy(&y, payload.data() + cloud_offset + 4, 4);
                    memcpy(&z, payload.data() + cloud_offset + 8, 4);
                    memcpy(&reflectivity, payload.data() + cloud_offset + 12, 1);
                    memcpy(&tag, payload.data() + cloud_offset + 13, 1);
                    cloud_offset += 14;
                    point.x = static_cast<float>(x) * 1e-3;
                    point.y = static_cast<float>(y) * 1e-3;
                    point.z = static_cast<float>(z) * 1e-3;
                }
                else if (payload[10] == 2)
                {
                    int16_t x, y, z;
                    memcpy(&x, payload.data() + cloud_offset + 0, 2);
                    memcpy(&y, payload.data() + cloud_offset + 2, 2);
                    memcpy(&z, payload.data() + cloud_offset + 4, 2);
                    memcpy(&reflectivity, payload.data() + cloud_offset + 6, 1);
                    memcpy(&tag, payload.data() + cloud_offset + 7, 1);
                    cloud_offset += 8;
                    point.x = static_cast<float>(x) * 1e-4;
                    point.y = static_cast<float>(y) * 1e-4;
                    point.z = static_cast<float>(z) * 1e-4;
                }
                else
                {
                    ROS_WARN("Unsupported data type");
                    return;
                }
                point.tag = tag;
                point.intensity = reflectivity;
                point.timestamp = timestamp + (point_nums - 1) * point_time_interval * 1e2;
                // ROS_INFO("point.timestamp: %f", point.timestamp);
                // point.curvature = static_cast<double>(timestamp) * 1e-9;  // 使用时间戳
                cloud->push_back(point);
            }

            if (!cloud->points.empty() && (cloud->points.back().timestamp - cloud->points[0].timestamp)*1e-9 >= 0.1)
            {
                sensor_msgs::PointCloud2 ros_cloud;
                pcl::toROSMsg(*cloud, ros_cloud);
                ros_cloud.header.stamp.fromNSec(cloud->points.front().timestamp);
                ros_cloud.header.frame_id = "lidar_frame";
                rosbag.write("/livox/lidar", ros_cloud.header.stamp, ros_cloud);
                cloud = PointCloudLivoxPtr(new PointCloudLivox());
            }
        }
        // IMU数据解析
        else if (payload.size() == imu_payload_size)
        {
            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp.fromNSec(timestamp);
            imu_msg.header.frame_id = "imu_frame";

            float gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z;
            memcpy(&gyro_x, payload.data() + cloud_offset + 0, 4);
            memcpy(&gyro_y, payload.data() + cloud_offset + 4, 4);
            memcpy(&gyro_z, payload.data() + cloud_offset + 8, 4);
            memcpy(&acc_x, payload.data() + cloud_offset + 12, 4);
            memcpy(&acc_y, payload.data() + cloud_offset + 16, 4);
            memcpy(&acc_z, payload.data() + cloud_offset + 20, 4);

            imu_msg.angular_velocity.x = gyro_x;
            imu_msg.angular_velocity.y = gyro_y;
            imu_msg.angular_velocity.z = gyro_z;
            imu_msg.linear_acceleration.x = acc_x;
            imu_msg.linear_acceleration.y = acc_y;
            imu_msg.linear_acceleration.z = acc_z;

            rosbag.write("/livox/imu", imu_msg.header.stamp, imu_msg);
        }
    }
}

int main(int argc, char **argv)
{
    std::string pcap_filepath, pcap_filename;
    ros::init(argc, argv, "pcap_to_rosbag");
    ros::NodeHandle nh;
    ros::param::param<std::string>("pcap_filename", pcap_filename, "");
    ros::param::param<std::string>("pcap_filepath", pcap_filepath, "");

    std::filesystem::path file_path(pcap_filename);
    rosbag::Bag bag;
    bag.open("/home/ryan/tools/pcap2ros_ws/src/pcap2ros/out/" + file_path.stem().string() + ".bag", rosbag::BagMode::Write);

    read_mid360_packets(pcap_filepath + pcap_filename, bag);

    bag.close();
    ROS_INFO("PCAP file has been successfully converted to rosbag.");
    return 0;
}
