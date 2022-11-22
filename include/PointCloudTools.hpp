// This class contains some custom made, useful utilities not found within ROS for dealing with Point Clouds

// C++ Imports
#include <string>

// ROS2 Imports
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>

class PointCloudTools
{
public:
    static void PrintPointCloud(pcl::PointCloud<pcl::PointXYZI> cloud);
    static void PrintPointCloudInfo(sensor_msgs::msg::PointCloud2 cloud);
    static pcl::PointCloud<pcl::PointXYZI> laserscan_to_pcl(sensor_msgs::msg::LaserScan scan);
    static sensor_msgs::msg::PointCloud2 laserscan_to_pc2(sensor_msgs::msg::LaserScan scan);
    static sensor_msgs::msg::LaserScan generate_random_laserscan(unsigned int num_readings = 360, double laser_frequency = 50);

private:

};