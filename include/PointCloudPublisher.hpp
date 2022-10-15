//TODO Description

// Include from standard C libraries
#include <cstdio>
#include <memory>
#include <string>

// Include from ROS 2 Packages
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Include from other libraries (e.g. RPLiDAR)
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>

using std::placeholders::_1;

class PointCloudPublisher : public rclcpp::Node
{
public:
  PointCloudPublisher();
  //virtual ~PointCloudPublisher();
  void publish_pointcloud(sensor_msgs::msg::PointCloud2 & pc); //Do we need the &ref symbol??

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
};