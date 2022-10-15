//TODO Description

// Include from standard C libraries
#include <cstdio>
#include <memory>
#include <string>

//Include from this package
#include "PointCloudPublisher.hpp"

// Include from ROS 2 Packages
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Include from other libraries (e.g. RPLiDAR)
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>

using std::placeholders::_1;

class LidarSubscriber : public rclcpp::Node
{
public:
  LidarSubscriber();
  virtual ~LidarSubscriber();

private:
  void scan_callback(const sensor_msgs::msg::LaserScan & scan_msg) const;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription;
  PointCloudPublisher* pc_publisher;
};