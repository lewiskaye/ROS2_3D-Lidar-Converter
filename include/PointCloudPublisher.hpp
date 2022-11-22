// Include from standard C libraries
#include <cstdio>
#include <memory>
#include <string>

// Include from ROS 2 Packages
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include "pcl_conversions/pcl_conversions.h"
//#include "sensor_msgs/point_cloud2_iterator.hpp"

using namespace std::placeholders;
using std::placeholders::_1;

class PointCloudPublisher : public rclcpp::Node
{
public:
  PointCloudPublisher();
  //virtual ~PointCloudPublisher();
  void publish_pointcloud(sensor_msgs::msg::PointCloud2 &pc);
  void PrintPointCloud(pcl::PointCloud<pcl::PointXYZI> cloud);

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
};