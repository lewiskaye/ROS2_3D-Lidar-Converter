//TODO Description

// Include from standard C libraries
#include <chrono>
#include <cstdio>
#include <memory>
#include <sstream>
#include <string>

// Include from ROS 2 Packages
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
//#include "turtlesim/msg/pose.hpp"

#include <sensor_msgs/msg/imu.hpp>
//#include <sensor_msgs/msg/laser_scan.hpp>
//#include <laser_geometry/laser_geometry.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class IMUFramePublisher : public rclcpp::Node
{
public:
  IMUFramePublisher();
  
  
private:
  // Subscriber to handle messges containing the IMU's raw/processed data
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string child_frame_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr test_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Method to do the processing & broadcasting when called
  void handle_imu(const std::shared_ptr<sensor_msgs::msg::Imu> msg); //shared ptr
  void test_data();
};