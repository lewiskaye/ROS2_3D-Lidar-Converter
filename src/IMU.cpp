//TODO Description

#include "../include/imu.hpp"
using std::placeholders::_1;

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("IMU Frame Publisher Node\n");

  //Impliment Lidar Subscriber
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUFramePublisher>());

  //Exit Node
  rclcpp::shutdown();
  return 0;
}


