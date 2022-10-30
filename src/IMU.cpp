// This File (executable) is an entry point for the IMU Frame Publisher - responsible for taking the Quaternion Data from the IMU topic and publishing the relevent TF

#include "../include/imu.hpp"
using std::placeholders::_1;

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("IMU Frame Publisher Node\n");

  // Spin the IMU Frame Subscriber Node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUFramePublisher>());

  //Exit Node
  rclcpp::shutdown();
  return 0;
}


