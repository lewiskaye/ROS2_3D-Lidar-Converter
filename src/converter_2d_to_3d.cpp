//TODO Description

#include "../include/converter_2d_to_3d.hpp"
using std::placeholders::_1;

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("Starting 2D -> 3D Converter Node\n");

  //Impliment Lidar Subscriber
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarSubscriber>());

  // Point Cloud Publisher is implimented within the subscriber (no extra call needed)

  //Exit Node
  rclcpp::shutdown();
  return 0;
}


