// This is the entry point for the main LiDAR Converter Node
// Which converts from 2D Scans to 3D Point Clouds

#include "../include/converter_2d_to_3d.hpp"
using std::placeholders::_1;

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  
  rclcpp::init(argc, argv);
  printf("Starting 2D -> 3D Converter Node\n");

  //Spin Lidar Subscriber Node (does the )
  rclcpp::spin(std::make_shared<LidarSubscriber>());

  // Point Cloud Publisher is implimented within the subscriber (no extra call needed)

  //Exit Node
  rclcpp::shutdown();
  return 0;
}


