// This file is the entry point for the Full 3D Scan-Capture Service
// Currently - This file handles all the logic for a scan capture, however it may be transfered into a Service or an Action Client in the future.  

// ON LOAD - Levels the platform to Horizontal using IMU and Stepper Motor by calling LEVEL in the Stepper Node (OLD- small angles)
// CONTINOUSLY - Showing Output in RViz
// PSEUDOCODE:
// - Rotating the Stepper Motor upwards to vertically straight up (according to IMU Readings OR just 90 deg if levelled out already)
// - Starting the Scan
// - Rotating gradually downwards (take scan method called in stepper?)
// - Stopping after set number of steps (180 deg roughly)
// - Stopping Scan
// - Exporting
// - Return to Horizontal position

#include "../include/capture_scan.hpp"
using std::placeholders::_1;

//using namespace std::chrono_literals;
//using namespace lidar_converter;


int main(int argc, char **argv)
{
	(void) argc;
	(void) argv;

	rclcpp::init(argc, argv);
	printf("Starting Capture Node\n");

	//Spin Stepper Motor Node
	auto stepper_client = std::make_shared<StepperMotorClient>();
	rclcpp::spin(stepper_client);

	std::cout << "test" << std::endl;
	//stepper_client.send_goal();

	// Point Cloud Publisher is implimented within the subscriber (no extra call needed)

	// Spin the Capture Scan??



	//Exit Node
	rclcpp::shutdown();
	return 0;
}