// This file/node is responsible for capturing a full 3D Scan by
// ON LOAD - Leveling the platdown to Horizontal using IMU and Stepper Motor small angles
// CONTINOUSLY - Showing Output in RViz
// PSEUDOCODE:
// - Rotating the Stepper Motor upwards to vertically straight up (according to IMU Readings OR just 90 deg if levelled out already)
// - Starting the Scan
// - Rotating gradually downwards (take scan method called in stepper?)
// - Stopping after set number of steps (180 deg roughly)
// - Stopping Scan
// - Exporting
// - Return to Horizontal position

// C++ Imports
#include <chrono>
#include <cstdlib>
#include <memory>

// ROS2 Imports
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

// Custom Imports
#include "../include/StepperMotorClient.hpp"

using namespace std::chrono_literals;


int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	//auto stepper_motor_client = StepperMotorClient();

	rclcpp::shutdown();
	return 0;
}