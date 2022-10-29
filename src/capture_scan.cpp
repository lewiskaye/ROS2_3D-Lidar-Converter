// This file/node is responsible for capturing a full 3D Scan by
// ON LOAD - Leveling the platdown to Horizontal using IMU and Stepper Motor small angles
// CONTINOUSLY - Showing Output in RViz
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

using namespace std::chrono_literals;


int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	// Initialise angle (deg) and steps (count)
	int angle = 0;
	int steps = 0;


	// Create Node
	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("capture_scan");

	// Create Stepper Motor Client within Node
	rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client = node->create_client<example_interfaces::srv::AddTwoInts>("move_motor");

	// Create a Request to move the Stepper Motor - as defined by the .srv interface used
	// TODO - change to customn .srv message
	auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
	request->a = angle;
	request->b = steps;

	// Wait repeatidly for service nodes on the network (1s intervals)
	while (!client->wait_for_service(1s)) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
			return 0;
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
	}

	// Send Servo Motor Rotation request to the service and wait for the result
	auto result = client->async_send_request(request);

	// OLD method for handling errors (as per ROS2 tutorials)
	if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
	{
		if (result.get()->sum == 0) {
			// Success!
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stepper Motor successfully moved");

		} else {
			// an error occured whilst moving the motor (or with the angle(s) passed)			
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stepper Motor was NOT successfully moved");

		}
	} else {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service move_motor to rotate the Stepper Motor");
	}

	rclcpp::shutdown();
	return 0;
}