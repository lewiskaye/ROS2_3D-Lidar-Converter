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

	//Handle cmd args
	if (argc == 2 && std::atoi(argv[1]) != 0) {
		angle = std::atoi(argv[1]);
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Angle entered in Degrees: %ld deg", angle);
		
	} else if(argc == 3 && std::atoi(argv[2]) != 0) {
		steps = std::atoi(argv[2]);
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Step count entered: %ld steps", steps);

	} else {
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: ros2 run lidar_converter stepper_client <target angle in deg> (<no of steps to rotate>)");
		return 1;
	}

	// Create Node
	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("stepper_client");

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

	if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
	{
		if (result.get()->sum == 0) {
			// an error occured whilst moving the motor (or with the angle(s) passed)
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Error - the Stepper Motor was not successfully moved");

		} else {
			// Success!
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stepper Motor successfully moved: %ld degrees", result.get()->sum);

		}
	} else {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service move_motor to rotate the Stepper Motor");
	}

	rclcpp::shutdown();
	return 0;
}