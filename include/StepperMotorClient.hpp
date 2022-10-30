// C++ Imports
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

// ROS2 Imports
#include "custom_interfaces/action/stepper_motor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// Custom Imports

using StepperMotor = custom_interfaces::action::StepperMotor;
using GoalHandleStepperMotor = rclcpp_action::ClientGoalHandle<StepperMotor>;

class StepperMotorClient : public rclcpp::Node
{
    public:
        StepperMotorClient();
        void level_motor(std::string imu_topic = "imu/imu");
        //void capture_scan();
        void send_goal();//float target_angle, float speed = 10);

    private:
        rclcpp_action::Client<StepperMotor>::SharedPtr move_motor_client_ptr_;
        rclcpp_action::Client<StepperMotor>::SharedPtr level_motor_client_ptr_;
        rclcpp::TimerBase::SharedPtr timer_;
        void goal_response_callback(const GoalHandleStepperMotor::SharedPtr & goal_handle);
        void feedback_callback(GoalHandleStepperMotor::SharedPtr, const std::shared_ptr<const StepperMotor::Feedback> feedback);
        void result_callback(const GoalHandleStepperMotor::WrappedResult & result);
        
};
