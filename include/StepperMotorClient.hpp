// C++ Imports
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

// ROS2 Imports
#include "custom_interfaces/action/stepper_motor.hpp"
#include "custom_interfaces/action/level.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rclcpp_components/register_node_macro.hpp"


// Custom Imports

using StepperMotor = custom_interfaces::action::StepperMotor;
using Level = custom_interfaces::action::Level;
using GoalHandleStepperMotor = rclcpp_action::ClientGoalHandle<StepperMotor>;
using GoalHandleLevel = rclcpp_action::ClientGoalHandle<Level>;

class StepperMotorClient : public rclcpp::Node
{
    public:
        StepperMotorClient();
        void level_motor(std::string imu_topic = "imu/imu");
        void capture_scan();
        void move_motor(float target_angle, float speed = 10);

    private:
        rclcpp_action::Client<StepperMotor>::SharedPtr move_motor_client_ptr_;
        rclcpp_action::Client<Level>::SharedPtr level_motor_client_ptr_;
        rclcpp_action::Client<Level>::SharedPtr level_client;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscription;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription;
        bool levelled = false;
        bool scanning = false;
        float latest_pitch = 999.9;
        void goal_response_callback(const GoalHandleStepperMotor::SharedPtr & goal_handle);
        void feedback_callback(GoalHandleStepperMotor::SharedPtr, const std::shared_ptr<const StepperMotor::Feedback> feedback);
        void result_callback(const GoalHandleStepperMotor::WrappedResult & result);
        void level_callback(const GoalHandleLevel::WrappedResult & level_result);
        void scan_callback(const sensor_msgs::msg::PointCloud2 & pc_msg);
        void imu_callback(const sensor_msgs::msg::Imu & imu_msg);
};
