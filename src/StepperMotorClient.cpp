#include "../include/StepperMotorClient.hpp"

StepperMotorClient::StepperMotorClient() : Node("stepper_client")
{
    this->move_motor_client_ptr_ = rclcpp_action::create_client<StepperMotor>(this, "move_motor");
    this->level_motor_client_ptr_ = rclcpp_action::create_client<StepperMotor>(this, "level_motor");

    // Timer to automatically send message (for usage as a standalone node)
    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&StepperMotorClient::send_goal, this));
}

// Send a command to level the Servo motor using the IMU
// Params: none OR IMU Topic
void StepperMotorClient::level_motor(std::string imu_topic)
{
    using namespace std::placeholders;

    // Wait for Action Server
    if (!this->level_motor_client_ptr_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
    }

}

// Send a command to move the Servo motor
// Params: float Target Angle (deg) and float Speed (rps)
void StepperMotorClient::send_goal()//float target_angle, float speed)
{
    using namespace std::placeholders;

    // Wait for Action Server
    if (!this->move_motor_client_ptr_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
    }

    // Set Target Angle (deg) and Desired Speed (rps)
    auto goal_msg = StepperMotor::Goal();
    goal_msg.target_angle = 10;//target_angle;
    goal_msg.speed = 10;//speed;

    // Send Message (goal) to Stepper Motor Action Server
    RCLCPP_INFO(this->get_logger(), "Sending goal to stepper motor");
    auto send_goal_options = rclcpp_action::Client<StepperMotor>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&StepperMotorClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&StepperMotorClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&StepperMotorClient::result_callback, this, _1);
    this->move_motor_client_ptr_->async_send_goal(goal_msg, send_goal_options);
}


void StepperMotorClient::goal_response_callback(const GoalHandleStepperMotor::SharedPtr & goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

// Handle Feedback from Action Server
void StepperMotorClient::feedback_callback(GoalHandleStepperMotor::SharedPtr, const std::shared_ptr<const StepperMotor::Feedback> feedback)
{
    std::stringstream ss;
    ss << "Motor at current angle: " << feedback->current_angle;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
}

// Handle when the Action Server sends back a result
void StepperMotorClient::result_callback(const GoalHandleStepperMotor::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Stepper Motor Rotation Completed Successfully");
        break;
        case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Stepper Motor Rotation was aborted");
        return;
        case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Stepper Motor Rotation was canceled");
        return;
        default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    // Display Resulting message code if desired
    // std::stringstream ss;
    // ss << "Result received: " << result.result->succeeded;
    // RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
}


//RCLCPP_COMPONENTS_REGISTER_NODE(StepperMotorClient::StepperMotorClient)