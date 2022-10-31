#include "../include/StepperMotorClient.hpp"
using std::placeholders::_1;

StepperMotorClient::StepperMotorClient() : Node("stepper_client")
{
    this->move_motor_client_ptr_ = rclcpp_action::create_client<StepperMotor>(this, "move_motor");
    this->level_motor_client_ptr_ = rclcpp_action::create_client<Level>(this, "level_motor");

    // Timer to automatically capture scan after 0.5s
    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&StepperMotorClient::capture_scan, this));

    // Create a subscription to the '/scan' topic
    pc_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>("scan3d", rclcpp::QoS(rclcpp::SensorDataQoS()) /*QoS for sensors (best effort etc)*/, std::bind(&StepperMotorClient::scan_callback, this, _1));
}


// Takes a full scan using LiDAR, IMU, Stepper Motor
// Params: none
void StepperMotorClient::capture_scan()
{
    // Check platform has been levelled
    if (levelled != true) {
        // Sleep for 1 second in the hope that the platform becomes levelled
        RCLCPP_ERROR(this->get_logger(), "Platform has not been levelled yet...");
        //sleep(1);
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
        //rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(), "Platform has been levelled");

    // Rotate the Stepper Motor upwards to vertically straight up (TODO - according to IMU Readings OR just 90 deg if levelled out already)
    float speed_fast = 10; //rps
    float speed_slow = 10; //rps //TODO const?
    RCLCPP_INFO(this->get_logger(), "[SCAN] Panning Motor Up to Vertical");
    move_motor(-90, speed_fast);
    // TODO wait?
    //rclcpp::sleep_for(std::chrono::milliseconds(2000));

    // Start the Scan
    RCLCPP_INFO(this->get_logger(), "[SCAN] Attempting to start scanning...");
    this->scanning = true;
    // Now this variable is set to true, the Scan Callback Function will process each scan...

    // Rotate gradually downwards (take scan method called in stepper?)
    RCLCPP_INFO(this->get_logger(), "[SCAN] Panning Motor Down");
    move_motor(180, speed_slow);

    // Stop Scan
    RCLCPP_INFO(this->get_logger(), "[SCAN] Stopping scan...");
    this->scanning = false;

    // Return to Horizontal position
    RCLCPP_INFO(this->get_logger(), "[SCAN] Panning Motor to Level");
    move_motor(90, speed_fast);

    // Export PCL


    
}


// Called every time the node recieves a PointCloud2 Scan Message on the topic scan3d
void StepperMotorClient::scan_callback(const sensor_msgs::msg::PointCloud2 & pc_msg)
{
    // Check if node in scanning state
    if (this->scanning == true) {
        RCLCPP_INFO(this->get_logger(), "[SCAN] Scan Successfully Starting...");
    }
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

    auto goal_msg = Level::Goal();
    goal_msg.imu_topic = "imu/imu";

    RCLCPP_INFO(this->get_logger(), "Sending 'LEVEL' command to stepper motor action client");
    auto level_goal_options = rclcpp_action::Client<Level>::SendGoalOptions();
    level_goal_options.result_callback = std::bind(&StepperMotorClient::level_callback, this, _1);
    this->level_motor_client_ptr_->async_send_goal(goal_msg, level_goal_options);
}

// Send a command to move the Servo motor
// Params: float Target Angle (deg) and float Speed (rps)
void StepperMotorClient::move_motor(float target_angle, float speed)
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
    //rclcpp::shutdown();
}

// Handle when the Action Server sends back a result
void StepperMotorClient::level_callback(const GoalHandleLevel::WrappedResult & level_result)
{
    switch (level_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Levelling Operation Completed Successfully");
        break;
        case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Levelling Operation was aborted");
        return;
        case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Levelling Operation was canceled");
        return;
        default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    // Set Self Variable
    this->levelled = true;
}

//RCLCPP_COMPONENTS_REGISTER_NODE(StepperMotorClient::StepperMotorClient)