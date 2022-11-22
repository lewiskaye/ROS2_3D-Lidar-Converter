// This file is responsible for taking a scan
// and communicating with the stepper motor

#include "../include/StepperMotorClient.hpp"
using std::placeholders::_1;

// Constructor
StepperMotorClient::StepperMotorClient() : Node("stepper_client")
{
    // Create Clients and Subscriptions
    this->move_motor_client_ptr_ = rclcpp_action::create_client<StepperMotor>(this, "move_motor");
    this->level_motor_client_ptr_ = rclcpp_action::create_client<Level>(this, "level_motor");
    this->level_client = rclcpp_action::create_client<Level>(this, "level_motor");

    // Subscribe to the '/scan3d' topic to capture scan data
    pc_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>("scan3d", rclcpp::QoS(rclcpp::SensorDataQoS()) /*QoS for sensors (best effort etc)*/, std::bind(&StepperMotorClient::scan_callback, this, _1));
    
    // Create an IMU Subscription to track the angle on the 'imu/imu' topic (Not currently in use)
    //imu_subscription = this->create_subscription<sensor_msgs::msg::Imu>("imu/imu", rclcpp::QoS(rclcpp::SensorDataQoS()) /*QoS for sensors (best effort etc)*/, std::bind(&StepperMotorClient::imu_callback, this, _1));
    
    // Timer to automatically capture scan after 0.2s delay
    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&StepperMotorClient::capture_scan, this));
}


// Takes a full scan using LiDAR, IMU, Stepper Motor
// Params: none
void StepperMotorClient::capture_scan()
{
    // Dissable Timer so this only runs once
    this->timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Starting Scan Capture!");

    // Send Command for Motor to be Levelled & Wait
    RCLCPP_INFO(this->get_logger(), "Levelling the Platform...");
    level_motor("imu/imu"); //TODO IMU Topic not currently used
    rclcpp::sleep_for(std::chrono::milliseconds(4000));

    // Rotate the Stepper Motor upwards to vertically straight up (TODO - according to IMU Readings OR just 90 deg if levelled out already)
    float speed_fast = 0.5; //rps
    float speed_slow = 0.1; //rps
    RCLCPP_INFO(this->get_logger(), "[SCAN] Panning Motor Up to Vertical");
    move_motor((float)-90, speed_fast);
    rclcpp::sleep_for(std::chrono::milliseconds(5000));

    // Start the Scan
    RCLCPP_INFO(this->get_logger(), "[SCAN] Attempting to start scanning...");
    this->scanning = true;
    // Now this variable is set to true, the Scan Callback Function will process each scan...

    // Rotate gradually downwards
    RCLCPP_INFO(this->get_logger(), "[SCAN] Panning Motor Down");
    move_motor((float)180, speed_slow);
    rclcpp::sleep_for(std::chrono::milliseconds(10000));

    // Stop Scan
    RCLCPP_INFO(this->get_logger(), "[SCAN] Stopping scan...");
    this->scanning = false;

    // Return to Horizontal position
    RCLCPP_INFO(this->get_logger(), "[SCAN] Panning Motor to Level");
    move_motor((float)-90, speed_fast);

    // Export PCL
    // TODO

    // Shut down Node
    rclcpp::shutdown();
}


// Called every time the node recieves a PointCloud2 Scan Message on the topic scan3d
void StepperMotorClient::scan_callback(const sensor_msgs::msg::PointCloud2 & pc_msg)
{
    std::cout << "SCAN CALLBACK" << std::endl;
    // Check if node in scanning state
    if (this->scanning == true) {
        RCLCPP_INFO(this->get_logger(), "[SCAN] Point Cloud Scan Successfully Called...");

        // Create a new PCL Point Cloud object
        pcl::PointCloud<pcl::PointXYZI> pcl;
        pcl::fromROSMsg(pc_msg, pcl);

        //DEBUG
        // PointCloudTools::PrintPointCloudInfo(pc_msg);
        // PointCloudTools::PrintPointCloud(pcl);

        // Transform Scan according to TF

        // Append PCL scans together into the final scan
        pcl_final += pcl;
    }
    // Once finished (by checking if pcl_final has anything in it)
    else if(pcl_final.empty()) {
        // Finished Scan
        //Export Point Cloud TODO
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

    // Create a Goal to send to the Stepper Driver
    auto goal_msg = Level::Goal();
    goal_msg.imu_topic = "imu/imu";

    // Send Goal to Stepper Driver
    RCLCPP_INFO(this->get_logger(), "Sending 'LEVEL' command to stepper motor action client");
    auto level_goal_options = rclcpp_action::Client<Level>::SendGoalOptions();
    level_goal_options.result_callback = std::bind(&StepperMotorClient::level_callback, this, _1);
    this->level_motor_client_ptr_->async_send_goal(goal_msg, level_goal_options);
    //auto response = level_client
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
    goal_msg.target_angle = target_angle;
    goal_msg.speed = speed;

    // Send Message (goal) to Stepper Motor Action Server
    RCLCPP_INFO(this->get_logger(), "Sending goal to stepper motor");
    auto send_goal_options = rclcpp_action::Client<StepperMotor>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&StepperMotorClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&StepperMotorClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&StepperMotorClient::result_callback, this, _1);
    this->move_motor_client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

// Logs information about the goal status
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
        // Set Self Variable
        this->levelled = true;
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