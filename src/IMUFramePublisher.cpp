// This file is responsible for the handling of the IMU
// including publishing the Transformations to the
// Point Cloud to rotate in 3D Space

// Definitions are placed in the header file
#include "../include/IMUFramePublisher.hpp"


// Constructer - run ONCE upon creation of the PointCloudPublisher object (one object per Lidar Subscriber node)
IMUFramePublisher::IMUFramePublisher() : Node("imu_frame_publisher") 
{
    // Declare and acquire 'child_frame' parameter
    child_frame_ = this->declare_parameter<std::string>("imu_frame", "imu_angle_adjustment");

    // Time for testing purposes every 1s - Disable for production
    //timer_ = this->create_wall_timer(10ms, std::bind(&IMUFramePublisher::test_data, this));
    //test_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/imu", 10);

    // Create a transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribe to IMU orientation topic and call handle_imu callback function on each message
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/imu", 
        rclcpp::QoS(rclcpp::SensorDataQoS()) /*QoS for sensors (best effort etc)*/,
        std::bind(&IMUFramePublisher::handle_imu, this, std::placeholders::_1));

}

// Method invoked when an new message is detected from the topic
void IMUFramePublisher::handle_imu(const std::shared_ptr<sensor_msgs::msg::Imu> msg)
{
    // Declare new Transfor Message
    geometry_msgs::msg::TransformStamped t = generate_transform(msg->orientation);
    
    // Send the transformation
    tf_broadcaster_->sendTransform(t);


    /* Unused code that may be useful - 
    tf2::Quaternion q;
    q.setRPY(msg->orientation.x, msg->orientation.y, msg->orientation.z); // TODO Modify this to access RPY from IMU
    //q.setW(msg->orientation.w);
    q.normalize();
    t.transform.rotation.x = q.x(); */

    //Log - Disabled due to too many prints
    //RCLCPP_INFO(this->get_logger(), "IMU TF Published");  
}

geometry_msgs::msg::TransformStamped IMUFramePublisher::generate_transform(geometry_msgs::msg::Quaternion orientation) {
    // Declare new Transfor Message
    geometry_msgs::msg::TransformStamped t;
    
    // Set Transform Frames with Child from Specified Parameter
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "base_link";
    t.child_frame_id = child_frame_.c_str();

    // Get x, y, z translation coordinates from the message
    // Keep static (0) since the sensor isn't moving in relation to the base link??
    t.transform.translation.x = 0;
    t.transform.translation.y = 0;
    t.transform.translation.z = 0;

    // Set Rotation based on the IMU's magnetometer readings
    //  ROS uses Quaternions for superior computational efficiency
    //  Since the IMU Message already contains a Quaternion, it can be passed straight through to the transform
    t.transform.rotation = orientation;

    // Return Transform
    return t;
}

void IMUFramePublisher::test_data()
{
    // Create a new IMU message
    sensor_msgs::msg::Imu message;

    // Create Quaternion to represent rotation/angle values
    tf2::Quaternion q_tf;

    // Generate fake X, Y, Z Vaues
    q_tf.setRPY(0, float(rand())/float((RAND_MAX)) * 0.1, 0);

    // Normalise Vector (takes w into account, not just x, y, z)
    q_tf.normalize();

    // Convert to Mesasge-type Quaternion, and add to IMU Message
    message.orientation = tf2::toMsg(q_tf);
    
    
    //Publish Test IMU Message
    RCLCPP_INFO(this->get_logger(), "IMU Test Data Published...");
    test_publisher_->publish(message);
}

IMUFramePublisher::~IMUFramePublisher() {
    // Dispose Cleanly
    rclcpp::shutdown();
}
