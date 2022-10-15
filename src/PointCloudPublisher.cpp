//TODO Description

// Definitions are placed in the header file
#include "../include/PointCloudPublisher.hpp"


// Constructer - run ONCE upon creation of the PointCloudPublisher object (one object per Lidar Subscriber node)
PointCloudPublisher::PointCloudPublisher() : Node("converter_2d_to_3d") 
{
    // Create a publisher to the '/scan3d' topic
    publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan3d", rclcpp::QoS(rclcpp::SensorDataQoS()));
}

// Method invoked when an new message is detected from the topic
void PointCloudPublisher::publish_pointcloud(sensor_msgs::msg::PointCloud2 & pc)
{
    //Place Logic here if needed

    //Publish Point Cloud
    publisher->publish(pc);
    
    //Log to console
    RCLCPP_INFO(this->get_logger(), "Point CLoud Published");  


}


