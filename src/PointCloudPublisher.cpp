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
void PointCloudPublisher::publish_pointcloud(sensor_msgs::msg::PointCloud2 &pc)
{
    //Place Logic here if needed

    //Publish Point Cloud
    publisher->publish(pc);
    
    //Log to console
    RCLCPP_INFO(this->get_logger(), "Point Cloud Published");  
    //std::cout << pc.data << std::endl;
    
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl::fromROSMsg(pc, pcl_cloud);
    
    std::cout << pcl_cloud.points[0].x << pcl_cloud.points[0].y << pcl_cloud.points[0].z << std::endl;

    // for (auto point = pcl_cloud.points.begin(); point != pcl_cloud.points.end(); ++point) {
    //     std::cout << point->x << std::endl;
    // }


    // OLD PCL CODE
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(); // PCL still uses boost::shared_ptr internally
    // pcl::fromROSMsg(*pc, *cloud); // This will convert the message into a pcl::PointCloud

    
}


