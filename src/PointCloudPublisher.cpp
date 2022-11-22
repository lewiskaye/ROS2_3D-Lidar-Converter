// This file is responsible for actually taking a Point Cloud
// passed in, and publishing it as a Point Cloud for
// any other ROS packages to use as a topic.  

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
    //Publish Point Cloud
    publisher->publish(pc);
    
    //Log to console
    RCLCPP_INFO(this->get_logger(), "Point Cloud Published");  
    //std::cout << pc.data << std::endl;
    
    // pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    // pcl::fromROSMsg(pc, pcl_cloud);
    // PrintPointCloud(pcl_cloud)

}

void PointCloudPublisher::PrintPointCloud(pcl::PointCloud<pcl::PointXYZI> cloud){
    for(auto &pt : cloud.points) {
        printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
    }
}
