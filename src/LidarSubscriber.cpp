// This file handles the conversion of LaserScan to Point Cloud
// Note - the Transformations are applied elsewhere

// Definitions are placed in the header file
#include "../include/LidarSubscriber.hpp"


// Constructer - run ONCE when the node is created
LidarSubscriber::LidarSubscriber() : Node("converter_2d_to_3d") //lidar_subscriber
{
    // Create a subscription to the '/scan' topic
    subscription = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(rclcpp::SensorDataQoS()) /*QoS for sensors (best effort etc)*/, std::bind(&LidarSubscriber::scan_callback, this, _1));
    
    // Create a new Point Cloud Publisher object to handle the publishing after all the 2D laser scan processing is done
    pc_publisher = new PointCloudPublisher();
}

// Method invoked when an new message is detected from the topic
void LidarSubscriber::scan_callback(const sensor_msgs::msg::LaserScan & scan_msg) const
{
    // Scan message is stored in a LaserScan type variable passed in callback method parameters
    //Log Scan Recieved
    //RCLCPP_INFO(this->get_logger(), "Scan Recieved.  Scan Time: '%f'", scan_msg.scan_time); 

    // Check if vector containing ranges isn't empty
    if (scan_msg.ranges.empty()) 
    {
        RCLCPP_INFO(this->get_logger(), "Empty vector of ranges");
    }

    //Define a projector to project laserscan to a 3D Point Cloud
    laser_geometry::LaserProjection projector;

    //Define a Point Cloud to store points in 3D Space
    sensor_msgs::msg::PointCloud2 cloud;

    //Project LaserScan onto a PointCloud
    projector.projectLaser(scan_msg, cloud);

    // The Transformations are applied from the IMU Frame Publisher Node    
    
    //Publish Point Cloud
    pc_publisher->publish_pointcloud(cloud);


    //DEBUG
    //std::cout << "Point Cloud @90: " << cloud.data[90] << "\n";
}



LidarSubscriber::~LidarSubscriber()
{
    // Dispose Cleanly
    rclcpp::shutdown();
}

