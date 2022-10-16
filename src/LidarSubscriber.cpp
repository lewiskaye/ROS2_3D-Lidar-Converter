//TODO Description

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

    //Define a projector to project laserscan to a 3D Point Cloud
    laser_geometry::LaserProjection projector;

    //Define a Point Cloud to store points in 3D Space
    sensor_msgs::msg::PointCloud2 cloud;

    //Project LaserScan onto a PointCloud
    projector.projectLaser(scan_msg, cloud);

    // 3D-ify the Point Cloud.
    //need an aditional object call here to split up code?

    // Transform/Rotate the 3D Point Cloud using TF Transformation Frames
    
    //Handle URDF File / 
    
    
    //Publish 3D Point Cloud
    pc_publisher->publish_pointcloud(cloud);

    //TF rotate for IMU


    //DEBUG
    //std::cout << "Point Cloud @90: " << cloud.data[90] << "\n";

    //Check if vector containing ranges isn't empty
    if (!scan_msg.ranges.empty()) 
    {
        //Store filtered ranges in a new array/vector
        auto filteredRanges = scan_msg.ranges; //May need to revisit 'copy' approach if it takes up too much memory

        //Convert to Numbered Array

        //For each point
            //Work out X, Y Coords (Z=0)
            //Convert to Point Cloud (see pre-made tutorials)

        //Store ranges in a string for temporary demo/debug.  Can be removed later
        //std::string tmp_ranges = "";

        int size = scan_msg.ranges.size();
        auto range = scan_msg.ranges[size - 1];
        auto angle = scan_msg.angle_increment * size * (180.0/M_PI); //TODO impliment function for to_deg

        //Calculate angle per-point assumung equal distribution - since the RPLiDAR Node doesn't publish angle inside the LaserScan message type.  TODO - I could impliment this?  This way, it would be a fork of the node, using SDK


        //std::cout << "Size: " << size << " | ";
        //std::cout << "Range: " << range << " | ";
        //std::cout << "Est Angle: " << angle  << "\n";


        //Convert either to point cloud, or to angle/range measurements

        //For each range in the vector
        for(float f : scan_msg.ranges) {
            
            
            //If not infinite or zero etc. - ALSO add check for < range_min and > range_max
            if (std::isnormal(f)) {
                
                
                //tmp_ranges = tmp_ranges + "," + std::to_string(f);
                //std::cout << std::to_string(scan_msg.ranges[i]) << ", ";
                //std::cout << f << "\n";
            }
            
        }

        //Log to console
        //RCLCPP_INFO(this->get_logger(), "Ranges: '%s'", tmp_ranges); 

    } else {
        //Empty Vector
        RCLCPP_INFO(this->get_logger(), "ERROR - empty vector of ranges");  //TODO can I declare a type 'error'?

    }

}

LidarSubscriber::~LidarSubscriber()
{
    // dispose clenly
  //LidarSubscriber::DisposeDriver(m_drv);
}

