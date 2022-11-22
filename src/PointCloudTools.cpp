// This class contains some custom made, useful utilities not found within ROS for dealing with Point Clouds

// Definitions are placed in the header file
#include "../include/PointCloudTools.hpp"


void PointCloudTools::PrintPointCloud(pcl::PointCloud<pcl::PointXYZI> cloud){
    // Display Cloud Width and Height
    printf ("Cloud: width = %d, height = %d\n", cloud.width, cloud.height);

    for(auto &pt : cloud.points) {
        printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
    }
}

void PointCloudTools::PrintPointCloudInfo(sensor_msgs::msg::PointCloud2 cloud){
    // Display Cloud Width and Height
    printf ("Cloud: width = %d, height = %d\n", cloud.width, cloud.height);
}

pcl::PointCloud<pcl::PointXYZI> PointCloudTools::laserscan_to_pcl(sensor_msgs::msg::LaserScan scan) {
    // Define a Point Cloud to store points in 3D Space
    sensor_msgs::msg::PointCloud2 cloud;
    pcl::PointCloud<pcl::PointXYZI> pcl;

    // Define a projector to project laserscan to a 3D Point Cloud
    laser_geometry::LaserProjection projector;
    projector.projectLaser(scan, cloud);

    pcl::fromROSMsg(cloud, pcl);

    return pcl;
}

sensor_msgs::msg::PointCloud2 PointCloudTools::laserscan_to_pc2(sensor_msgs::msg::LaserScan scan) {
    // Define a Point Cloud to store points in 3D Space
    sensor_msgs::msg::PointCloud2 cloud;
    pcl::PointCloud<pcl::PointXYZI> pcl;

    // Define a projector to project laserscan to a 3D Point Cloud
    laser_geometry::LaserProjection projector;
    projector.projectLaser(scan, cloud);

    return cloud;
}


sensor_msgs::msg::LaserScan PointCloudTools::generate_random_laserscan(unsigned int num_readings, double laser_frequency) {
    // Store Scan in variable
    sensor_msgs::msg::LaserScan scan;
    
    double ranges[num_readings];
    double intensities[num_readings];
    int count = 0;
    for(unsigned int i = 0; i < num_readings; ++i)
    {
    ranges[i] = rand() % 12;
    intensities[i] = 100 + count;
    }
    scan.header.frame_id = "laser_frame";
    scan.angle_min = -180 * (3.14/180);
    scan.angle_max = 180 * (3.14/180);
    scan.angle_increment = 360/num_readings * (3.14/180);
    scan.time_increment = (1 / laser_frequency) / (num_readings);
    scan.range_min = 0.02;
    scan.range_max = 12.0;
    scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);
    for(unsigned int i = 0; i < num_readings; ++i)
    {
    scan.ranges[i] = ranges[i];
    scan.intensities[i] = intensities[i];
    //std::cout << std::to_string(scan.ranges[i]) << ", " << std::endl; //DEBUG
    }

    return scan;
}
