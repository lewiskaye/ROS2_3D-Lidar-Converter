// This class contains some custom made, useful utilities not found within ROS for dealing with Point Clouds

// Definitions are placed in the header file
#include "../include/PointCloudTools.hpp"


class PointCloudTools
{
    static void PrintPointCloud(pcl::PointCloud<pcl::PointXYZI> cloud){
        // Display Cloud Width and Height
        printf ("Cloud: width = %d, height = %d\n", cloud.width, cloud.height);

        for(auto &pt : cloud.points) {
            printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
        }
    }

    static void PrintPointCloudInfo(sensor_msgs::msg::PointCloud2 cloud){
        // Display Cloud Width and Height
        printf ("Cloud: width = %d, height = %d\n", cloud.width, cloud.height);
    }

    static pcl::PointCloud<pcl::PointXYZI> laserscan_to_pcl(sensor_msgs::msg::LaserScan scan) {
        // Define a Point Cloud to store points in 3D Space
        sensor_msgs::msg::PointCloud2 cloud;
        pcl::PointCloud<pcl::PointXYZI> pcl;

        // Define a projector to project laserscan to a 3D Point Cloud
        laser_geometry::LaserProjection projector;
        projector.projectLaser(scan, cloud);

        pcl::fromROSMsg(cloud, pcl);

        return pcl;
    }

};