#include "gtest/gtest.h"
#include "../include/LidarSubscriber.hpp"

// SanityCheck - should always return true
TEST(LidarConverterTestSuite, SanityCheck) {
    EXPECT_TRUE(true);
}


sensor_msgs::msg::LaserScan generate_random_laserscan() {
    sensor_msgs::msg::LaserScan scan;
    unsigned int num_readings = 4;
    double laser_frequency = 50;
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
      //std::cout << std::to_string(scan.ranges[i]) << ", "; //DEBUG
    }

    return scan;
}

sensor_msgs::msg::LaserScan generate_test_laserscan(double ranges[], unsigned int num_readings) {
    sensor_msgs::msg::LaserScan scan;
    //unsigned int num_readings = 4;
    double laser_frequency = 50;
    //double ranges[] = {1,2,3,4};
    //double intensities[] = {100,100,100,100};

    scan.header.frame_id = "laser_frame";
    scan.angle_min = -180 * (3.14159/180);
    scan.angle_max = 180 * (3.14159/180);
    scan.angle_increment = 360/num_readings * (3.14159/180);
    scan.time_increment = (1 / laser_frequency) / (num_readings);
    scan.range_min = 0.02;
    scan.range_max = 12.0;
    scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);
    for(unsigned int i = 0; i < num_readings; ++i)
    {
      scan.ranges[i] = ranges[i];
      scan.intensities[i] = 100;
    }

    return scan;
}

pcl::PointCloud<pcl::PointXYZI> laserscan_to_pcl(sensor_msgs::msg::LaserScan scan) {
    // Define a Point Cloud to store points in 3D Space
    sensor_msgs::msg::PointCloud2 cloud;
    pcl::PointCloud<pcl::PointXYZI> pcl;

    // Define a projector to project laserscan to a 3D Point Cloud
    laser_geometry::LaserProjection projector;
    projector.projectLaser(scan, cloud);

    pcl::fromROSMsg(cloud, pcl);

    return pcl;
}


TEST(LidarConverterTestSuite, TestLaserScanToPointCloud2) {
    // Generate fake scan message with fake points
    double test_ranges[] = {1,2,3,4};
    // PASS, PASS, PASS, PASS
    sensor_msgs::msg::LaserScan scan = generate_test_laserscan(test_ranges, 4);
    pcl::PointCloud<pcl::PointXYZI> pcl = laserscan_to_pcl(scan);

    EXPECT_NEAR(pcl.points[0].x, -test_ranges[0], 0.1);
    EXPECT_NEAR(pcl.points[1].y, -test_ranges[1], 0.1);
    EXPECT_NEAR(pcl.points[2].x, test_ranges[2], 0.1);
    EXPECT_NEAR(pcl.points[3].y, test_ranges[3], 0.1);
}

TEST(LidarConverterTestSuite, TestAbnormalValues) {
    // Generate fake scan message with fake points
    double test_ranges[] = {4.1321564,-4,13,99999};
    // PASS, FAIL, FAIL, FAIL
    sensor_msgs::msg::LaserScan scan = generate_test_laserscan(test_ranges, 4);
    pcl::PointCloud<pcl::PointXYZI> pcl = laserscan_to_pcl(scan);

    EXPECT_NEAR(pcl.points[0].x, -test_ranges[0], 0.1);
    EXPECT_NE(pcl.points[1].y, -test_ranges[1]);
    EXPECT_NE(pcl.points[2].x, test_ranges[2]);
    EXPECT_NE(pcl.points[3].y, test_ranges[3]);
}

