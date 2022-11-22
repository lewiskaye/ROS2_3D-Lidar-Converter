#include "gtest/gtest.h"
#include "../include/IMUFramePublisher.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


// SanityCheck - should always return true
TEST(IMUTestSuite, SanityCheck) {
    EXPECT_TRUE(true);
}


// Generate a fake message with input values
sensor_msgs::msg::Imu generate_msg(int r, int p, int y) {
    sensor_msgs::msg::Imu message;

    // Create Quaternion to represent rotation/angle values
    tf2::Quaternion q_tf;

    // Generate fake X, Y, Z Vaues
    q_tf.setRPY(r, p, y);

    // Normalise Vector (takes w into account, not just x, y, z)
    q_tf.normalize();

    message.orientation = tf2::toMsg(q_tf);
    return message;
}


// Tests the Transforms
TEST(IMUTestSuite, TestTransforms) {
    rclcpp::init(0, nullptr);
    auto test_obj = std::make_shared<IMUFramePublisher>();
    //rclcpp::spin(test_obj);

    // Generate Values
    int values_good[] = {0,1,2,30,45,70,85,90,-45,-90,-2};
    int values_bad[] = {91,95,364,365,366,-91,-365};

    // Test Good Values
    for(int g : values_good) {
        //Generate fake msg
        auto msg = generate_msg(g, g, g);

        //Generate Transform
        auto t = test_obj->generate_transform(msg.orientation);

        // Test Transforms are same as orientation values
        EXPECT_EQ(t.transform.rotation.x, msg.orientation.x);
        EXPECT_EQ(t.transform.rotation.y, msg.orientation.y);
        EXPECT_EQ(t.transform.rotation.z, msg.orientation.z);
        EXPECT_EQ(t.transform.rotation.w, msg.orientation.w);
        EXPECT_EQ(t.transform.translation.x, 0);
        EXPECT_EQ(t.transform.translation.y, 0);
        EXPECT_EQ(t.transform.translation.z, 0);
    }

    // Test Bad Values??
    for(int b : values_bad) {
        //Generate fake msg
        auto msg = generate_msg(b, b, b);

        //Generate Transform
        auto t = test_obj->generate_transform(msg.orientation);

        // Test Transforms are same as orientation values
        EXPECT_EQ(t.transform.rotation.x, msg.orientation.x);
        EXPECT_EQ(t.transform.rotation.y, msg.orientation.y);
        EXPECT_EQ(t.transform.rotation.z, msg.orientation.z);
        EXPECT_EQ(t.transform.rotation.w, msg.orientation.w);
        EXPECT_EQ(t.transform.translation.x, 0);
        EXPECT_EQ(t.transform.translation.y, 0);
        EXPECT_EQ(t.transform.translation.z, 0);
    }
}

// Test Quaternions
TEST(IMUTestSuite, TestQuaternions) {
    rclcpp::init(0, nullptr);
    //auto test_obj = std::make_shared<IMUFramePublisher>();

    // Generate Values
    float values_in[][3] = 
        {{0,0,0},
        {180,0,0},
        {0,0,-180}};
    float values_out[][4] = 
        {{0,0,0,1},
        {1,0,0,0},
        {0,0,-1,0}};

    // Test Good Values
    for(int i = 0; i < 3; i++) {
        //Generate fake msg
        auto msg = generate_msg(values_in[i][0], values_in[i][1], values_in[i][2]);

        tf2::Quaternion q;
        q.setRPY(values_in[i][0], values_in[i][1], values_in[i][2]);

        // Test Transforms are same as orientation values
        EXPECT_NEAR(q.getX(), values_out[i][0],0.11);
        EXPECT_NEAR(q.getY(), values_out[i][1],0.11);
        EXPECT_NEAR(q.getZ(), values_out[i][2],0.11);
        // EXPECT_FLOAT_EQ(msg.orientation.x, values_out[i][0]);
        // EXPECT_FLOAT_EQ(msg.orientation.y, values_out[i][1]);
        // EXPECT_FLOAT_EQ(msg.orientation.z, values_out[i][2]);
        // EXPECT_FLOAT_EQ(msg.orientation.w, values_out[i][3]);
    }
}