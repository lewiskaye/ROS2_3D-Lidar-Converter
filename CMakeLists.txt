cmake_minimum_required(VERSION 3.8)
project(lidar_converter)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find Dependencies
# Format: find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.8 REQUIRED)
#1.3 alternative
#ament_auto_find_build_dependencies(REQUIRED sensor_msgs)

# PCL
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Converter Node
add_executable(converter_2d_to_3d src/converter_2d_to_3d.cpp src/LidarSubscriber.cpp src/PointCloudPublisher.cpp src/PointCloudTools.cpp)
target_link_libraries(converter_2d_to_3d ${PCL_LIBRARIES})
ament_target_dependencies(converter_2d_to_3d rclcpp rclcpp_action std_msgs sensor_msgs laser_geometry pcl_conversions)
target_include_directories(converter_2d_to_3d PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(converter_2d_to_3d PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# IMU Node
add_executable(imu src/IMU.cpp src/IMUFramePublisher.cpp)
ament_target_dependencies(imu geometry_msgs tf2_geometry_msgs rclcpp tf2 tf2_ros std_msgs sensor_msgs)
target_include_directories(imu PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(imu PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Capture Scan Client Node
add_executable(capture_scan src/capture_scan.cpp src/StepperMotorClient.cpp src/PointCloudTools.cpp)
target_link_libraries(capture_scan ${PCL_LIBRARIES})
ament_target_dependencies(capture_scan custom_interfaces rclcpp rclcpp_action rclcpp_components std_msgs sensor_msgs tf2 tf2_geometry_msgs laser_geometry pcl_conversions)
target_include_directories(capture_scan PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(capture_scan PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Install Nodes into ROS 2
install(TARGETS converter_2d_to_3d imu capture_scan
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY description launch rviz
  DESTINATION share/${PROJECT_NAME}
)

#Testing
if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # # # the following line skips the linter which checks for copyrights
  # # # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # # # the following line skips cpplint (only works in a git repo)
  # # # comment the line when this package is in a git repo and when
  # # # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

  # Find GoogleTest
  find_package(ament_cmake_gtest REQUIRED)

  set(TESTFILES
    test/main.cpp
    test/lidar_converter_test.cpp
    test/imu_test.cpp
    src/IMUFramePublisher.cpp
    src/PointCloudTools.cpp)

  # Add gtest executables
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp rclcpp_action rclcpp_components std_msgs sensor_msgs laser_geometry tf2 tf2_ros geometry_msgs tf2_geometry_msgs example_interfaces custom_interfaces pcl_conversions PCL)


  install(TARGETS
    ${PROJECT_NAME}_test
    # Install the generated test executable file to the path after DESTINATION
    DESTINATION lib/${PROJECT_NAME})

endif()

ament_package()