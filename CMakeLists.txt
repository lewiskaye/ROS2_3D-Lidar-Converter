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
#ament_auto_find_build_dependencies(REQUIRED sensor_msgs)


# Stepper Motor Action Client
add_library(stepper_client SHARED
  src/StepperMotorClient.cpp)
target_include_directories(stepper_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(stepper_client
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(stepper_client
  "custom_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(stepper_client PLUGIN "lidar_converter::StepperMotorClient" EXECUTABLE stepper_motor_client)

# Converter Node
add_executable(converter_2d_to_3d src/converter_2d_to_3d.cpp src/LidarSubscriber.cpp src/PointCloudPublisher.cpp)
ament_target_dependencies(converter_2d_to_3d rclcpp std_msgs sensor_msgs laser_geometry)
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

# Stepper Motor Client Node
#add_executable(stepper_client src/stepper_client.cpp)
#ament_target_dependencies(stepper_client
#  rclcpp example_interfaces)

# Install Nodes into ROS 2
install(TARGETS converter_2d_to_3d imu stepper_client
  DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
  
install(DIRECTORY description launch rviz
  DESTINATION share/${PROJECT_NAME}
)

#Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
