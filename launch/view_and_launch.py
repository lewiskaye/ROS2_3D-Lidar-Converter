#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # Parameters for the RPLiDAR LiDAR Sensor
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200') #for RPLiDAR Sensors A1/A2 set to 115200
    frame_id = LaunchConfiguration('frame_id', default='lidar_sensor')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')

    # Parameters for IMU Default Child Link/Frame
    imu_frame = LaunchConfiguration('imu_frame', default='imu_angle_adjustment')

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Parameters for the RVIZ config
    rviz_config_dir = os.path.join(get_package_share_directory('lidar_converter'), 'rviz', 'rplidar_3d.rviz')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('lidar_converter'))
    xacro_file = os.path.join(pkg_path,'description','robot_urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # Create a robot_state_publisher node
    rsp_params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[rsp_params]
    )

    # Launch
    return LaunchDescription([
        # Serial Port
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),
        # Serial Baudrate
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        # Frame ID
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),
        # Inverted
        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),
        # Angle Compensate
        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),
        # Use Sim Time
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        # child_frame for IMU
        DeclareLaunchArgument(
            'imu_frame',
            default_value=imu_frame,
            description='Default child frame for the IMU to use'),

        # Launch Robot State Publisher - publishes the state of the robot
        node_robot_state_publisher,

        # RPLiDAR Node
        Node(
            package='slamtec_rplidar_ros2',
            executable='rplidar_scan_publisher',
            name='rplidar_scan_publisher',
            parameters=[{'serial_port': serial_port, 
                         'serial_baudrate': serial_baudrate, 
                         'frame_id': frame_id,
                         'inverted': inverted, 
                         'angle_compensate': angle_compensate}],
            output='screen'),
        
        # LiDAR Converter Node
        Node(
            package='lidar_converter',
            executable='converter_2d_to_3d',
            name='converter_2d_to_3d',
            output='screen'),

        # IMU Node
        Node(
            package='lidar_converter',
            executable='imu',
            name='imu',
            parameters=[{'imu_frame': imu_frame}],
            output='screen'),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'),

        # Joint State Publisher
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'),
    ])

