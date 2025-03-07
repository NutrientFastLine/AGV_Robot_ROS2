#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    #==========启动robot_desscription URDF文件========================================================
    
    # description_pkg_share = FindPackageShare(package='urdf_agv_description').find('urdf_agv_description') 
    # description_model_path = os.path.join(description_pkg_share, f'urdf/{"agv_base.urdf"}')

    # urdf_model = LaunchConfiguration('model',default=description_model_path)

    # robot_description = {
    #     'robot_description': Command(['cat ', urdf_model])
    # }
    # robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     parameters=[robot_description]
    # )
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    # )

    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/sllidar_base')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='256000')
    frame_id = LaunchConfiguration('frame_id', default='laser_link')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    sllidar_ros2_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[
            {
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate
            }
        ],
        output='screen',
        )
    ld = LaunchDescription()

    # ld.add_action(joint_state_publisher_node)
    # ld.add_action(robot_state_publisher_node)
    
    ld.add_action(sllidar_ros2_node)

    return ld
