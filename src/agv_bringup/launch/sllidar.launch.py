#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    #==========启动robot_desscription URDF文件========================================================
    
    description_pkg_share = FindPackageShare(package='urdf_agv_description').find('urdf_agv_description') 
    description_model_path = os.path.join(description_pkg_share, f'urdf/{"agv_base.urdf"}')

    urdf_model = LaunchConfiguration('model',default=description_model_path)

    robot_description = {
        'robot_description': Command(['cat ', urdf_model])
    }
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/sllidar_base')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='256000')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    scan_topic = LaunchConfiguration('scan_topic', default='scan')
    inverted = LaunchConfiguration('inverted', default='true')
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
                'scan_topic': scan_topic,
                'inverted': inverted,
                'angle_compensate': angle_compensate
            }
        ],
        remappings=[
            ('/scan', '/scan_raw'),
        ],
        output='screen',
        )

    laser_tf_sync = Node(
        package='laser_tf_sync',
        executable='laser_tf_sync',
        name='laser_tf_sync',
        output='screen',
    )

    base_to_laser = Node(
        package='tf2_ros', 
        executable='static_transform_publisher', 
        name='base_to_laser',
        arguments=[
        '--x', '0',
        '--y', '0',
        '--z', '0.15',
        '--roll', '0',
        '--pitch', '3.1415946',
        '--yaw', '0',
        '--frame-id', 'base_link',
        '--child-frame-id', 'laser'
        ],
    )
    laser_filter_pkg_share = FindPackageShare(package='agv_bringup').find('agv_bringup') 
    laser_filter_config_path = os.path.join(laser_filter_pkg_share, f'config/{"agv_laser_filter.yaml"}')

    laser_filter_node =Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[
                laser_filter_config_path
            ],
            remappings=[
                ('/scan', '/scan_raw'),
                ('/scan_filtered', '/scan')
            ]
        )

    ld = LaunchDescription()

    # ld.add_action(joint_state_publisher_node)
    # ld.add_action(robot_state_publisher_node)
    ld.add_action(sllidar_ros2_node)
    # ld.add_action(laser_tf_sync)
    ld.add_action(base_to_laser)
    ld.add_action(laser_filter_node)

    return ld
