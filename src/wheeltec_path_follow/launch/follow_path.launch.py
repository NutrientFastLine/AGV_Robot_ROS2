import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    #路径跟踪文件路径
    pathfilename = DeclareLaunchArgument('pathfilename',default_value='/home/wheeltec/wheeltec_ros2/src/wheeltec_path_follow/path/wheeltec_path')
    #是否循环进行路径跟踪
    run_in_loop = DeclareLaunchArgument('run_in_loop',default_value='True')

    follow_path = launch_ros.actions.Node(
            package='wheeltec_path_follow', 
            executable='follow_path.py', 
            name='follow_path',
            output='screen',
            parameters=[{'pathfilename': LaunchConfiguration('pathfilename')},
                    {'run_in_loop': LaunchConfiguration('run_in_loop')},]
    )


    return LaunchDescription([
        pathfilename,run_in_loop,follow_path
    ])

