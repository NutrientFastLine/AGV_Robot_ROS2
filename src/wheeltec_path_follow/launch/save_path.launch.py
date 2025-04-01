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

    save_path = launch_ros.actions.Node(
            package='wheeltec_path_follow', 
            executable='save_path', 
            name='save_path',
            output='screen',
            parameters=[{'pathfilename': LaunchConfiguration('pathfilename')},]
    )


    return LaunchDescription([
        pathfilename,save_path
    ])

