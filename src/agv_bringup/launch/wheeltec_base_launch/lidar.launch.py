import os
from pathlib import Path
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    Lslidar_dir = get_package_share_directory('lslidar_driver')
    Lslidar_launch_dir = os.path.join(Lslidar_dir, 'launch')

    pointcloud_to_laserscan_dir = get_package_share_directory('pointcloud_to_laserscan')
    pointcloud_to_laserscan_launch_dir = os.path.join(pointcloud_to_laserscan_dir, 'launch')
           
    LSCX = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(Lslidar_launch_dir, 'lslidar_cx_launch.py')),)

    
    point_to_scan = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pointcloud_to_laserscan_launch_dir, 'pointcloud_to_laserscan_launch.py')),)
    

    sllidar_dir = get_package_share_directory('agv_bringup')
    sllidar_launch_dir = os.path.join(sllidar_dir, 'launch')

    SLM12 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sllidar_launch_dir, 'sllidar_a2m12.launch.py')),)

    # Create the launch description and populate
    ld = LaunchDescription()
    '''
    Please select your lidar here, options include:
    LSC16,LSC32,ld14、Ld06.
    1.If you are using LS* lidar (including lsn10, lsm10*), please don't forget to 
    modify the tf conversion parameters of robot_mode_description.launch.py
    according to the user guide file.
    2.If you are using m10 lidar, please pay attention to distinguish whether it is m10p or not.
    '''
#     ld.add_action(SLM12)
    ld.add_action(LSCX)
    ld.add_action(point_to_scan)
    return ld

