import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    #=============================1.定位到包的地址=============================================================
    agv_launch_dir = get_package_share_directory('agv_launch')    

    #=============================2.声明启动launch文件，传入：地图路径、是否使用仿真时间以及nav2参数文件==============
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([agv_launch_dir,'/launch','/base.launch.py']),
        )
    
    navigation2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([agv_launch_dir,'/launch','/navigation2.launch.py']),
    )
    #=========定义启动文件========================================================
    ld = LaunchDescription()
    ld.add_action(base_launch)
    ld.add_action(navigation2_launch)

    return ld
