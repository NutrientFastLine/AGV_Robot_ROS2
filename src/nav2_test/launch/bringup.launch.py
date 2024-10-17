import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    #=============================1.定位到包的地址=============================================================
    nav2_test_dir = get_package_share_directory('nav2_test')    
    
    #=============================2.声明参数，获取配置文件路径===================================================
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='True')

    #=============================3.声明启动launch文件，传入：地图路径、是否使用仿真时间以及nav2参数文件==============
    localization_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_test_dir,'/launch','/localization_server.launch.py']),
        launch_arguments={                
            'use_sim_time': use_sim_time,
            'autostart': autostart}.items(),
        )
    
    navigation_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_test_dir,'/launch','/navigation_server.launch.py']),
        launch_arguments={                
            'use_sim_time': use_sim_time,
            'autostart': autostart}.items(),
    )
    #=========定义启动文件========================================================
    ld = LaunchDescription()
    ld.add_action(localization_server_launch)
    ld.add_action(navigation_server_launch)

    return ld
