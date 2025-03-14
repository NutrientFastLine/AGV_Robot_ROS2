import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    #=============================1.定位到包的地址=============================================================
    launch_package_dir = get_package_share_directory('agv_bringup')
    nav2_bringup_dir = get_package_share_directory('agv_bringup')
    
    
    #=============================2.声明参数，获取配置文件路径===================================================
    use_sim_time = LaunchConfiguration('use_sim_time', default='False') 

    map_yaml_path = LaunchConfiguration('map',default=os.path.join(launch_package_dir,'maps','map.yaml'))
    # map_yaml_path = LaunchConfiguration('map',default=os.path.join(launch_package_dir,'maps','toolbox_325map.yaml'))
    # map_yaml_path = LaunchConfiguration('map',default=os.path.join(launch_package_dir,'maps','gazebo_1.yaml'))

    nav2_param_path = LaunchConfiguration('params_file',default=os.path.join(launch_package_dir,'config','robot_inside_nav2.yaml'))
    # nav2_param_path = LaunchConfiguration('params_file',default=os.path.join(launch_package_dir,'config','robot_outside_nav2.yaml'))
    # nav2_param_path = LaunchConfiguration('params_file',default=os.path.join(launch_package_dir,'config','robot_gazebo_nav2.yaml'))

    #=============================3.声明启动launch文件，传入：地图路径、是否使用仿真时间以及nav2参数文件==============
    nav2_bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir,'/launch','/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        )
    
    return LaunchDescription([nav2_bringup_launch])
