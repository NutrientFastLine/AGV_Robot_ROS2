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
    nav2_bringup_dir = get_package_share_directory('agv_launch')
    
    #=============================2.声明参数，获取配置文件路径===================================================
    nav2_bringup_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
    collision_monitor_launch_file = os.path.join(agv_launch_dir, 'launch', 'collision_monitor.launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_path = os.path.join(agv_launch_dir, 'maps', 'map.yaml')
    nav2_param_path = LaunchConfiguration('params_file',default=os.path.join(agv_launch_dir,'config','robot_nav2.yaml'))
    # nav2_param_path = LaunchConfiguration('params_file',default=os.path.join(agv_launch_dir,'config','nav2_params.yaml'))
    collision_param_path = LaunchConfiguration('params_file',default=os.path.join(agv_launch_dir,'config','collision_monitor_params.yaml'))

    #=============================3.声明启动launch文件，传入：地图路径、是否使用仿真时间以及nav2参数文件==============
    nav2_bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup_launch_file),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path
                }.items(),
        )

    collision_monitor_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(collision_monitor_launch_file)
        )

    ld = LaunchDescription()
    ld.add_action(nav2_bringup_launch)
    # ld.add_action(collision_monitor_launch)

    return ld
