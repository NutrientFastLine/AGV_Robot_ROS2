import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    #=============================1.定位到包的地址=============================================================
    navigation2_dir = get_package_share_directory('agv_launch')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    
    #=============================2.声明参数，获取配置文件路径===================================================
    use_sim_time = LaunchConfiguration('use_sim_time', default='false') 
    map_yaml_path = LaunchConfiguration('map',default=os.path.join(navigation2_dir,'maps','325_1.yaml'))
    nav2_param_path = LaunchConfiguration('params_file',default=os.path.join(navigation2_dir,'config','robot_nav2.yaml'))
    # rviz_config_dir = os.path.join(nav2_bringup_dir,'rviz','nav2_default_view.rviz')

    #=============================3.声明启动launch文件，传入：地图路径、是否使用仿真时间以及nav2参数文件==============
    nav2_bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir,'/launch','/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        )
    
    ld = LaunchDescription()
    ld.add_action(nav2_bringup_launch)

    return ld
