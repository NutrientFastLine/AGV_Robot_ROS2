import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    #=============================1. 获取包的路径 ===========================================
    agv_launch_dir = get_package_share_directory('agv_bringup')    

    #=============================2. 定义launch文件路径 ====================================
    base_launch_file = os.path.join(agv_launch_dir, 'launch', 'base.launch.py')
    sllidar_launch_file = os.path.join(agv_launch_dir, 'launch', 'sllidar.launch.py')
    navigation2_launch_file = os.path.join(agv_launch_dir, 'launch', 'navigation2.launch.py')
    collision_launch_file = os.path.join(agv_launch_dir, 'launch', 'collision_monitor.launch.py')

    #=============================3. 声明启动launch文件 ====================================
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_file)
    )
    
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sllidar_launch_file)
    )
    
    navigation2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation2_launch_file)
    )
    
    #=============================5. 定义并返回LaunchDescription ==========================
    ld = LaunchDescription()
    ld.add_action(base_launch)
    ld.add_action(sllidar_launch)
    # ld.add_action(navigation2_launch)

    return ld
