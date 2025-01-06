import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    #=============================1. 获取包的路径 ===========================================
    agv_launch_dir = get_package_share_directory('agv_launch')    
    astra_camera_dir = get_package_share_directory('astra_camera') 

    #=============================2. 定义launch文件路径 ====================================
    base_launch_file = os.path.join(agv_launch_dir, 'launch', 'base.launch.py')
    navigation2_launch_file = os.path.join(agv_launch_dir, 'launch', 'navigation2.launch.py')
    astra_mini_launch_file = os.path.join(astra_camera_dir, 'launch', 'astra_mini.launch.py')

    #=============================3. 声明启动launch文件 ====================================
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_file)
    )
    
    navigation2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation2_launch_file)
    )

    astra_mini_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(astra_mini_launch_file)
    )
    
    #=============================4. 定义并返回LaunchDescription ==========================
    ld = LaunchDescription()
    # ld.add_action(base_launch)
    # ld.add_action(astra_mini_launch)
    ld.add_action(navigation2_launch)

    return ld
