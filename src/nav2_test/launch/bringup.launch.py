import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    #=============================1. 获取包的路径 ===========================================
    nav2_test_dir = get_package_share_directory('nav2_test')
    astra_camera_dir = get_package_share_directory('astra_camera') 
    agv_launch_dir = get_package_share_directory('agv_launch')  

    #=============================2. 定义launch文件路径 ====================================
    localization_server_launch_file = os.path.join(nav2_test_dir, 'launch', 'localization_server.launch.py')
    navigation_server_launch_file = os.path.join(nav2_test_dir, 'launch', 'navigation_server.launch.py')
    astra_mini_launch_file = os.path.join(astra_camera_dir, 'launch', 'astra_mini.launch.py')
    base_launch_file = os.path.join(agv_launch_dir, 'launch', 'base.launch.py')

    #=============================3. 声明参数 ==============================================
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='True')

    #=============================4. 声明启动launch文件 ====================================
    launch_arguments = {
        'use_sim_time': use_sim_time,
        'autostart': autostart
    }.items()

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_file)
    )

    astra_mini_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(astra_mini_launch_file)
    )

    localization_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_server_launch_file),
        launch_arguments=launch_arguments
    )

    navigation_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_server_launch_file),
        launch_arguments=launch_arguments
    )

    #=============================5. 定义并返回LaunchDescription ==========================
    ld = LaunchDescription()
    ld.add_action(base_launch)
    # ld.add_action(astra_mini_launch)
    ld.add_action(localization_server_launch)
    ld.add_action(navigation_server_launch)

    return ld
