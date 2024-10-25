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
    launch_dir = os.path.join(nav2_test_dir, 'launch')
    astra_camera_dir = get_package_share_directory('astra_camera') 

    #=============================2. 声明参数 ==============================================
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='True')

    #=============================3. 定义launch文件路径 ====================================
    localization_server_launch_file = os.path.join(launch_dir, 'localization_server.launch.py')
    navigation_server_launch_file = os.path.join(launch_dir, 'navigation_server.launch.py')
    astra_mini_launch_file = os.path.join(astra_camera_dir, 'launch', 'astra_mini.launch.py')

    #=============================4. 声明启动launch文件 ====================================
    launch_arguments = {
        'use_sim_time': use_sim_time,
        'autostart': autostart
    }.items()

    localization_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_server_launch_file),
        launch_arguments=launch_arguments
    )

    navigation_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_server_launch_file),
        launch_arguments=launch_arguments
    )

    astra_mini_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(astra_mini_launch_file)
    )

    #=============================5. 定义并返回LaunchDescription ==========================
    ld = LaunchDescription()
    ld.add_action(astra_mini_launch)
    ld.add_action(localization_server_launch)
    ld.add_action(navigation_server_launch)

    return ld
