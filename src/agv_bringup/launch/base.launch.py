import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    #==========启动robot_desscription URDF文件========================================================
    
    description_pkg_share = FindPackageShare(package='agv_description').find('agv_description') 
    description_model_path = os.path.join(description_pkg_share, f'urdf/{"agv_base.urdf"}')

    urdf_model = LaunchConfiguration('model',default=description_model_path)

    robot_description = {
        'robot_description': Command(['cat ', urdf_model])
    }
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )
  

    #=====================运行底盘节点需要的配置==========================================================

    base_serial_port = LaunchConfiguration('port', default='/dev/ds2024_base')
    
    ds2024_driver_node = Node(
        package='ds2024_driver',
        executable='ds2024_driver_node',
        name='ds2024_driver_node',
        parameters=[{'serial_name': base_serial_port}],
        output='screen',
    )

    #=====================运行imu节点需要的配置==========================================================

    imu_serial_port = LaunchConfiguration('port', default='/dev/imu_base')
    imu_baudrate = LaunchConfiguration('baudrate', default= '921600') 
    
    handsfree_driver_node = Node(
        package="handsfree_driver",
        executable="handsfree_driver_node",
        name="handsfree_driver_node",
        parameters=[
            {
                'port': imu_serial_port,
                'baudrate': imu_baudrate,
            }
        ],
        output='screen',
    )

    # #==========启动robot_localization 融合odom和imu数据========================================================

    robot_localization_config_pkg = get_package_share_directory('agv_bringup')
    robot_localization_config_path = os.path.join(robot_localization_config_pkg, 'config', 'ekf.yaml')

    # 确保路径存在
    if not os.path.exists(robot_localization_config_path):
        raise FileNotFoundError(f"Could not find ekf.yaml at {robot_localization_config_path}")

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_config_path],
        remappings=[
            ('/odometry/filtered', '/odom')  # 这里重映射 odom 话题
            ],
    )   

    #===============================================定义启动文件========================================================

    ld = LaunchDescription()
    
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_state_publisher_node)

    ld.add_action(ds2024_driver_node)
    ld.add_action(handsfree_driver_node)    

    ld.add_action(robot_localization_node)

    return ld