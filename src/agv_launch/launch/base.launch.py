import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    #==========启动robot_desscription URDF文件========================================================
    
    desscription_pkg_share = FindPackageShare(package='robot_description').find('robot_description') 
    desscription_urdf_model_path = os.path.join(desscription_pkg_share, f'urdf/{"agv_base.urdf"}')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[desscription_urdf_model_path]
        )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[desscription_urdf_model_path]
        )    

    #=====================运行底盘节点需要的配置==========================================================

    base_serial_port = LaunchConfiguration('port', default='/dev/ttyTHS1')

    base_serial_port = '/dev/ds2024_base'
    
    DS2024_motor_driver_node = Node(
        package='base_control',
        executable='DS2024_motor_driver_node',
        name='DS2024_motor_driver_node',
        parameters=[
            {
                'serial_name': base_serial_port,
            }
        ],
        output='screen',
    )

    #=====================运行imu节点需要的配置==========================================================

    imu_serial_port = LaunchConfiguration('port', default='/dev/ttyUSB0')
    imu_baudrate = LaunchConfiguration('baudrate', default= '921600') 

    imu_serial_port = "/dev/imu_base"
    
    imu_driver_node = Node(
        package="imu_driver_py",
        executable="imu_driver_node",
        name="imu_driver_node",
        parameters=[
            {
                'port': imu_serial_port,
                'baudrate': imu_baudrate,
            }
        ],
        output='screen',
    )

    #=====================运行雷达节点需要的配置==========================================================

    sllidar_channel_type =  LaunchConfiguration('channel_type', default='serial')
    sllidar_serial_port = LaunchConfiguration('serial_port', default='/dev/sllidar_base')
    sllidar_serial_baudrate = LaunchConfiguration('serial_baudrate', default='256000')
    sllidar_frame_id = LaunchConfiguration('frame_id', default='laser_link')
    sllidar_inverted = LaunchConfiguration('inverted', default='false')
    sllidar_angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    sllidar_scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    sllidar_ros2_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[
            {
                'channel_type': sllidar_channel_type,
                'serial_port': sllidar_serial_port,
                'serial_baudrate': sllidar_serial_baudrate,
                'frame_id': sllidar_frame_id,
                'inverted': sllidar_inverted,
                'angle_compensate': sllidar_angle_compensate
            }
        ],
        output='screen',
        )

    # #==========启动robot_localization 融合odom和imu数据========================================================

    # declare_robot_localization_config = DeclareLaunchArgument(
    #     'robot_localization_config',
    #     default_value=os.path.join(
    #         FindPackageShare('agv_launch').find('agv_launch'),
    #         'config', 'ekf.yaml'
    #     ),
    #     description='Full path to the robot localization config file (ekf.yaml)'
    # )
    
    # robot_localization_config_pkg = FindPackageShare(package='agv_launch').find('agv_launch') 
    # robot_localization_config_path = os.path.join(robot_localization_config_pkg, f'config/{"ekf.yaml"}')
    robot_localization_config_pkg = get_package_share_directory('agv_launch')
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
    )   

    #=====================运行需要的配置==========================================================



    #===============================================定义启动文件========================================================

    ld = LaunchDescription()
    
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_state_publisher_node)

    ld.add_action(DS2024_motor_driver_node)
    ld.add_action(imu_driver_node)    
    ld.add_action(sllidar_ros2_node)

    ld.add_action(robot_localization_node)

    return ld