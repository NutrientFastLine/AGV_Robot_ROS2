import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


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

    base_serial_port = LaunchConfiguration('port', default='/dev/ttyUSB0')

    base_serial_port = '/dev/ttyTHS1'
    
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

    channel_type =  LaunchConfiguration('channel_type', default='serial')
    sllidar_serial_port = LaunchConfiguration('serial_port', default='/dev/sllidar_base')
    sllidar_serial_baudrate = LaunchConfiguration('serial_baudrate', default='256000')
    frame_id = LaunchConfiguration('frame_id', default='laser_link')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    sllidar_ros2_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[
            {
                'channel_type': channel_type,
                'serial_port': sllidar_serial_port,
                'serial_baudrate': sllidar_serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate
            }
        ],
        output='screen',
        )

    # #==========启动robot_localization 融合odom和imu数据========================================================
    
    # robot_start_pkg = FindPackageShare(package='robot_start').find('robot_start') 

    # ekf_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[os.path.join(robot_start_pkg, 'params', 'ekf.yaml')],
    # )   
    
    #===============================================定义启动文件========================================================

    ld = LaunchDescription()
    
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_state_publisher_node)

    ld.add_action(DS2024_motor_driver_node)
    ld.add_action(imu_driver_node)    
    ld.add_action(sllidar_ros2_node)

    # ld.add_action(ekf_node)

    return ld