# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

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

    port = LaunchConfiguration('port', default='/dev/ttyUSB0')
    baudrate = LaunchConfiguration('baudrate', default= '921600') 

    port = "/dev/imu_base"
    
    imu_driver_node = Node(
        package="imu_driver_py",
        executable="imu_driver_node",
        name="imu_driver_node",
        parameters=[
            {
                'port': port,
                'baudrate': baudrate,
            }
        ],
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(imu_driver_node)


    return ld
