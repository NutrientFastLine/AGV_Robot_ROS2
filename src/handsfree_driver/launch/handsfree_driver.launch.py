from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    port = LaunchConfiguration('port', default='/dev/imu_base')
    baudrate = LaunchConfiguration('baudrate', default='921600')

    handsfree_driver_node = Node(
        package="handsfree_driver",
        executable="handsfree_driver_node",
        name="handsfree_driver_node",
        parameters=[
            {
                'port': port,
                'baudrate': baudrate,
            }
        ],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(handsfree_driver_node)

    return ld