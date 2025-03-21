import os

from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml

def generate_robot_node(robot_urdf,child):
    return launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'robot_state_publisher_{child}',
        arguments=[os.path.join(get_package_share_directory('urdf_agv_description'), 'urdf', robot_urdf)],
    )

def generate_static_transform_publisher_node(translation, rotation, parent, child):
    return launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'base_to_{child}',
        arguments=[
            '--x',     translation[0],
            '--y',     translation[1],
            '--z',     translation[2],
            '--roll',  rotation[0],
            '--pitch', rotation[1],
            '--yaw',   rotation[2],
            '--frame-id', parent,
            '--child-frame-id', child
        ],
    )
    
def generate_launch_description():

    agv_base = LaunchConfiguration('agv_base', default='true')
         
    agv_base_diff = GroupAction(
        condition=IfCondition(agv_base),
        actions=[
            generate_robot_node('agv_base.urdf','agv_base'),
            generate_static_transform_publisher_node(['0', '0', '0.255'], ['0', '3.1415926', '0'], 'base_footprint', 'laser'),
            # generate_static_transform_publisher_node(['0.11365', '0.00017', '0.0762'], ['0', '0', '0'], 'base_footprint', 'camera_link'),
    ])            

    
    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(agv_base_diff)

    return ld
