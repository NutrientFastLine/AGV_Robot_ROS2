import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration

def generate_launch_description():
 
    controller_yaml = os.path.join(get_package_share_directory('nav2_test'), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('nav2_test'), 'config', 'bt.yaml')
    planner_yaml = os.path.join(get_package_share_directory('nav2_test'), 'config', 'planner.yaml')
    behavior_yaml = os.path.join(get_package_share_directory('nav2_test'), 'config', 'behavior.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    autostart = LaunchConfiguration('autostart', default='True')
    
    nav2_controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_yaml],
        remappings= [('cmd_vel', 'cmd_vel_raw')]
    )
    nav2_planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_yaml]
    )
    nav2_behaviors_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[behavior_yaml],
        output='screen'
    )

    nav2_bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_yaml]
    )
    nav2_lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': ['planner_server',
                                    'controller_server',
                                    'behavior_server',
                                    'bt_navigator']}]
    )

    ld = LaunchDescription()

    ld.add_action(nav2_controller_node)
    ld.add_action(nav2_planner_node)
    ld.add_action(nav2_behaviors_node)
    ld.add_action(nav2_bt_navigator_node)
    ld.add_action(nav2_lifecycle_manager_node)
    
    return ld