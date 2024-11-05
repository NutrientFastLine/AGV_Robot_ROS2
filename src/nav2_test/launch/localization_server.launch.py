import os
 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
 
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    nav2_yaml = os.path.join(get_package_share_directory('nav2_test'), 'config', 'amcl.yaml')
    map_file = os.path.join(get_package_share_directory('nav2_test'), 'maps', '325_00.yaml')
 
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    autostart = LaunchConfiguration('autostart', default='True')

    nav2_map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, 
                    {'yaml_filename':map_file} ]
    )

    nav2_amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_yaml]
    )

    nav2_lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': ['map_server', 'amcl']}]
    )            
 

    ld = LaunchDescription()

    ld.add_action(nav2_map_server_node)
    ld.add_action(nav2_amcl_node)
    
    ld.add_action(nav2_lifecycle_manager_node)


    return ld