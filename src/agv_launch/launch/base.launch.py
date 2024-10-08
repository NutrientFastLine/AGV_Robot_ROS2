# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    port = LaunchConfiguration('port', default='/dev/ttyUSB1')


    port = '/dev/ds2024_base'
    
    DS2024_motor_driver_node = Node(
        package='base_control',
        executable='DS2024_motor_driver_node',
        name='DS2024_motor_driver_node',
        parameters=[
            {
                'serial_name': port, # 配置为参数
            }
        ],
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(DS2024_motor_driver_node)


    return ld

