# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    port = LaunchConfiguration('port', default='/dev/ttyUSB1')
    baudrate = LaunchConfiguration('baudrate', default= '921600') 

    port = "/dev/IMU_base"
    
    DS2024_motor_driver_node = Node(
        package="base_control",
        executable="DS2024_motor_driver_node",
        name="DS2024_motor_driver_node",
        parameters=[
            {
                'port': port,
                'baudrate': baudrate,
            }
        ],
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(DS2024_motor_driver_node)


    return ld
