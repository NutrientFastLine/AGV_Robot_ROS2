# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    port = LaunchConfiguration('port', default='/dev/ttyUSB0')
    baudrate = LaunchConfiguration('baudrate', default= '921600') 

    port = "/dev/imu_base"
    
    imu_driver_node = Node(
        package="imu_driver",
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

    ld.add_action(imu_driver_node)


    return ld
