import launch
import launch_ros
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_name = "agv_base.urdf"
    # 获取默认路径
    urdf_agv_path = get_package_share_directory('robot_description')
    default_model_path = urdf_agv_path + '/urdf/agv_base.urdf'

    # 为 Launch 声明参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='URDF 的绝对路径')
    # 获取文件内容生成新的参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['cat ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str)

    # 状态发布节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    # 关节状态发布节点
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        )

    ld = LaunchDescription()
    ld.add_action(action_declare_arg_mode_path)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_state_publisher_node)
    # ld.add_action(rviz2_node)

    return ld
