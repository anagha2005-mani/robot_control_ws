from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    robot_description = Command([
        'ros2 param get --hide-type /robot_state_publisher robot_description'
    ])

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('dobot_hardware_ros2'),
        'config',
        'ros2_controllers.yaml'
    ])

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_yaml
        ],
        output='screen'
    )

    return LaunchDescription([
        ros2_control_node
    ])
