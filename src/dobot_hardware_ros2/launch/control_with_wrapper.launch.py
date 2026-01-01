from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution

def generate_launch_description():

    robot_description = Command([
        "xacro ",
        PathJoinSubstitution([
            FindPackageShare("dobot_moveit_config"),
            "config",
            "dobot.urdf.xacro"
        ])
    ])

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("dobot_hardware_ros2"),
        "config",
        "controllers.yaml"
    ])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_yaml
        ],
        output="screen"
    )

    return LaunchDescription([
        control_node
    ])

