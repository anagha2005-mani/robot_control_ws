from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

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
        FindPackageShare("dobot_moveit_config"),
        "config",
        "ros2_controllers.yaml"
    ])

    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description},
                controllers_yaml   # 👈 THIS must be a separate entry
            ],
            output="screen",
        )
    ])
