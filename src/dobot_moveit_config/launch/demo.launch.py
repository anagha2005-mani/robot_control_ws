from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
import os


def generate_launch_description():

    desc_pkg = get_package_share_directory("dobot_description")
    moveit_pkg = get_package_share_directory("dobot_moveit_config")

    # -----------------------------
    # Robot Description
    # -----------------------------
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ",
                os.path.join(
                    desc_pkg,
                    "model",
                    "magician_moveit.urdf.xacro"
                )
            ]),
            value_type=str,
        )
    }

    robot_description_semantic = {
        "robot_description_semantic": open(
            os.path.join(moveit_pkg, "config", "dobot.srdf")
        ).read()
    }

    # -----------------------------
    # robot_state_publisher (TF!)
    # -----------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # -----------------------------
    # joint_state_publisher
    # -----------------------------
    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    # -----------------------------
    # Move Group
    # -----------------------------
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            os.path.join(moveit_pkg, "config", "kinematics.yaml"),
            os.path.join(moveit_pkg, "config", "ompl_planning.yaml"),
            os.path.join(moveit_pkg, "config", "joint_limits.yaml"),
            {"planning_plugin": "ompl_interface/OMPLPlanner"},
        ],
    )

    # -----------------------------
    # RViz (delayed)
    # -----------------------------
    rviz = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    os.path.join(moveit_pkg, "config", "moveit.rviz"),
                ],
            )
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        move_group,
        rviz,
    ])
