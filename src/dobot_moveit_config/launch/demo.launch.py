from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # -----------------------------
    # Build MoveIt configuration
    # -----------------------------
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="dobot_magician",
            package_name="dobot_moveit_config"
        )
        .robot_description(
            file_path="model/magician_moveit.urdf.xacro",
            mappings={
                "DOF": "4",
                "tool": "gripper",
                "use_camera": "false",
            },
        )
        .robot_description_semantic(
            file_path="config/dobot.srdf"
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml"
        )
        .planning_pipelines(
            pipelines=["ompl"]
        )
        .to_moveit_configs()
    )

    # -----------------------------
    # Robot State Publisher
    # -----------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description
        ],
    )

    # -----------------------------
    # Move Group
    # -----------------------------
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict()
        ],
    )

    # -----------------------------
    # RViz
    # -----------------------------
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
        ],
    )

    # -----------------------------
    # Launch
    # -----------------------------
    return LaunchDescription([
        robot_state_publisher,
        move_group_node,
        rviz_node,
    ])
