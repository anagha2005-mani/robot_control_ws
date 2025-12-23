from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="dobot",
            package_name="dobot_moveit_config"
        )
        .robot_description(file_path="config/dobot.urdf.xacro")
        .robot_description_semantic(file_path="config/dobot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description]
    )

    # Move Group Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        move_group_node,
        rviz_node,
    ])