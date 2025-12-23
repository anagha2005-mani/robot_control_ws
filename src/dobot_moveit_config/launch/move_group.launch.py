from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("dobot", package_name="dobot_moveit_config")
        .robot_description(file_path="config/dobot.urdf.xacro")
        .robot_description_semantic(file_path="config/dobot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    return generate_move_group_launch(moveit_config)

