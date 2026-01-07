#!/usr/bin/env python3
"""
Launch file for Dobot backend nodes (ROS 2 Jazzy)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Launch arguments
    planning_group = LaunchConfiguration('planning_group')

    declare_planning_group = DeclareLaunchArgument(
        'planning_group',
        default_value='dobot_arm',
        description='MoveIt planning group name'
    )

    # Motion planner node
    motion_planner_node = Node(
        package='dobot_backend',
        executable='motion_planner',
        name='dobot_motion_planner',
        output='screen',
        parameters=[{
            'planning_group': planning_group
        }]
    )

    # State manager node
    state_manager_node = Node(
        package='dobot_backend',
        executable='state_manager',
        name='dobot_state_manager',
        output='screen'
    )

    return LaunchDescription([
        declare_planning_group,
        motion_planner_node,
        state_manager_node
    ])