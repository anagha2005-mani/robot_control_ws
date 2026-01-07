#!/usr/bin/env python3
"""
Launch file for Dobot Web Interface (ROS 2 Jazzy)

Starts the Flask-based frontend web server node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Launch arguments
    web_host = LaunchConfiguration('web_host')
    web_port = LaunchConfiguration('web_port')

    declare_web_host = DeclareLaunchArgument(
        'web_host',
        default_value='0.0.0.0',
        description='Host IP for the web server'
    )

    declare_web_port = DeclareLaunchArgument(
        'web_port',
        default_value='5000',
        description='Port for the web server'
    )

    # Web server node
    web_server_node = Node(
        package='dobot_frontend',
        executable='web_server',
        name='dobot_web_server',
        output='screen',
        parameters=[{
            'web_host': web_host,
            'web_port': web_port,
        }]
    )

    return LaunchDescription([
        declare_web_host,
        declare_web_port,
        web_server_node,
    ])
