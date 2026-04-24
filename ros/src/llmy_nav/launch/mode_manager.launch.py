#!/usr/bin/env python3
"""
Mode Manager Launch File for LLMy Robot

Launches the navigation mode manager which orchestrates
switching between navigation, mapping, and mapfree modes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    maps_dir = LaunchConfiguration('maps_dir')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'maps_dir',
            default_value='~/.llmy/maps',
            description='Directory for map storage'
        ),

        Node(
            package='llmy_nav',
            executable='mode_manager.py',
            name='mode_manager',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'maps_dir': maps_dir},
            ]
        ),
    ])
