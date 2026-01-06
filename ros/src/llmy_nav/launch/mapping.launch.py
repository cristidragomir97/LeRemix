#!/usr/bin/env python3
"""
Mapping Launch File for LLMy Robot

Launches SLAM Toolbox for building maps:
- slam_toolbox: Async SLAM for real-time map building
- lifecycle_manager: Manages SLAM node lifecycle

Use this mode for creating maps of new environments.
Save maps using the map_saver_cli or via the mode_manager service.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('llmy_nav')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # Config file path
    slam_config = os.path.join(pkg_dir, 'config', 'slam_toolbox.yaml')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start lifecycle nodes'
        ),

        # SLAM Toolbox - Async mode for real-time mapping
        LifecycleNode(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_config,
                {'use_sim_time': use_sim_time},
            ]
        ),

        # Lifecycle Manager for SLAM
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_slam',
                    output='screen',
                    parameters=[
                        {'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'bond_timeout': 20.0},
                        {'node_names': ['slam_toolbox']},
                    ]
                ),
            ]
        ),
    ])
