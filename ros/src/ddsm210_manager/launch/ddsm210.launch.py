#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('ddsm210_manager')
    default_config = os.path.join(pkg_dir, 'config', 'ddsm210.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to DDSM210 configuration file')

    ddsm210_node = Node(
        package='ddsm210_manager',
        executable='ddsm210_node',
        name='ddsm210_node',
        # Pass the yaml first (node name match) AND an explicit ports override last.
        # Later params win, so this forces the bus to ACM4/ACM5 regardless of yaml.
        parameters=[
            LaunchConfiguration('config_file'),
            {'ports': ['/dev/ttyACM3', '/dev/ttyACM4']},
        ],
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0)

    return LaunchDescription([
        config_arg,
        ddsm210_node,
    ])
