#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('st3215_manager')
    default_config = os.path.join(pkg_dir, 'config', 'llmy.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to servo manager configuration file')

    servo_manager_node = Node(
        package='st3215_manager',
        executable='servo_manager_node',
        name='servo_manager_node',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0)

    return LaunchDescription([
        config_arg,
        servo_manager_node,
    ])
