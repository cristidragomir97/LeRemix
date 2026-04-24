#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('llmy_imu')
    default_config = os.path.join(pkg_dir, 'config', 'imu.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to IMU configuration file')

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='Frame ID for IMU data')

    use_mag_arg = DeclareLaunchArgument(
        'use_mag',
        default_value='true',
        description='Use magnetometer for orientation filtering')

    # IMU sensor node (publishes imu/raw + imu/mag)
    imu_node = Node(
        package='llmy_imu',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')])

    # Madgwick filter for orientation fusion (imu/raw + imu/mag -> imu/data)
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'use_mag': LaunchConfiguration('use_mag'),
            'publish_tf': False,
            'world_frame': 'enu',
            'fixed_frame': LaunchConfiguration('frame_id'),
        }],
        remappings=[
            ('imu/data_raw', 'imu/raw'),
            ('imu/mag', 'imu/mag'),
            ('imu/data', 'imu/data'),
        ])

    return LaunchDescription([
        config_arg,
        frame_id_arg,
        use_mag_arg,
        imu_node,
        imu_filter_node,
    ])
