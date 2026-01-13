#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_llmy_slam = get_package_share_directory('llmy_slam')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    ekf_config_arg = DeclareLaunchArgument(
        'ekf_config',
        default_value=os.path.join(pkg_llmy_slam, 'config', 'ekf.yaml'),
        description='Path to EKF config file'
    )

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    ekf_config = LaunchConfiguration('ekf_config')

    # EKF node for sensor fusion (wheel odom + IMU)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('odometry/filtered', '/odometry/filtered')
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        ekf_config_arg,
        ekf_node,
    ])
