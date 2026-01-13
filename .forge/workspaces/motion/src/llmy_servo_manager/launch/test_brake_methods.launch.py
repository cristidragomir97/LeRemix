#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Launch file to test different braking methods"""
    
    # Get the package directory
    pkg_dir = get_package_share_directory('llmy_servo_manager_py')
    
    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'servo_manager.yaml')
    
    # Launch argument for brake method
    brake_method_arg = DeclareLaunchArgument(
        'brake_method',
        default_value='torque_disable',
        description='Braking method to test: torque_disable, velocity_ramp, or position_brake'
    )
    
    # Test servo manager with specific brake method
    test_servo_manager = Node(
        package='llmy_servo_manager_py',
        executable='servo_manager_node',
        name='servo_manager_test',
        parameters=[
            config_file,
            {
                'brake_method': LaunchConfiguration('brake_method'),
                'loc_enable': True,
                'arm_enable': False,
                'head_enable': False,
                'loc_speed_scale': 0.3  # Slightly higher for testing
            }
        ],
        output='screen',
        emulate_tty=True,
        prefix='stdbuf -o L'  # Line buffered output for better logging
    )
    
    return LaunchDescription([
        brake_method_arg,
        test_servo_manager
    ])