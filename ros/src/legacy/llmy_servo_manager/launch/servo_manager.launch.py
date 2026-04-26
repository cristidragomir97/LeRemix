#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('llmy_servo_manager')
    
    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'servo_manager.yaml')
    
    # Launch arguments
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to servo manager configuration file'
    )
    
    brake_method_arg = DeclareLaunchArgument(
        'brake_method',
        default_value='torque_disable',
        description='Braking method: torque_disable, velocity_ramp, or position_brake'
    )

    servo_port_arg = DeclareLaunchArgument(
        'servo_port',
        default_value='/dev/ttyTHS1',
        description='Serial port for servo manager'
    )

    servo_baud_arg = DeclareLaunchArgument(
        'servo_baud',
        default_value='1000000',
        description='Baudrate for servo manager'
    )

    # Servo manager node
    servo_manager_node = Node(
        package='llmy_servo_manager',
        executable='servo_manager_node',
        name='servo_manager_node',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'brake_method': LaunchConfiguration('brake_method'),
                'port': LaunchConfiguration('servo_port'),
                'baud': LaunchConfiguration('servo_baud')
            }
        ],
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0
    )
    
    return LaunchDescription([
        config_arg,
        brake_method_arg,
        servo_port_arg,
        servo_baud_arg,
        servo_manager_node
    ])