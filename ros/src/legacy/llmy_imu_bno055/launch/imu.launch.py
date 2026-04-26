#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100.0',
        description='Publishing rate for IMU data in Hz'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='Frame ID for IMU data'
    )

    i2c_bus_arg = DeclareLaunchArgument(
        'i2c_bus',
        default_value='7',
        description='I2C bus number'
    )

    i2c_addr_arg = DeclareLaunchArgument(
        'i2c_addr',
        default_value='40',  # 0x28 in decimal
        description='I2C address (decimal)'
    )

    # Create the IMU node
    # BNO055 provides fused orientation, so no external filter needed
    imu_node = Node(
        package='llmy_imu_bno055',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{
            'publish_rate': LaunchConfiguration('publish_rate'),
            'frame_id': LaunchConfiguration('frame_id'),
            'i2c_bus': LaunchConfiguration('i2c_bus'),
            'i2c_addr': LaunchConfiguration('i2c_addr'),
        }]
    )

    return LaunchDescription([
        publish_rate_arg,
        frame_id_arg,
        i2c_bus_arg,
        i2c_addr_arg,
        imu_node,
    ])
