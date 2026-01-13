#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='200.0',
        description='Publishing rate for IMU data in Hz'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='Frame ID for IMU data'
    )
    
    use_mag_arg = DeclareLaunchArgument(
        'use_mag',
        default_value='true',
        description='Use magnetometer for orientation filtering'
    )
    
    # Create the IMU node
    imu_node = Node(
        package='llmy_imu',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{
            'publish_rate': LaunchConfiguration('publish_rate'),
            'frame_id': LaunchConfiguration('frame_id'),
        }]
    )
    
    # Create the Madgwick filter node for orientation estimation
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
            ('imu/data', 'imu/fused'),
        ]
    )
    
    return LaunchDescription([
        publish_rate_arg,
        frame_id_arg,
        use_mag_arg,
        imu_node,
        imu_filter_node
    ])