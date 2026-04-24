#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    camera_name = LaunchConfiguration('camera_name')
    device_type = LaunchConfiguration('device_type')

    # Declare launch arguments
    declare_camera_name = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='RealSense camera name'
    )
    
    declare_device_type = DeclareLaunchArgument(
        'device_type',
        default_value='d435i',
        description='RealSense device type'
    )

    # RealSense camera launch (minimal configuration)
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ),
        launch_arguments=[
            ('camera_name', camera_name),
            ('device_type', device_type),
            ('enable_depth', 'true'),
            ('enable_color', 'true'),
            ('enable_infra1', 'false'),
            ('enable_infra2', 'false'),
            ('depth_module.profile', '640x480x30'),
            ('rgb_camera.profile', '640x480x30'),
            ('align_depth.enable', 'true'),
            ('pointcloud.enable', 'false'),
            ('enable_sync', 'true'),
        ]
    )

    return LaunchDescription([
        declare_camera_name,
        declare_device_type,
        realsense_launch,
    ])