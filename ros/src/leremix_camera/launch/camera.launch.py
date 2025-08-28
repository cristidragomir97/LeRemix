#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    camera_name = LaunchConfiguration('camera_name')
    device_type = LaunchConfiguration('device_type')
    enable_compressed = LaunchConfiguration('enable_compressed')
    enable_laser_scan = LaunchConfiguration('enable_laser_scan')
    laser_scan_min_height = LaunchConfiguration('laser_scan_min_height')
    laser_scan_max_height = LaunchConfiguration('laser_scan_max_height')
    laser_scan_angle_min = LaunchConfiguration('laser_scan_angle_min')
    laser_scan_angle_max = LaunchConfiguration('laser_scan_angle_max')
    laser_scan_angle_increment = LaunchConfiguration('laser_scan_angle_increment')
    laser_scan_range_min = LaunchConfiguration('laser_scan_range_min')
    laser_scan_range_max = LaunchConfiguration('laser_scan_range_max')


    # Declare launch arguments
    declare_camera_name = DeclareLaunchArgument(
        'camera_name',
        default_value='front_camera',
        description='RealSense camera name'
    )
    
    declare_device_type = DeclareLaunchArgument(
        'device_type',
        default_value='',
        description='RealSense device type (e.g., d435i, d455, l515)'
    )
    
    declare_enable_compressed = DeclareLaunchArgument(
        'enable_compressed',
        default_value='true',
        description='Enable compressed image transport'
    )
    
    declare_enable_laser_scan = DeclareLaunchArgument(
        'enable_laser_scan',
        default_value='true',
        description='Enable depth to laser scan conversion'
    )
    
    declare_laser_scan_min_height = DeclareLaunchArgument(
        'laser_scan_min_height',
        default_value='-0.5',
        description='Minimum height for laser scan slice'
    )
    
    declare_laser_scan_max_height = DeclareLaunchArgument(
        'laser_scan_max_height',
        default_value='0.5',
        description='Maximum height for laser scan slice'
    )
    
    declare_laser_scan_angle_min = DeclareLaunchArgument(
        'laser_scan_angle_min',
        default_value='-1.57',  # -90 degrees
        description='Minimum angle for laser scan'
    )
    
    declare_laser_scan_angle_max = DeclareLaunchArgument(
        'laser_scan_angle_max',
        default_value='1.57',   # +90 degrees
        description='Maximum angle for laser scan'
    )
    
    declare_laser_scan_angle_increment = DeclareLaunchArgument(
        'laser_scan_angle_increment',
        default_value='0.005',
        description='Angular resolution of laser scan'
    )
    
    declare_laser_scan_range_min = DeclareLaunchArgument(
        'laser_scan_range_min',
        default_value='0.2',
        description='Minimum range for laser scan'
    )
    
    declare_laser_scan_range_max = DeclareLaunchArgument(
        'laser_scan_range_max',
        default_value='10.0',
        description='Maximum range for laser scan'
    )

    # RealSense camera launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ),
        launch_arguments={
            'camera_namespace': '', 
            'camera_name': camera_name,
            'device_type': device_type,
            'enable_depth': 'true',
            'enable_color': 'true',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30',
            'align_depth.enable': 'true',
            'pointcloud.enable': 'false',  # Disable to save bandwidth
            'enable_sync': 'true',
        }.items()
    )

    # Compressed image transport nodes
    # Color image compression
    compressed_color_node = Node(
        package='image_transport',
        executable='republish',
        name='compressed_color_republisher',
        arguments=[
            'raw', 'compressed',
            '--ros-args',
            '--remap', 'in:=front_camera/color/image_raw',
            '--remap', 'out:=front_camera/color/image_compressed',
        ],
        condition=IfCondition(enable_compressed)
    )
    

    # Depth image to laser scan conversion
    depthimage_to_laserscan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        parameters=[{
            'scan_height': 10,
            'scan_time': 0.020,  # 30Hz
            'range_min': laser_scan_range_min,
            'range_max': laser_scan_range_max,
            'output_frame_id': 'lidar',
        }],
        remappings=[
            ('depth', 'front_camera/depth/image_rect_raw'),
            ('depth_camera_info', 'front_camera/depth/camera_info'),
            ('scan', '/scan'),
        ],
        condition=IfCondition(enable_laser_scan)
    )


    return LaunchDescription([
        # Launch arguments
        declare_camera_name,
        declare_device_type,
        declare_enable_compressed,
        declare_enable_laser_scan,
        declare_laser_scan_min_height,
        declare_laser_scan_max_height,
        declare_laser_scan_angle_min,
        declare_laser_scan_angle_max,
        declare_laser_scan_angle_increment,
        declare_laser_scan_range_min,
        declare_laser_scan_range_max,
        
        # Nodes
        realsense_launch,
        compressed_color_node,
        depthimage_to_laserscan_node,
    ])