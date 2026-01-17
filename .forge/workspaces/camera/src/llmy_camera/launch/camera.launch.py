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
    wrist_camera_device = LaunchConfiguration('wrist_camera_device')
    enable_wrist_camera = LaunchConfiguration('enable_wrist_camera')
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
        default_value='head_camera',
        description='RealSense camera name'
    )
    
    declare_wrist_camera_device = DeclareLaunchArgument(
        'wrist_camera_device',
        default_value='/dev/video0',
        description='Wrist camera device path'
    )
    
    declare_enable_wrist_camera = DeclareLaunchArgument(
        'enable_wrist_camera',
        default_value='true',
        description='Enable wrist camera'
    )
    
    declare_device_type = DeclareLaunchArgument(
        'device_type',
        default_value='',
        description='RealSense device type (e.g., d435i, d455, l515)'
    )
    
    declare_enable_compressed = DeclareLaunchArgument(
        'enable_compressed',
        default_value='false',
        description='Enable compressed image transport'
    )
    
    declare_enable_laser_scan = DeclareLaunchArgument(
        'enable_laser_scan',
        default_value='false',
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
        launch_arguments=[
            ('camera_namespace', 'head_camera'),
            ('camera_name', camera_name),
            ('device_type', device_type),
            ('enable_depth', 'true'),
            ('enable_color', 'true'),
            ('enable_infra1', 'false'),
            ('enable_infra2', 'false'),
            ('depth_module.profile', '640x480x30'),
            ('rgb_camera.profile', '640x480x30'),
            ('align_depth.enable', 'true'),
            ('pointcloud.enable', 'false'),  # Disable to save bandwidth
            ('enable_sync', 'true'),
        ]
    )


    # Wrist camera using usb_cam
    wrist_camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='wrist_camera',
        namespace='wrist_camera',
        parameters=[{
            'video_device': wrist_camera_device,
            'camera_frame_id': 'wrist_camera_frame',
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'mjpeg2rgb',
            'framerate': 30.0,
            'camera_name': 'wrist_camera',
            'io_method': 'mmap',
        }],
        condition=IfCondition(enable_wrist_camera)
    )


    return LaunchDescription([
        # Launch arguments
        declare_camera_name,
        declare_wrist_camera_device,
        declare_enable_wrist_camera,
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
        wrist_camera_node,
    ])
