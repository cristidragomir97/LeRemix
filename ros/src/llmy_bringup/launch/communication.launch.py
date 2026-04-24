from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    use_rosbridge_arg = DeclareLaunchArgument(
        'use_rosbridge',
        default_value='true',
        description='Enable rosbridge server for web interfaces'
    )
    
    use_rosboard_arg = DeclareLaunchArgument(
        'use_rosboard',
        default_value='true',
        description='Enable rosboard web dashboard'
    )

    use_image_compression_arg = DeclareLaunchArgument(
        'use_image_compression',
        default_value='true',
        description='Enable image compression nodes'
    )

    # Launch configurations
    use_rosbridge = LaunchConfiguration('use_rosbridge')
    use_rosboard = LaunchConfiguration('use_rosboard')
    use_image_compression = LaunchConfiguration('use_image_compression')

    # Rosbridge server for web interfaces
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        condition=IfCondition(use_rosbridge)
    )

    # Rosboard web dashboard
    rosboard_node = Node(
        package='rosboard',
        executable='rosboard_node',
        name='rosboard_node',
        output='screen',
        condition=IfCondition(use_rosboard)
    )

    # Compressed image transport nodes
    # Color image compression for head camera
    compressed_color_node = Node(
        package='image_transport',
        executable='republish',
        name='compressed_color_republisher',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', 'head_camera/head_camera/color/image_raw'),
            ('out', 'head_camera/head_camera/color/image_compressed'),
        ],
        condition=IfCondition(use_image_compression),
        output='screen'
    )

    # Compressed image transport for wrist camera
    compressed_wrist_node = Node(
        package='image_transport',
        executable='republish',
        name='compressed_wrist_republisher',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', 'wrist_camera/image_raw'),
            ('out', 'wrist_camera/image_compressed'),
        ],
        condition=IfCondition(use_image_compression),
        output='screen'
    )

    # Depth image to laser scan conversion
    depthimage_to_laserscan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        parameters=[{
            'scan_height': 10,
            'scan_time': 0.020,  # 30Hz
            'range_min': 0.2,
            'range_max': 10.0,
            'output_frame_id': 'lidar',
        }],
        remappings=[
            ('depth', 'head_camera/head_camera/depth/image_rect_raw'),
            ('depth_camera_info', 'head_camera/head_camera/depth/camera_info'),
            ('scan', '/scan'),
        ],
        condition=IfCondition(use_image_compression),
        output='screen'
    )

    return LaunchDescription([
        # Launch arguments
        use_rosbridge_arg,
        use_rosboard_arg,
        use_image_compression_arg,

        # Nodes
        rosbridge_node,
        rosboard_node,
        compressed_color_node,
        compressed_wrist_node,
        depthimage_to_laserscan_node,
    ])