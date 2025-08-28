from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Enable camera (leremix_camera)'
    )
    
    use_imu_arg = DeclareLaunchArgument(
        'use_imu',
        default_value='true',
        description='Enable IMU (leremix_imu)'
    )
    
    use_xbox_arg = DeclareLaunchArgument(
        'use_xbox',
        default_value='true',
        description='Enable Xbox controller (leremix_teleop_xbox)'
    )
    
    microros_transport_arg = DeclareLaunchArgument(
        'microros_transport',
        default_value='serial',
        description='Transport for micro-ROS agent'
    )
    
    microros_device_arg = DeclareLaunchArgument(
        'microros_device',
        default_value='/dev/ttyUSB0',
        description='Device for micro-ROS agent'
    )
    
    microros_baudrate_arg = DeclareLaunchArgument(
        'microros_baudrate',
        default_value='115200',
        description='Baudrate for micro-ROS agent'
    )

    # Launch configurations
    use_camera = LaunchConfiguration('use_camera')
    use_imu = LaunchConfiguration('use_imu')
    use_xbox = LaunchConfiguration('use_xbox')
    microros_transport = LaunchConfiguration('microros_transport')
    microros_device = LaunchConfiguration('microros_device')
    microros_baudrate = LaunchConfiguration('microros_baudrate')

    # Get package share directories
    control_plugin_pkg_share = get_package_share_directory('leremix_control_plugin')
    controllers_cfg = os.path.join(control_plugin_pkg_share, 'config', 'controllers.yaml')
    hw_cfg = os.path.join(control_plugin_pkg_share, 'config', 'ros2_control_esp32_bridge.yaml')

    # Include leremix_camera launch file
    camera_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('leremix_camera'),
            'launch',
            'camera.launch.py'
        ]),
        condition=IfCondition(use_camera)
    )

    # Include leremix_imu launch file
    imu_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('leremix_imu'),
            'launch',
            'imu.launch.py'
        ]),
        condition=IfCondition(use_imu)
    )

    # Include leremix_teleop_xbox launch file
    xbox_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('leremix_teleop_xbox'),
            'launch',
            'teleop_xbox.launch.py'
        ]),
        condition=IfCondition(use_xbox)
    )

    # ros2_control node (controller_manager)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[controllers_cfg, hw_cfg]
    )

    # Controller spawners
    spawner_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    spawner_omnidirectional_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['omnidirectional_controller'],
        output='screen'
    )
    
    spawner_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen'
    )

    # micro-ROS agent node
    microros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=[microros_transport, '--dev', microros_device, '-b', microros_baudrate],
        output='screen'
    )

    return LaunchDescription([
        # Launch arguments
        use_camera_arg,
        use_imu_arg,
        use_xbox_arg,
        microros_transport_arg,
        microros_device_arg,
        microros_baudrate_arg,
        
        # Launch includes and nodes
        camera_launch,
        imu_launch,
        xbox_launch,
        controller_manager,
        spawner_joint_state_broadcaster,
        spawner_omnidirectional_controller,
        spawner_arm_controller,
        microros_agent,
    ])