from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
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
    controllers_cfg = PathJoinSubstitution([
        FindPackageShare('leremix_control_plugin'),
        'config',
        'controllers.yaml'
    ])
    hw_cfg = PathJoinSubstitution([
        FindPackageShare('leremix_control_plugin'),
        'config',
        'ros2_control_esp32_bridge.yaml'
    ])
    
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

    # Robot description
    robot_description_content = Command([
        'xacro ', 
        PathJoinSubstitution([
            FindPackageShare('leremix_description'),
            'urdf',
            'LeRemix.xacro'
        ])
    ])

    # Robot state publisher - publishes robot_description topic
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # ros2_control node (controller_manager) - will use robot_description topic
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description_content}, controllers_cfg, hw_cfg]
    )

    # Controller spawners - delayed to wait for controller_manager
    spawner_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )
    
    spawner_omnidirectional_controller = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['omnidirectional_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )
    
    spawner_arm_controller = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['arm_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    # micro-ROS agent node
    microros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=[microros_transport, '--dev', microros_device, '-b', microros_baudrate],
        output='screen'
    )

    # rosboard node for web dashboard
    rosboard = Node(
        package='rosboard',
        executable='rosboard_node',
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
        robot_state_publisher,
        controller_manager,
        camera_launch,
        imu_launch,
        xbox_launch,
        spawner_joint_state_broadcaster,
        spawner_omnidirectional_controller,
        spawner_arm_controller,
        microros_agent,
        rosboard,
    ])