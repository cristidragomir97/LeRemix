from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package share directories
    controllers_cfg = PathJoinSubstitution([
        FindPackageShare('llmy_control'),
        'config',
        'controllers.hw.yaml'
    ])
    hw_cfg = PathJoinSubstitution([
        FindPackageShare('llmy_control'),
        'config',
        'ros2_control_bridge.yaml'
    ])

    # Robot description
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('llmy_control'),
            'urdf',
            'LLMy.hardware.xacro'
        ])
    ])

    # Robot state publisher - publishes robot_description topic
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': False}
        ]
    )

    # ros2_control node (controller_manager) - will use robot_description topic
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': False},
            controllers_cfg,
            hw_cfg
        ]
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

    spawner_diff_drive_controller = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
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

    spawner_head_controller = TimerAction(
        period=11.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['head_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        # Core control nodes
        robot_state_publisher,
        controller_manager,

        # Controller spawners
        spawner_joint_state_broadcaster,
        spawner_diff_drive_controller,
        spawner_arm_controller,
        spawner_head_controller,
    ])
