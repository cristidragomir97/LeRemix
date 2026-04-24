from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

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
    twist_mux_cfg = PathJoinSubstitution([
        FindPackageShare('llmy_control'),
        'config',
        'twist_mux.yaml'
    ])

    # Robot description (use_sim:=false to exclude Gazebo hardware)
    # Wrap with ParameterValue(value_type=str) so launch doesn't try to parse the URDF as YAML.
    robot_description_content = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                FindPackageShare('llmy_control'),
                'urdf',
                'LLMy.hardware.xacro'
            ]),
            ' use_sim:=false'
        ]),
        value_type=str
    )

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

    # Load arm_trajectory_controller in inactive state (for MoveIt)
    # Use: ros2 control switch_controller --activate arm_trajectory_controller --deactivate arm_controller
    spawner_arm_trajectory_controller = TimerAction(
        period=11.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['arm_trajectory_controller', '--inactive',
                           '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    spawner_head_controller = TimerAction(
        period=13.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['head_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    # Twist mux - multiplexes velocity commands from xbox, nav, mcp
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_cfg, {'use_sim_time': False}],
        remappings=[
            ('cmd_vel_out', '/diff_drive_controller/cmd_vel'),
        ]
    )

    return LaunchDescription([
        # Core control nodes
        robot_state_publisher,
        controller_manager,
        twist_mux,

        # Controller spawners
        spawner_joint_state_broadcaster,
        spawner_diff_drive_controller,
        spawner_arm_controller,
        spawner_arm_trajectory_controller,
        spawner_head_controller,
    ])
