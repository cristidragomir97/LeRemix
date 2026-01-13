from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_base_systems_arg = DeclareLaunchArgument(
        'use_base_systems',
        default_value='true',
        description='Enable base systems (servo manager)'
    )

    use_control_stack_arg = DeclareLaunchArgument(
        'use_control_stack',
        default_value='true',
        description='Enable control stack (controllers, robot state publisher)'
    )

    servo_port_arg = DeclareLaunchArgument(
        'servo_port',
        default_value='/dev/ttyACM0',
        description='Serial port for servo manager'
    )

    servo_baud_arg = DeclareLaunchArgument(
        'servo_baud',
        default_value='1000000',
        description='Baudrate for servo manager'
    )

    # Launch configurations
    use_base_systems = LaunchConfiguration('use_base_systems')
    use_control_stack = LaunchConfiguration('use_control_stack')
    servo_port = LaunchConfiguration('servo_port')
    servo_baud = LaunchConfiguration('servo_baud')

    # Include base systems (servo manager)
    servo_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('llmy_servo_manager'),
                'launch',
                'servo_manager.launch.py'
            ])
        ),
        launch_arguments={
            'servo_port': servo_port,
            'servo_baud': servo_baud
        }.items(),
        condition=IfCondition(use_base_systems)
    )

    # Include control stack (robot_state_publisher, controller_manager, spawners)
    control_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('llmy_control'),
                'launch',
                'control_stack.launch.py'
            ])
        ),
        condition=IfCondition(use_control_stack)
    )

    return LaunchDescription([
        # Launch arguments
        use_base_systems_arg,
        use_control_stack_arg,
        servo_port_arg,
        servo_baud_arg,

        # Launch includes and nodes
        servo_manager_launch,
        control_stack_launch,
    ])
