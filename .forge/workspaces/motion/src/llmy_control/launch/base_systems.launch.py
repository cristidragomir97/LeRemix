from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments for servo manager
    servo_port_arg = DeclareLaunchArgument(
        'servo_port',
        default_value='/dev/ttyTHS1',
        description='Serial port for servo manager'
    )

    servo_baud_arg = DeclareLaunchArgument(
        'servo_baud',
        default_value='1000000',
        description='Baudrate for servo manager'
    )

    servo_config_arg = DeclareLaunchArgument(
        'servo_config',
        default_value='',
        description='Path to servo manager config file (uses package default if empty)'
    )

    # Launch configurations
    servo_port = LaunchConfiguration('servo_port')
    servo_baud = LaunchConfiguration('servo_baud')
    servo_config = LaunchConfiguration('servo_config')

    # Include servo manager launch
    servo_manager_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('llmy_servo_manager'),
            'launch',
            'servo_manager.launch.py'
        ])
    )

    return LaunchDescription([
        # Launch arguments
        servo_port_arg,
        servo_baud_arg,
        servo_config_arg,

        # Base system nodes
        servo_manager_launch,
    ])
