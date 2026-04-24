from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_sensors_arg = DeclareLaunchArgument(
        'use_sensors',
        default_value='true',
        description='Enable sensors (cameras, IMU, RPLidar)'
    )
    
    use_motion_arg = DeclareLaunchArgument(
        'use_motion',
        default_value='true',
        description='Enable motion systems (servos, ros2_control, teleop)'
    )
    
    use_communication_arg = DeclareLaunchArgument(
        'use_communication',
        default_value='true',
        description='Enable communication (rosbridge, rosboard, image compression)'
    )

    # Launch configurations
    use_sensors = LaunchConfiguration('use_sensors')
    use_motion = LaunchConfiguration('use_motion')
    use_communication = LaunchConfiguration('use_communication')

    # Include sensors launch file
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('llmy_bringup'),
                'launch',
                'sensors.launch.py'
            ])
        ),
        condition=IfCondition(use_sensors)
    )

    # Include motion launch file
    motion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('llmy_control'),
                'launch',
                'motion.launch.py'
            ])
        ),
        condition=IfCondition(use_motion)
    )

    # Include communication launch file
    communication_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('llmy_bringup'),
                'launch',
                'communication.launch.py'
            ])
        ),
        condition=IfCondition(use_communication)
    )

    return LaunchDescription([
        # Launch arguments
        use_sensors_arg,
        use_motion_arg,
        use_communication_arg,

        # Launch includes
        sensors_launch,
        motion_launch,
        communication_launch,
    ])