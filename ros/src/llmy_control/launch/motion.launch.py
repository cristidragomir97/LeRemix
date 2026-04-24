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

    use_st3215_arg = DeclareLaunchArgument(
        'use_st3215',
        default_value='true',
        description='Enable ST3215 servo manager (arm/camera)')

    use_ddsm210_arg = DeclareLaunchArgument(
        'use_ddsm210',
        default_value='true',
        description='Enable DDSM210 motor manager (wheels)')

    # Launch configurations
    use_base_systems = LaunchConfiguration('use_base_systems')
    use_control_stack = LaunchConfiguration('use_control_stack')
    use_st3215 = LaunchConfiguration('use_st3215')
    use_ddsm210 = LaunchConfiguration('use_ddsm210')

    # ST3215 servo manager (arm + camera motors)
    st3215_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('st3215_manager'),
                'launch',
                'servo_manager.launch.py'
            ])
        ),
        condition=IfCondition(use_st3215)
    )

    # DDSM210 motor manager (wheel motors)
    ddsm210_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ddsm210_manager'),
                'launch',
                'ddsm210.launch.py'
            ])
        ),
        condition=IfCondition(use_ddsm210)
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
        use_st3215_arg,
        use_ddsm210_arg,

        # Motor managers
        st3215_launch,
        ddsm210_launch,

        # Control stack (robot_state_publisher, controllers)
        control_stack_launch,
    ])
