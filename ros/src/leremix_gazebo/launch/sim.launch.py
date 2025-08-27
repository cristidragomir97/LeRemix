from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gui        = LaunchConfiguration("gui")
    pause      = LaunchConfiguration("pause")
    spawn_z    = LaunchConfiguration("spawn_z")
    cm_timeout = LaunchConfiguration("cm_timeout")

    declare_gui        = DeclareLaunchArgument("gui",        default_value="true")
    declare_pause      = DeclareLaunchArgument("pause",      default_value="false")
    declare_spawn_z    = DeclareLaunchArgument("spawn_z",    default_value="0.05")
    declare_cm_timeout = DeclareLaunchArgument("cm_timeout", default_value="60")

    # Use the overlay (includes base xacro + gazebo overlay + ros2_control sim backend)
    xacro_file = PathJoinSubstitution([FindPackageShare("leremix_gazebo"), "urdf", "leremix_gazebo_overlay.xacro"])
    robot_description = {"robot_description": Command([FindExecutable(name="xacro"), " ", xacro_file])}

    world = PathJoinSubstitution([
        FindPackageShare('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_house.world'
    ])

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[robot_description], output="screen"
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gzserver.launch.py"
            ])
        ),
        launch_arguments={
            "world": world,
            "pause": pause,
            "verbose": "true"
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gzclient.launch.py"])),
        condition=IfCondition(gui)
    )

    spawn = Node(
        package="gazebo_ros", executable="spawn_entity.py", name="spawn_entity",
        arguments=["-entity", "LeRemix", "-topic", "robot_description", "-z", spawn_z, '-x', '0.2', '-y', '0.2'],
        output="screen"
    )

    # Spawners — make sure names match your YAML (leremix_control/config/*.yaml)
    jsb = Node(
        package="controller_manager", executable="spawner", name="spawner_joint_state_broadcaster",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", cm_timeout],
        output="screen",
    )

    omni_controller = Node(
        package="controller_manager", executable="spawner", name="spawner_base_controller",
        arguments=["omnidirectional_controller", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", cm_timeout],
        output="screen",
    )

    arm_controller = Node(
        package="controller_manager", executable="spawner", name="spawner_base_controller",
        arguments=["arm_controller", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", cm_timeout],
        output="screen",
    )



    # Chain: spawn entity → JS broadcaster → base & arm
    after_spawn = RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb]))
    after_js    = RegisterEventHandler(OnProcessExit(target_action=jsb,   on_exit=[omni_controller, arm_controller]))

    return LaunchDescription([
        declare_gui, declare_pause, declare_spawn_z, declare_cm_timeout,
        rsp, gzserver, gzclient, spawn, after_spawn, after_js
    ])
