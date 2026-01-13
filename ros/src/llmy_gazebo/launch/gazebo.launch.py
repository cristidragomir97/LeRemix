#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    # Package directories
    pkg_llmy_description = get_package_share_directory('llmy_description')
    pkg_llmy_gazebo = get_package_share_directory('llmy_gazebo')
    pkg_llmy_control = get_package_share_directory('llmy_control')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Get the install prefix to set up Gazebo resource path
    install_prefix = get_package_prefix('llmy_description')

    # Set GZ_SIM_RESOURCE_PATH to include the ROS 2 install directory
    # This allows Gazebo to find package:// URIs
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(install_prefix, 'share')
    )

    # Paths
    world_file = os.path.join(pkg_llmy_gazebo, 'worlds', 'sticky_floor.world')
    urdf_file = os.path.join(pkg_llmy_description, 'urdf', 'LLMy.xacro')
    controllers_file = os.path.join(pkg_llmy_control, 'config', 'controllers.sim.yaml')
    bridge_config = os.path.join(pkg_llmy_gazebo, 'config', 'gz_ros_bridge.yaml')

    # Process the URDF
    robot_description_content = xacro.process_file(urdf_file).toxml()

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    # Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )

    # Clock Bridge (Gazebo -> ROS)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # ROS-Gazebo Bridge for sensors
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '--ros-args',
            '-p', ['config_file:=', bridge_config]
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Image bridges for cameras
    image_bridge_head_rgb = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='image_bridge_head_rgb',
        arguments=['/head_camera/rgb'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    image_bridge_head_depth = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='image_bridge_head_depth',
        arguments=['/head_camera/depth'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    image_bridge_wrist = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='image_bridge_wrist',
        arguments=['/wrist_camera/rgb'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'llmy',
            '-z', '0.2'
        ],
        output='screen'
    )

    # Joint State Broadcaster
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # Differential Drive Controller
    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_controller'],
        output='screen'
    )

    # Arm Controller
    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen'
    )

    # Head Controller
    load_head_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'head_controller'],
        output='screen'
    )

    # Event handlers to load controllers after robot is spawned
    load_joint_state_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    load_diff_drive_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_diff_drive_controller],
        )
    )

    load_arm_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_diff_drive_controller,
            on_exit=[load_arm_controller],
        )
    )

    load_head_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_arm_controller,
            on_exit=[load_head_controller],
        )
    )

    return LaunchDescription([
        gz_resource_path,
        robot_state_publisher,
        gz_sim,
        clock_bridge,
        bridge,
        image_bridge_head_rgb,
        image_bridge_head_depth,
        image_bridge_wrist,
        spawn_robot,
        load_joint_state_broadcaster_event,
        load_diff_drive_controller_event,
        load_arm_controller_event,
        load_head_controller_event,
    ])
