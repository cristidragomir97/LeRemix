#!/usr/bin/env python3
"""
Navigation Launch File for LLMy Robot

Launches the full Nav2 stack for map-based autonomous navigation:
- map_server: Serves the static map
- amcl: Particle filter localization
- planner_server: Global path planning (SmacPlanner2D)
- controller_server: Local control (MPPI)
- bt_navigator: Behavior tree execution
- behavior_server: Recovery behaviors
- velocity_smoother: Command smoothing
- lifecycle_manager: Manages node lifecycle
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('llmy_nav')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')

    # Config file paths
    planner_config = os.path.join(pkg_dir, 'config', 'planner.yaml')
    controller_config = os.path.join(pkg_dir, 'config', 'controller.yaml')
    costmap_config = os.path.join(pkg_dir, 'config', 'costmap.yaml')
    amcl_config = os.path.join(pkg_dir, 'config', 'amcl.yaml')
    bt_config = os.path.join(pkg_dir, 'config', 'bt_navigator.yaml')
    behavior_config = os.path.join(pkg_dir, 'config', 'behavior.yaml')
    smoother_config = os.path.join(pkg_dir, 'config', 'velocity_smoother.yaml')

    # BT file paths
    bt_dir = os.path.join(pkg_dir, 'behavior_trees')

    # Default map path
    default_map = os.path.join(os.path.expanduser('~'), '.llmy', 'maps', 'home.yaml')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Path to map YAML file'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start lifecycle nodes'
        ),

        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'yaml_filename': map_yaml},
                {'topic_name': 'map'},
                {'frame_id': 'map'},
            ]
        ),

        # AMCL - Localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                amcl_config,
                {'use_sim_time': use_sim_time},
            ]
        ),

        # Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[
                planner_config,
                costmap_config,
                {'use_sim_time': use_sim_time},
            ]
        ),

        # Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[
                controller_config,
                costmap_config,
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('cmd_vel', 'cmd_vel_raw'),
            ]
        ),

        # Behavior Tree Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                bt_config,
                {'use_sim_time': use_sim_time},
                {'default_nav_to_pose_bt_xml': os.path.join(bt_dir, 'nav_to_pose.xml')},
                {'default_nav_through_poses_bt_xml': os.path.join(bt_dir, 'nav_through_poses.xml')},
            ]
        ),

        # Behavior Server (recovery behaviors)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[
                behavior_config,
                costmap_config,
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('cmd_vel', 'cmd_vel_raw'),
            ]
        ),

        # Velocity Smoother
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[
                smoother_config,
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('cmd_vel', 'cmd_vel_raw'),
                ('cmd_vel_smoothed', 'cmd_vel_nav'),
            ]
        ),

        # Lifecycle Manager - delayed start to allow nodes to register
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[
                        {'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'bond_timeout': 20.0},
                        {'node_names': [
                            'map_server',
                            'amcl',
                            'planner_server',
                            'controller_server',
                            'bt_navigator',
                            'behavior_server',
                            'velocity_smoother',
                        ]},
                    ]
                ),
            ]
        ),
    ])
