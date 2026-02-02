#!/usr/bin/env python3
"""
SLAM + Navigation Launch File for LLMy Robot

Launches SLAM Toolbox and Nav2 together for simultaneous mapping and navigation:
- slam_toolbox: Builds the map and provides map->odom transform
- planner_server: Global path planning (SmacPlanner2D)
- controller_server: Local control (MPPI)
- bt_navigator: Behavior tree execution
- behavior_server: Recovery behaviors
- velocity_smoother: Command smoothing
- lifecycle_manager: Manages all node lifecycles

No map_server or amcl needed — slam_toolbox handles both the map and localization.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('llmy_nav')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # Config file paths
    slam_config = os.path.join(pkg_dir, 'config', 'slam_toolbox.yaml')
    planner_config = os.path.join(pkg_dir, 'config', 'planner.yaml')
    controller_config = os.path.join(pkg_dir, 'config', 'controller.yaml')
    costmap_config = os.path.join(pkg_dir, 'config', 'costmap_mapfree.yaml')
    bt_config = os.path.join(pkg_dir, 'config', 'bt_navigator.yaml')
    behavior_config = os.path.join(pkg_dir, 'config', 'behavior.yaml')
    smoother_config = os.path.join(pkg_dir, 'config', 'velocity_smoother.yaml')

    # BT file paths
    bt_dir = os.path.join(pkg_dir, 'behavior_trees')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start lifecycle nodes'
        ),

        # SLAM Toolbox - provides map topic and map->odom transform
        LifecycleNode(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_config,
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

        # Lifecycle Manager - manages all nodes together
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_slam_nav',
                    output='screen',
                    parameters=[
                        {'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'bond_timeout': 20.0},
                        {'node_names': [
                            'slam_toolbox',
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
