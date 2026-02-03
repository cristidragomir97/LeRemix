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
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('llmy_nav')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    explore = LaunchConfiguration('explore')

    # Config file paths
    explore_config = os.path.join(pkg_dir, 'config', 'explore.yaml')
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf.sim.yaml')
    slam_config = os.path.join(pkg_dir, 'config', 'online_mapping.yaml')
    planner_config = os.path.join(pkg_dir, 'config', 'planner.yaml')
    controller_config = os.path.join(pkg_dir, 'config', 'controller.yaml')
    costmap_config = os.path.join(pkg_dir, 'config', 'costmap_mapfree.yaml')
    bt_config = os.path.join(pkg_dir, 'config', 'bt_navigator.yaml')
    behavior_config = os.path.join(pkg_dir, 'config', 'behavior.yaml')
    smoother_config = os.path.join(pkg_dir, 'config', 'velocity_smoother.yaml')

    # BT file paths
    bt_dir = os.path.join(pkg_dir, 'behavior_trees')

    # SLAM Toolbox lifecycle node (self-managed, not via lifecycle_manager)
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': use_sim_time},
        ]
    )

    # Auto-configure slam_toolbox on startup
    configure_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # Auto-activate slam_toolbox after configuration
    activate_slam = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

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
        DeclareLaunchArgument(
            'explore',
            default_value='false',
            description='Enable autonomous frontier exploration'
        ),

        # EKF node for sensor fusion (wheel odom + IMU)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config,
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('odometry/filtered', '/odometry/filtered'),
            ]
        ),

        # SLAM Toolbox - self-managed lifecycle (configure + activate)
        slam_node,
        configure_slam,
        activate_slam,

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
        # Delayed to allow EKF to start publishing odom->base_footprint TF
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_slam_nav',
                    output='screen',
                    parameters=[
                        {'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'bond_timeout': 30.0},
                        {'node_names': [
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

        # Autonomous frontier exploration (optional)
        TimerAction(
            period=20.0,
            actions=[
                Node(
                    condition=IfCondition(explore),
                    package='explore_lite',
                    executable='explore',
                    name='explore_node',
                    output='screen',
                    parameters=[
                        explore_config,
                        {'use_sim_time': use_sim_time},
                    ],
                ),
            ]
        ),
    ])
