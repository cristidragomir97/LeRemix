#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Lifelong Mapping Launch - Dynamic mode switching

    Allows switching between mapping and localization using services.
    Can load existing maps and continue mapping across sessions.

    Automatically activates slam_toolbox lifecycle node.

    Topics:
    - Subscribes: /scan (from RPLidar)
    - Publishes: /map, /map_metadata

    Services:
    - /slam_toolbox/pause_new_measurements - Pause/resume mapping
    - /slam_toolbox/serialize_map - Save map
    - /slam_toolbox/deserialize_map - Load map

    Frames:
    - base_footprint (ground level, parent of base_link)
    - base_link (robot center)
    - lidar_frame (from URDF)
    - laser (rplidar frame_id)
    """

    # Config files
    pkg_dir = get_package_share_directory('llmy_slam')
    config_file = os.path.join(pkg_dir, 'config', 'lifelong_mapping.yaml')
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf.yaml')

    # EKF node for sensor fusion (wheel odom + IMU)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': True}
        ],
        remappings=[
            ('odometry/filtered', '/odometry/filtered')
        ]
    )

    # Static TF: lidar_frame -> laser (rplidar uses "laser" frame_id)
    # Note: base_footprint is already defined in URDF as child of base_link
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_frame_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'lidar_frame', '--child-frame-id', 'laser'],
        parameters=[{'use_sim_time': False}]
    )

    # SLAM Toolbox (lifecycle node) - using lifelong/async node
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='lifelong_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': False}
        ]
    )

    # Auto-configure slam_toolbox on startup
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # Auto-activate slam_toolbox after configuration
    activate_event = RegisterEventHandler(
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
        ekf_node,
        laser_tf,
        slam_node,
        configure_event,
        activate_event,
    ])
