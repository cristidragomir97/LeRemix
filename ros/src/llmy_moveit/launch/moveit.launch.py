#!/usr/bin/env python3
"""
MoveIt 2 launch file for LLMy robot arm.

This launch file starts the move_group node and optionally RViz.
It expects the control stack (controller_manager, robot_state_publisher)
to already be running. The arm_trajectory_controller must be loaded
(it will be activated by MoveIt on demand).

Usage:
  # Hardware (default):
  ros2 launch llmy_moveit moveit.launch.py

  # Simulation:
  ros2 launch llmy_moveit moveit.launch.py use_sim_time:=true

  # Without RViz:
  ros2 launch llmy_moveit moveit.launch.py launch_rviz:=false
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, file_path):
    """Load a yaml file from a ROS package."""
    full_path = os.path.join(get_package_share_directory(package_name), file_path)
    with open(full_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')

    # Package paths
    pkg_moveit = get_package_share_directory('llmy_moveit')
    pkg_description = get_package_share_directory('llmy_description')

    # Robot description - use the base URDF (without sim hardware)
    robot_description_content = Command([
        'xacro ',
        os.path.join(pkg_description, 'urdf', 'LLMy.xacro'),
        ' use_sim:=false'
    ])

    robot_description = {'robot_description': robot_description_content}

    # SRDF
    srdf_path = os.path.join(pkg_moveit, 'config', 'llmy.srdf')
    with open(srdf_path, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    # Load config files
    kinematics_yaml = load_yaml('llmy_moveit', 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml('llmy_moveit', 'config/joint_limits.yaml')
    ompl_planning_yaml = load_yaml('llmy_moveit', 'config/ompl_planning.yaml')
    moveit_controllers_yaml = load_yaml('llmy_moveit', 'config/moveit_controllers.yaml')

    # MoveIt configuration
    moveit_config = {
        'robot_description_kinematics': kinematics_yaml,
        'robot_description_planning': {'joint_limits': joint_limits_yaml},
        'planning_pipelines': {
            'pipeline_names': ['ompl'],
        },
        'ompl': ompl_planning_yaml,
        'moveit_simple_controller_manager': moveit_controllers_yaml,
    }

    # Move Group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            moveit_config,
            {'use_sim_time': use_sim_time},
            {'trajectory_execution.allowed_execution_duration_scaling': 2.0},
            {'trajectory_execution.allowed_goal_duration_margin': 1.0},
            {'trajectory_execution.allowed_start_tolerance': 0.05},
            {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'},
        ],
    )

    # RViz with MoveIt plugin
    rviz_config = os.path.join(pkg_moveit, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[
            robot_description,
            robot_description_semantic,
            moveit_config,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        move_group_node,
        rviz_node,
    ])
