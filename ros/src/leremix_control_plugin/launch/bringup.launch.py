from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('leremix_control_plugin')
    controllers_cfg = os.path.join(pkg_share, 'config', 'controllers.yaml')
    hw_cfg = os.path.join(pkg_share, 'config', 'ros2_control_esp32_bridge.yaml')

    # controller_manager with both configs
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[controllers_cfg, hw_cfg]
    )

    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    spawner_base = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['omnidirectional_controller'],
        output='screen'
    )
    spawner_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen'
    )

    return LaunchDescription([
        controller_manager,
        spawner_jsb,
        spawner_base,
        spawner_arm,
    ])
