#!/usr/bin/env python3
"""
Command Handlers for LLMy Robot
Handles ROS message processing and motor command conversion
"""

import math
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class CommandHandlers:
    """Handles ROS command message processing and motor control"""

    def __init__(self, node: Node, motor_manager, config):
        self.node = node
        self.motor_manager = motor_manager
        self.config = config

        # Command state tracking
        self.last_base_commands = []
        self.last_arm_commands = []
        self.last_camera_commands = []
    
    def handle_base_command(self, msg: Float64MultiArray):
        """Handle base/locomotion motor commands for 2-wheel differential drive"""

        if not self.config.loc_enable:
            return

        if len(msg.data) != len(self.config.loc_ids):
            self.node.get_logger().warn(f"Base command size mismatch: expected {len(self.config.loc_ids)}, got {len(msg.data)}")
            return

        # Check if commands have changed
        if (len(self.last_base_commands) == len(msg.data) and
            all(abs(a - b) < 1e-6 for a, b in zip(msg.data, self.last_base_commands))):
            return

        self.last_base_commands = list(msg.data)

        # MAPPING: Hardware interface sends commands in order of base_joints_ from ros2_control XACRO
        # XACRO order: right_wheel_joint, left_wheel_joint
        # Motor IDs: 1=right, 2=left
        # So: msg.data[0]->ID1 (right), msg.data[1]->ID2 (left)

        wheel_names = ["right", "left"]
        self.node.get_logger().debug(f"Base command received: {[f'{wheel_names[i]}={msg.data[i]:.3f}' for i in range(len(msg.data))]}")

        for i, motor_id in enumerate(self.config.loc_ids):
            velocity = msg.data[i]

            # Negate left wheel (index 1) for differential drive - wheels face opposite directions
            if i == 1:
                velocity = -velocity

            try:
                # Convert velocity (rad/s) to motor speed (steps/s)
                # Formula: steps/s = rad/s × (ticks_per_rev / 2π)
                rad_per_sec = velocity * self.config.loc_speed_scale
                steps_per_radian = self.config.ticks_per_rev / (2.0 * math.pi)
                speed_float = rad_per_sec * steps_per_radian
                speed_raw = int(max(-3400, min(3400, speed_float)))  # ST3215 max speed: 3400 steps/s

                # Log the motor command for debugging
                self.node.get_logger().debug(f"  Motor {motor_id} ({wheel_names[i]}): velocity={velocity:.3f} → speed_raw={speed_raw}")

                # Send command to motor
                self.motor_manager.send_velocity_command(motor_id, speed_raw)

            except Exception as e:
                self.node.get_logger().error(f"Failed to send velocity command to motor {motor_id}: {e}")
    
    def handle_arm_command(self, msg: Float64MultiArray):
        """Handle arm motor commands"""
        if not self.config.arm_enable:
            return

        if len(msg.data) != len(self.config.arm_ids):
            self.node.get_logger().warn(f"Arm command size mismatch: expected {len(self.config.arm_ids)}, got {len(msg.data)}")
            return

        # Check if commands have changed
        if (len(self.last_arm_commands) == len(msg.data) and
            all(abs(a - b) < 1e-6 for a, b in zip(msg.data, self.last_arm_commands))):
            return

        self.last_arm_commands = list(msg.data)

        # MAPPING: Hardware interface sends commands in order of joint names ["1","2","3","4","5","6"]
        # which maps directly to motor IDs [5,6,7,8,9,10] in order (msg.data[0] -> motor_id 5, etc.)
        # Arm joints: 5=base_rotation, 6=shoulder, 7=elbow, 8=wrist, 9=wrist_rotation, 10=gripper
        for i, motor_id in enumerate(self.config.arm_ids):
            try:
                # Convert from ROS radians to servo ticks
                angle_rad = msg.data[i]
                pos_ticks = self._radians_to_ticks(angle_rad)

                # Apply arm speed scaling
                scaled_speed = int(200 * self.config.arm_speed_scale)
                scaled_accel = int(200 * self.config.arm_speed_scale)

                self.motor_manager.send_position_command(motor_id, pos_ticks, scaled_speed, scaled_accel)

            except Exception as e:
                self.node.get_logger().error(f"Failed to send arm command to motor {motor_id}: {e}")
    
    def handle_camera_command(self, msg: Float64MultiArray):
        """Handle camera motor commands (tilt only)"""
        if not self.config.camera_enable:
            return

        if len(msg.data) != len(self.config.camera_ids):
            self.node.get_logger().warn(f"Camera command size mismatch: expected {len(self.config.camera_ids)}, got {len(msg.data)}")
            return

        # Check if commands have changed
        if (len(self.last_camera_commands) == len(msg.data) and
            all(abs(a - b) < 1e-6 for a, b in zip(msg.data, self.last_camera_commands))):
            return

        self.last_camera_commands = list(msg.data)

        # Camera motor: ID 11 = tilt
        for i, motor_id in enumerate(self.config.camera_ids):
            try:
                # Convert from ROS radians to servo ticks
                angle_rad = msg.data[i]
                pos_ticks = self._radians_to_ticks(angle_rad)

                # Apply camera speed scaling
                scaled_speed = int(100 * self.config.camera_speed_scale)
                scaled_accel = int(150 * self.config.camera_speed_scale)

                self.motor_manager.send_position_command(motor_id, pos_ticks, scaled_speed, scaled_accel)

            except Exception as e:
                self.node.get_logger().error(f"Failed to send camera command to motor {motor_id}: {e}")
    
    def _radians_to_ticks(self, angle_rad: float) -> int:
        """Convert radians to servo ticks"""
        servo_center = 2048
        pos_ticks = int(servo_center + (angle_rad / (2.0 * math.pi)) * self.config.ticks_per_rev)
        return max(0, min(4095, pos_ticks))
    
    def _ticks_to_radians(self, pos_ticks: int) -> float:
        """Convert servo ticks to radians"""
        servo_center = 2048
        return (pos_ticks - servo_center) / self.config.ticks_per_rev * 2.0 * math.pi