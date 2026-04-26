#!/usr/bin/env python3
"""Generic command handler for a motor group."""

import math
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .motor_group import MotorGroup


ST3215_MAX_SPEED = 3400  # steps/s hardware limit


class GroupCommandHandler:
    """Handles ROS command messages for a single motor group.

    One instance is created per enabled group, each with its own subscriber.
    Uses command_indices to pick values from the incoming Float64MultiArray,
    so multiple groups can share a topic or the array can contain joints
    managed by other packages.
    """

    def __init__(self, node: Node, motor_manager, group: MotorGroup,
                 ticks_per_rev: int):
        self.node = node
        self.motor_manager = motor_manager
        self.group = group
        self.ticks_per_rev = ticks_per_rev
        self.servo_center = ticks_per_rev // 2
        self._last_commands: list[float] = []
        self._max_cmd_index = max(group.command_indices) if group.command_indices else 0

    def handle_command(self, msg: Float64MultiArray):
        if not self.group.enable:
            return

        if len(msg.data) <= self._max_cmd_index:
            self.node.get_logger().warn(
                f"[{self.group.name}] Message too short: need index "
                f"{self._max_cmd_index}, got length {len(msg.data)}")
            return

        # Extract values at configured indices
        values = [msg.data[ci] for ci in self.group.command_indices]

        # Skip if commands haven't changed
        if (len(self._last_commands) == len(values)
                and all(abs(a - b) < 1e-6
                        for a, b in zip(values, self._last_commands))):
            return

        self._last_commands = list(values)

        for i, motor_id in enumerate(self.group.motor_ids):
            value = values[i] * self.group.direction[i]

            try:
                if self.group.mode == "velocity":
                    self._send_velocity(motor_id, value)
                else:
                    self._send_position(motor_id, value)
            except Exception as e:
                self.node.get_logger().error(
                    f"[{self.group.name}] Command failed for motor {motor_id}: {e}")

    def _send_velocity(self, motor_id: int, rad_per_sec: float):
        """Convert rad/s to motor steps/s and send."""
        scaled = rad_per_sec * self.group.speed_scale
        steps_per_radian = self.ticks_per_rev / (2.0 * math.pi)
        speed_raw = int(max(-ST3215_MAX_SPEED,
                            min(ST3215_MAX_SPEED,
                                scaled * steps_per_radian)))
        self.motor_manager.send_velocity_command(motor_id, speed_raw)

    def _send_position(self, motor_id: int, angle_rad: float):
        """Convert radians to servo ticks and send."""
        pos_ticks = int(self.servo_center
                        + (angle_rad / (2.0 * math.pi)) * self.ticks_per_rev)
        pos_ticks = max(0, min(self.ticks_per_rev - 1, pos_ticks))

        speed = int(self.group.position_speed * self.group.speed_scale)
        accel = int(self.group.position_accel * self.group.speed_scale)
        self.motor_manager.send_position_command(motor_id, pos_ticks, speed, accel)
