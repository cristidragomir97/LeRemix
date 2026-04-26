#!/usr/bin/env python3
"""Joint state telemetry publisher — uses state_indices for fixed-position output."""

import math
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .motor_group import MotorGroup


class TelemetrySystem:
    """Publishes JointState messages with motors placed at configured indices.

    Each motor's state_index determines its position in the JointState arrays,
    allowing multiple packages to publish to the same topic with non-overlapping
    indices.
    """

    def __init__(self, node: Node, motor_manager, groups: list[MotorGroup],
                 ticks_per_rev: int, joint_states_topic: str, total_joints: int):
        self.node = node
        self.motor_manager = motor_manager
        self.ticks_per_rev = ticks_per_rev
        self.servo_center = ticks_per_rev // 2
        self.total_joints = total_joints

        # Build per-motor info: (motor_id, joint_name, direction, state_index, unwrap)
        self._motors: list[tuple[int, str, int, int, bool]] = []
        for group in groups:
            if not group.enable:
                continue
            for i, mid in enumerate(group.motor_ids):
                self._motors.append((
                    mid,
                    group.joint_names[i],
                    group.direction[i],
                    group.state_indices[i],
                    group.unwrap_position,
                ))

        # Position unwrapping state
        self._last_raw_pos: dict[int, int] = {}
        self._revolution_count: dict[int, int] = {}

        # Publisher
        self.state_pub = node.create_publisher(JointState, joint_states_topic, 10)
        node.get_logger().info(f"  Joint state publisher: {joint_states_topic} "
                               f"(total_joints={total_joints})")

    def publish_telemetry(self):
        # Pre-allocate arrays at total_joints size
        names = [""] * self.total_joints
        positions = [0.0] * self.total_joints
        velocities = [0.0] * self.total_joints
        populated = [False] * self.total_joints

        for mid, joint_name, direction, state_idx, unwrap in self._motors:
            pos, speed = self.motor_manager.read_motor_state(mid)
            if pos is None or speed is None:
                continue

            if unwrap:
                pos = self._unwrap_position(mid, pos)

            names[state_idx] = joint_name
            positions[state_idx] = self._ticks_to_radians(pos) * direction
            velocities[state_idx] = self._speed_to_rad_per_sec(speed) * direction
            populated[state_idx] = True

        # Only publish if at least one motor responded
        if not any(populated):
            return

        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        msg.velocity = velocities
        self.state_pub.publish(msg)

    def _unwrap_position(self, motor_id: int, raw_pos: int) -> int:
        """Track continuous rotation across the 0-4095 wrap boundary."""
        max_ticks = self.ticks_per_rev
        half_ticks = max_ticks // 2

        if motor_id not in self._last_raw_pos:
            self._last_raw_pos[motor_id] = raw_pos
            self._revolution_count[motor_id] = 0
            return raw_pos

        delta = raw_pos - self._last_raw_pos[motor_id]
        if delta > half_ticks:
            self._revolution_count[motor_id] -= 1
        elif delta < -half_ticks:
            self._revolution_count[motor_id] += 1

        self._last_raw_pos[motor_id] = raw_pos
        return raw_pos + self._revolution_count[motor_id] * max_ticks

    def _ticks_to_radians(self, pos_ticks: int) -> float:
        return (pos_ticks - self.servo_center) / self.ticks_per_rev * 2.0 * math.pi

    def _speed_to_rad_per_sec(self, speed: int) -> float:
        steps_per_radian = self.ticks_per_rev / (2.0 * math.pi)
        return speed / steps_per_radian
