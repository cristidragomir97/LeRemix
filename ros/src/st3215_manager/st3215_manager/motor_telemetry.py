#!/usr/bin/env python3
"""Per-motor telemetry publisher (current, voltage, load, temperature).

Motor names are derived from group config — no hardcoded mapping.
"""

from rclpy.node import Node
from std_msgs.msg import Float32, Int32

from .motor_group import MotorGroup


class MotorTelemetrySystem:
    """Publishes detailed telemetry on per-motor topics.

    Topics per motor:
        /motor_telemetry/<joint_name>/current      (Float32, mA)
        /motor_telemetry/<joint_name>/voltage       (Float32, V)
        /motor_telemetry/<joint_name>/load          (Float32, %)
        /motor_telemetry/<joint_name>/temperature   (Int32, C)
    """

    def __init__(self, node: Node, motor_manager, groups: list[MotorGroup],
                 enable: bool, rate: float):
        self.node = node
        self.motor_manager = motor_manager
        self.enable = enable
        self._publishers: dict[int, dict] = {}

        if not enable:
            node.get_logger().info("Motor telemetry disabled")
            return

        # Build name map and create publishers from groups
        for group in groups:
            if not group.enable:
                continue
            for i, mid in enumerate(group.motor_ids):
                name = group.joint_names[i]
                # Topic tokens can't start with a digit; joint names like "1".."6" need a prefix.
                topic_token = name if not name[:1].isdigit() else f"joint_{name}"
                self._publishers[mid] = {
                    'current': node.create_publisher(
                        Float32, f"/motor_telemetry/{topic_token}/current", 10),
                    'voltage': node.create_publisher(
                        Float32, f"/motor_telemetry/{topic_token}/voltage", 10),
                    'load': node.create_publisher(
                        Float32, f"/motor_telemetry/{topic_token}/load", 10),
                    'temperature': node.create_publisher(
                        Int32, f"/motor_telemetry/{topic_token}/temperature", 10),
                }
                node.get_logger().info(
                    f"  Motor telemetry topics for: {name} (ID {mid})")

        node.get_logger().info(f"Motor telemetry enabled at {rate} Hz")

    def publish_telemetry(self):
        if not self.enable:
            return

        for mid, publishers in self._publishers.items():
            telemetry = self.motor_manager.read_motor_telemetry(mid)

            if telemetry['current'] is not None:
                msg = Float32()
                msg.data = float(telemetry['current'])
                publishers['current'].publish(msg)

            if telemetry['voltage'] is not None:
                msg = Float32()
                msg.data = float(telemetry['voltage'])
                publishers['voltage'].publish(msg)

            if telemetry['load'] is not None:
                msg = Float32()
                msg.data = float(telemetry['load'])
                publishers['load'].publish(msg)

            if telemetry['temperature'] is not None:
                msg = Int32()
                msg.data = int(telemetry['temperature'])
                publishers['temperature'].publish(msg)
