#!/usr/bin/env python3
"""
Motor Telemetry System for LLMy Robot
Publishes individual motor telemetry (current, voltage, load, temperature) on separate topics
"""

from rclpy.node import Node
from std_msgs.msg import Float32, Int32


class MotorTelemetrySystem:
    """Publishes detailed motor telemetry on individual topics

    Topics published per motor:
        /motor_telemetry/<name>/current      - Current draw in mA (Float32)
        /motor_telemetry/<name>/voltage      - Voltage in V (Float32)
        /motor_telemetry/<name>/load         - Load percentage (Float32)
        /motor_telemetry/<name>/temperature  - Temperature in C (Int32)
    """

    # Motor ID to name mapping
    MOTOR_NAMES = {
        1: "right_wheel",
        2: "left_wheel",
        3: "arm_base",
        4: "arm_shoulder",
        5: "arm_elbow",
        6: "arm_wrist",
        7: "arm_wrist_rotation",
        8: "arm_gripper",
        11: "tilt",
    }

    def __init__(self, node: Node, motor_manager, config):
        self.node = node
        self.motor_manager = motor_manager
        self.config = config

        # Publishers dict: motor_id -> {metric: publisher}
        self._publishers = {}

        if not config.motor_telemetry_enable:
            self.node.get_logger().info("Motor telemetry disabled")
            return

        self._create_publishers()
        self.node.get_logger().info(f"Motor telemetry enabled at {config.motor_telemetry_rate} Hz")

    def _get_motor_name(self, motor_id: int) -> str:
        """Get human-readable name for motor ID, fallback to motor_<id>"""
        return self.MOTOR_NAMES.get(motor_id, f"motor_{motor_id}")

    def _create_publishers(self):
        """Create publishers for all enabled motors"""
        enabled_ids = self.config.get_enabled_motor_ids()

        for motor_id in enabled_ids:
            name = self._get_motor_name(motor_id)
            self._publishers[motor_id] = {
                'current': self.node.create_publisher(
                    Float32, f"/motor_telemetry/{name}/current", 10),
                'voltage': self.node.create_publisher(
                    Float32, f"/motor_telemetry/{name}/voltage", 10),
                'load': self.node.create_publisher(
                    Float32, f"/motor_telemetry/{name}/load", 10),
                'temperature': self.node.create_publisher(
                    Int32, f"/motor_telemetry/{name}/temperature", 10),
            }
            self.node.get_logger().info(f"  Motor telemetry topics created for: {name} (ID {motor_id})")

    def publish_telemetry(self):
        """Read and publish telemetry for all enabled motors"""
        if not self.config.motor_telemetry_enable:
            return

        for motor_id, publishers in self._publishers.items():
            telemetry = self.motor_manager.read_motor_telemetry(motor_id)

            # Publish current (mA)
            if telemetry['current'] is not None:
                msg = Float32()
                msg.data = float(telemetry['current'])
                publishers['current'].publish(msg)

            # Publish voltage (V)
            if telemetry['voltage'] is not None:
                msg = Float32()
                msg.data = float(telemetry['voltage'])
                publishers['voltage'].publish(msg)

            # Publish load (%)
            if telemetry['load'] is not None:
                msg = Float32()
                msg.data = float(telemetry['load'])
                publishers['load'].publish(msg)

            # Publish temperature (C)
            if telemetry['temperature'] is not None:
                msg = Int32()
                msg.data = int(telemetry['temperature'])
                publishers['temperature'].publish(msg)
