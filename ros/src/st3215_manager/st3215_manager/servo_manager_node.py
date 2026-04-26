#!/usr/bin/env python3
"""ST3215 Servo Manager Node — generic, config-driven motor controller."""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray

from .config import ServoManagerConfig
from .motor_manager import MotorManager
from .brake_system import BrakeSystem
from .command_handler import GroupCommandHandler
from .telemetry import TelemetrySystem
from .motor_telemetry import MotorTelemetrySystem


class ServoManagerNode(Node):
    def __init__(self):
        super().__init__('servo_manager_node')

        self.get_logger().info("=== ST3215 Servo Manager Starting ===")

        # Load configuration
        self.config = ServoManagerConfig(self)
        self.config.log_configuration()

        # Connect to motor bus
        self.motor_manager = MotorManager(self, self.config.port, self.config.baud)
        if not self.motor_manager.connect():
            raise RuntimeError("Failed to connect to ST3215 servo bus")

        # Test connectivity
        enabled_ids = self.config.get_all_enabled_motor_ids()
        self.motor_manager.test_connectivity(enabled_ids)

        # Optional startup test sequences
        if self.config.test_on_startup:
            for group in self.config.get_enabled_groups():
                self.motor_manager.test_group(group)
            time.sleep(0.5)

        # Initialize each enabled group
        for group in self.config.get_enabled_groups():
            self.motor_manager.initialize_group(group, self.config.ticks_per_rev)

        # Brake system
        self.brake_system = BrakeSystem(
            self, self.motor_manager,
            self.config.brake_method,
            self.config.velocity_ramp_time,
            self.config.brake_acceleration)

        # Command handlers + subscribers (one per enabled group)
        cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT)

        self._command_handlers: list[GroupCommandHandler] = []
        self._subscribers = []

        for group in self.config.get_enabled_groups():
            handler = GroupCommandHandler(
                self, self.motor_manager, group, self.config.ticks_per_rev)
            sub = self.create_subscription(
                Float64MultiArray, group.topic, handler.handle_command, cmd_qos)
            self._command_handlers.append(handler)
            self._subscribers.append(sub)
            self.get_logger().info(f"  Subscriber: {group.topic} -> group '{group.name}'")

        # Telemetry
        enabled_groups = self.config.get_enabled_groups()

        self.telemetry = TelemetrySystem(
            self, self.motor_manager, enabled_groups,
            self.config.ticks_per_rev, self.config.joint_states_topic,
            self.config.total_joints)

        self.motor_telemetry = MotorTelemetrySystem(
            self, self.motor_manager, enabled_groups,
            self.config.motor_telemetry_enable,
            self.config.motor_telemetry_rate)

        # Telemetry timers
        self.create_timer(
            1.0 / self.config.telemetry_rate,
            self.telemetry.publish_telemetry)
        self.get_logger().info(f"  Telemetry timer: {self.config.telemetry_rate} Hz")

        if self.config.motor_telemetry_enable:
            self.create_timer(
                1.0 / self.config.motor_telemetry_rate,
                self.motor_telemetry.publish_telemetry)
            self.get_logger().info(
                f"  Motor telemetry timer: {self.config.motor_telemetry_rate} Hz")

        # Publish initial joint states for ros2_control awareness
        self.telemetry.publish_telemetry()
        time.sleep(0.1)
        self.telemetry.publish_telemetry()

        # Startup summary
        self.get_logger().info("=== ST3215 Servo Manager Ready ===")
        for group in enabled_groups:
            self.get_logger().info(
                f"  [{group.name}] {group.mode} | "
                f"{group.motor_count} motors | {group.topic}")
        self.get_logger().info(
            f"  Total: {len(enabled_ids)} motors enabled")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ServoManagerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
