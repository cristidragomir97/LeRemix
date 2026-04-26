#!/usr/bin/env python3
"""Low-level ST3215 motor communication and control."""

import math
import time
from st3215 import ST3215
from rclpy.node import Node

from .motor_group import MotorGroup


class MotorManager:
    """Manages serial communication with FEETECH ST3215 servo motors."""

    def __init__(self, node: Node, port: str, baud: int = 1000000):
        self.node = node
        self.port = port
        self.baud = baud
        self.bus: ST3215 | None = None
        self.running_motors: dict[int, int] = {}  # motor_id -> current_speed

    def connect(self) -> bool:
        try:
            self.bus = ST3215(self.port)
            self.node.get_logger().info(f"Connected to serial port: {self.port}")
            return True
        except Exception as e:
            self.node.get_logger().error(f"Failed to open serial port: {e}")
            return False

    def test_connectivity(self, motor_ids: list[int]) -> int:
        """Ping motors and return count of responsive ones."""
        if not motor_ids:
            return 0

        self.node.get_logger().info("Testing motor connectivity...")
        connected = 0
        for mid in motor_ids:
            if self.bus.PingServo(mid):
                connected += 1
                self.node.get_logger().info(f"  Motor {mid}: ONLINE")
            else:
                self.node.get_logger().warn(f"  Motor {mid}: OFFLINE")
        self.node.get_logger().info(f"Connectivity: {connected}/{len(motor_ids)} motors online")
        return connected

    def test_group(self, group: MotorGroup):
        """Run a brief test sequence for a motor group."""
        self.node.get_logger().info(f"Testing group '{group.name}'...")

        for i, mid in enumerate(group.motor_ids):
            if not self.bus.PingServo(mid):
                continue
            try:
                if group.mode == "velocity":
                    self.bus.SetMode(mid, 1)
                    self.bus.StartServo(mid)
                    self.bus.Rotate(mid, 500)
                    time.sleep(1.0)
                    self.bus.Rotate(mid, 0)
                    self.node.get_logger().info(f"  Motor {mid}: velocity test passed")
                else:
                    self.bus.SetMode(mid, 0)
                    self.bus.StartServo(mid)
                    current_pos = self.bus.ReadPosition(mid)
                    if current_pos is not None:
                        test_pos = max(0, min(4095, current_pos + 100))
                        self.bus.MoveTo(mid, test_pos, 500, 200)
                        time.sleep(0.5)
                        self.bus.MoveTo(mid, current_pos, 500, 200)
                        self.node.get_logger().info(f"  Motor {mid}: position test passed")
                time.sleep(0.2)
            except Exception as e:
                self.node.get_logger().warn(f"  Motor {mid} test failed: {e}")

    def initialize_group(self, group: MotorGroup, ticks_per_rev: int):
        """Initialize all motors in a group according to its mode."""
        self.node.get_logger().info(
            f"Initializing group '{group.name}' ({group.mode} mode, "
            f"{group.motor_count} motors)...")

        for i, mid in enumerate(group.motor_ids):
            accel = group.accel[i]
            try:
                if group.mode == "velocity":
                    self._init_velocity_motor(mid, accel)
                else:
                    self._init_position_motor(mid, accel, ticks_per_rev)
                time.sleep(0.1)
            except Exception as e:
                self.node.get_logger().warn(
                    f"Failed to initialize motor {mid} in group '{group.name}': {e}")

        self.node.get_logger().info(f"Group '{group.name}' initialized.")

    def _init_velocity_motor(self, motor_id: int, accel: int):
        self.bus.SetMode(motor_id, 1)
        self.bus.StartServo(motor_id)
        self.running_motors[motor_id] = 0
        self.node.get_logger().info(f"  Motor {motor_id}: velocity mode, started")

    def _init_position_motor(self, motor_id: int, accel: int, ticks_per_rev: int):
        self.bus.SetMode(motor_id, 0)
        self.bus.SetAcceleration(motor_id, accel)

        current_pos = self.bus.ReadPosition(motor_id)
        if current_pos is not None:
            servo_center = ticks_per_rev // 2
            angle_rad = (current_pos - servo_center) / ticks_per_rev * 2.0 * math.pi
            angle_deg = math.degrees(angle_rad)

            self.bus.StartServo(motor_id)
            time.sleep(0.05)
            self.bus.MoveTo(motor_id, current_pos, 100, accel)

            self.node.get_logger().info(
                f"  Motor {motor_id}: position mode, holding at "
                f"{angle_deg:.1f} deg ({current_pos} ticks)")
        else:
            self.node.get_logger().warn(
                f"  Motor {motor_id}: could not read position")

    # -- runtime commands -----------------------------------------------------

    def send_velocity_command(self, motor_id: int, speed: int):
        """Send a velocity command (steps/s). No internal scaling applied."""
        try:
            if speed == 0:
                self.bus.Rotate(motor_id, 0)
                time.sleep(0.01)
                self.bus.StopServo(motor_id)
                self.running_motors.pop(motor_id, None)
            else:
                if motor_id not in self.running_motors:
                    self.bus.StartServo(motor_id)
                self.running_motors[motor_id] = speed
                self.bus.Rotate(motor_id, speed)
        except Exception as e:
            self.node.get_logger().error(
                f"Velocity command failed for motor {motor_id}: {e}")

    def send_position_command(self, motor_id: int, pos_ticks: int,
                              speed: int, accel: int):
        """Send a position command (ticks)."""
        self.bus.MoveTo(motor_id, pos_ticks, speed, accel)

    # -- readback -------------------------------------------------------------

    def read_motor_state(self, motor_id: int):
        """Read position (ticks) and speed (steps/s). Returns (pos, speed) or (None, None)."""
        try:
            pos = self.bus.ReadPosition(motor_id)
            speed_data = self.bus.ReadSpeed(motor_id)

            if pos is not None and speed_data is not None:
                if isinstance(speed_data, tuple) and len(speed_data) >= 3:
                    speed, comm_result, error = speed_data[0], speed_data[1], speed_data[2]
                    if comm_result == 0 and error == 0:
                        return pos, speed
                    return pos, 0
                else:
                    speed = speed_data[0] if isinstance(speed_data, tuple) else speed_data
                    return pos, speed
        except Exception:
            pass
        return None, None

    def read_motor_telemetry(self, motor_id: int) -> dict:
        """Read current (mA), voltage (V), load (%), temperature (C)."""
        telemetry = {
            'current': None,
            'voltage': None,
            'load': None,
            'temperature': None,
        }
        try:
            telemetry['current'] = self.bus.ReadCurrent(motor_id)
            telemetry['voltage'] = self.bus.ReadVoltage(motor_id)
            telemetry['load'] = self.bus.ReadLoad(motor_id)
            telemetry['temperature'] = self.bus.ReadTemperature(motor_id)
        except Exception as e:
            self.node.get_logger().debug(
                f"Telemetry read failed for motor {motor_id}: {e}")
        return telemetry
