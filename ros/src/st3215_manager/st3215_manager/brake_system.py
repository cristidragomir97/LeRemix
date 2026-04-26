#!/usr/bin/env python3
"""Brake system for safe motor stopping. Already generic — copied as-is."""

import time
from rclpy.node import Node


class BrakeSystem:
    """Manages safe motor braking using configurable methods."""

    def __init__(self, node: Node, motor_manager, brake_method: str = "torque_disable",
                 velocity_ramp_time: float = 0.5, brake_acceleration: int = 100):
        self.node = node
        self.motor_manager = motor_manager
        self.brake_method = brake_method
        self.velocity_ramp_time = velocity_ramp_time
        self.brake_acceleration = brake_acceleration
        self.braking_motors: set[int] = set()

    def safe_brake_motor(self, motor_id: int, current_speed: int):
        if motor_id in self.braking_motors:
            return

        self.braking_motors.add(motor_id)
        try:
            if self.brake_method == "velocity_ramp":
                self._velocity_ramp_brake(motor_id, current_speed)
            elif self.brake_method == "position_brake":
                self._position_brake(motor_id)
            else:
                self._torque_disable_brake(motor_id)
        except Exception as e:
            self.node.get_logger().error(f"Failed to brake motor {motor_id}: {e}")
        finally:
            self.braking_motors.discard(motor_id)
            self.motor_manager.running_motors.pop(motor_id, None)

    def _torque_disable_brake(self, motor_id: int):
        self.motor_manager.bus.StopServo(motor_id)
        time.sleep(0.1)

    def _velocity_ramp_brake(self, motor_id: int, current_speed: int):
        steps = 10
        step_time = self.velocity_ramp_time / steps
        for i in range(steps):
            ramp_speed = int(current_speed * (1.0 - (i + 1) / steps))
            self.motor_manager.bus.Rotate(motor_id, ramp_speed)
            time.sleep(step_time)

    def _position_brake(self, motor_id: int):
        current_pos = self.motor_manager.bus.ReadPosition(motor_id)
        if current_pos is not None:
            self.motor_manager.bus.MoveTo(
                motor_id, current_pos, 255, min(255, self.brake_acceleration))

    def emergency_stop_all(self, motor_ids: list[int]):
        self.node.get_logger().warn("Emergency stop activated")
        for mid in motor_ids:
            try:
                self.motor_manager.bus.StopServo(mid)
            except Exception as e:
                self.node.get_logger().error(f"Emergency stop failed for motor {mid}: {e}")

    def is_motor_braking(self, motor_id: int) -> bool:
        return motor_id in self.braking_motors
