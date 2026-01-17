#!/usr/bin/env python3
"""
Brake System for LLMy Robot
Handles safe motor braking to prevent power spikes
"""

import time
import threading
from rclpy.node import Node


class BrakeSystem:
    """Manages safe motor braking using different methods"""
    
    def __init__(self, node: Node, motor_manager, brake_method: str = "torque_disable", 
                 velocity_ramp_time: float = 0.5, brake_acceleration: int = 100):
        self.node = node
        self.motor_manager = motor_manager
        self.brake_method = brake_method
        self.velocity_ramp_time = velocity_ramp_time
        self.brake_acceleration = brake_acceleration
        
        # Braking state tracking
        self.braking_motors = set()
        self.brake_threads = {}
    
    def safe_brake_motor(self, motor_id: int, current_speed: int):
        """Safely brake a motor using the configured method to prevent power spikes"""
        if motor_id in self.braking_motors:
            return  # Already braking
            
        self.braking_motors.add(motor_id)
        
        try:
            if self.brake_method == "torque_disable":
                self._torque_disable_brake(motor_id)
                
            elif self.brake_method == "velocity_ramp":
                self._velocity_ramp_brake(motor_id, current_speed)
                
            elif self.brake_method == "position_brake":
                self._position_brake(motor_id)
                
            else:
                # Fallback to torque disable
                self.node.get_logger().warn(f"Unknown brake method {self.brake_method}, using torque disable")
                self._torque_disable_brake(motor_id)
                
        except Exception as e:
            self.node.get_logger().error(f"Failed to brake motor {motor_id}: {e}")
        finally:
            self.braking_motors.discard(motor_id)
            if hasattr(self.motor_manager, 'running_motors') and motor_id in self.motor_manager.running_motors:
                del self.motor_manager.running_motors[motor_id]
    
    def _torque_disable_brake(self, motor_id: int):
        """Apply torque disable braking method (gentlest)"""
        self.node.get_logger().debug(f"Applying torque disable brake to motor {motor_id}")
        self.motor_manager.motor_manager.StopServo(motor_id)
        time.sleep(0.1)
    
    def _velocity_ramp_brake(self, motor_id: int, current_speed: int):
        """Apply velocity ramp braking method"""
        self.node.get_logger().debug(f"Applying velocity ramp brake to motor {motor_id}")
        steps = 10
        step_time = self.velocity_ramp_time / steps
        
        for i in range(steps):
            ramp_speed = int(current_speed * (1.0 - (i + 1) / steps))
            self.motor_manager.motor_manager.Rotate(motor_id, ramp_speed)
            time.sleep(step_time)
    
    def _position_brake(self, motor_id: int):
        """Apply position braking method"""
        self.node.get_logger().debug(f"Applying position brake to motor {motor_id}")
        current_pos = self.motor_manager.motor_manager.ReadPosition(motor_id)
        if current_pos is not None:
            self.motor_manager.motor_manager.MoveTo(motor_id, current_pos, 255, min(255, self.brake_acceleration))
    
    def emergency_stop_all(self, motor_ids: list):
        """Emergency stop all specified motors"""
        self.node.get_logger().warn("Emergency stop activated for all motors")
        for motor_id in motor_ids:
            try:
                self.motor_manager.motor_manager.StopServo(motor_id)
            except Exception as e:
                self.node.get_logger().error(f"Failed to emergency stop motor {motor_id}: {e}")
    
    def is_motor_braking(self, motor_id: int) -> bool:
        """Check if a motor is currently braking"""
        return motor_id in self.braking_motors