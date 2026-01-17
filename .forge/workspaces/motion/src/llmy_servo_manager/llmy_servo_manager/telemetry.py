#!/usr/bin/env python3
"""
Telemetry System for LLMy Robot
Handles motor state publishing and monitoring
"""

from rclpy.node import Node
from sensor_msgs.msg import JointState


class TelemetrySystem:
    """Manages motor telemetry publishing"""
    
    def __init__(self, node: Node, motor_manager, config):
        self.node = node
        self.motor_manager = motor_manager
        self.config = config
        
        # Create publisher
        self.state_pub = self.node.create_publisher(JointState, "/motor_manager/joint_states", 10)
        self.node.get_logger().info("  ✅ Joint state publisher: /motor_manager/joint_states")
    
    def publish_telemetry(self):
        """Publish motor telemetry as JointState message"""
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        
        # Read all enabled motors
        enabled_ids = self.config.get_enabled_motor_ids()
            
        for motor_id in enabled_ids:
            pos, speed = self.motor_manager.read_motor_state(motor_id)
            
            if pos is not None and speed is not None:
                # Map motor IDs to ros2_control joint names
                joint_name = self._motor_id_to_joint_name(motor_id)
                msg.name.append(joint_name)
                
                # Convert servo ticks to radians
                pos_rad = self._ticks_to_radians(pos)
                msg.position.append(pos_rad)
                
                # Convert speed to rad/s
                vel_rad_s = self._speed_to_rad_per_sec(speed)
                msg.velocity.append(vel_rad_s)
                
        if len(msg.name) > 0:
            self.state_pub.publish(msg)
    
    def _ticks_to_radians(self, pos_ticks: int) -> float:
        """Convert servo ticks to radians"""
        import math
        servo_center = 2048
        return (pos_ticks - servo_center) / self.config.ticks_per_rev * 2.0 * math.pi
    
    def _speed_to_rad_per_sec(self, speed: int) -> float:
        """Convert motor speed (steps/s) to rad/s

        Formula: rad/s = steps/s / (ticks_per_rev / 2π)
        Where 1 revolution = ticks_per_rev steps = 2π radians
        """
        import math
        steps_per_radian = self.config.ticks_per_rev / (2.0 * math.pi)
        return speed / steps_per_radian
    
    def _motor_id_to_joint_name(self, motor_id: int) -> str:
        """Map motor ID to ros2_control joint name"""
        if motor_id in self.config.loc_ids:
            # Base motors: 2-wheel differential drive
            # Motor ID 1 = right_wheel, Motor ID 2 = left_wheel
            wheel_id_to_name = {
                1: "right_wheel_joint",
                2: "left_wheel_joint",
            }
            return wheel_id_to_name.get(motor_id, f"wheel_{motor_id}_joint")
        elif motor_id in self.config.arm_ids:
            # Arm motors: map motor IDs [5,6,7,8,9,10] to joint names ["1","2","3","4","5","6"]
            idx = self.config.arm_ids.index(motor_id)
            return str(idx + 1)
        elif motor_id in self.config.camera_ids:
            # Camera motor: ID 11 = camera_tilt
            camera_id_to_name = {
                11: "camera_tilt"
            }
            return camera_id_to_name.get(motor_id, f"camera_joint_{motor_id}")
        else:
            # Fallback for unknown motors
            return f"motor_{motor_id}"
    
    def get_motor_summary(self) -> dict:
        """Get summary of motor states for diagnostics"""
        enabled_ids = self.config.get_enabled_motor_ids()
        summary = {
            "total_enabled": len(enabled_ids),
            "responding": 0,
            "positions": {},
            "velocities": {}
        }
        
        for motor_id in enabled_ids:
            pos, speed = self.motor_manager.read_motor_state(motor_id)
            if pos is not None and speed is not None:
                summary["responding"] += 1
                summary["positions"][motor_id] = self._ticks_to_radians(pos)
                summary["velocities"][motor_id] = self._speed_to_rad_per_sec(speed)
        
        return summary