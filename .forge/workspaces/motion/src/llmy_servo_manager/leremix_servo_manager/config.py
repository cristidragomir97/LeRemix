#!/usr/bin/env python3
"""
Configuration Management for LLMy Robot
Handles parameter loading and validation
"""

from rclpy.node import Node


class ServoManagerConfig:
    """Configuration container for servo manager parameters"""
    
    def __init__(self, node: Node):
        self.node = node
        self._declare_parameters()
        self._load_parameters()
        self._validate_parameters()
    
    def _declare_parameters(self):
        """Declare all ROS parameters with default values"""
        # Communication parameters
        self.node.declare_parameter("port", "/dev/ttyTHS1")
        self.node.declare_parameter("baud", 1000000)
        self.node.declare_parameter("ticks_per_rev", 4096)
        self.node.declare_parameter("telemetry_rate", 30.0)

        # Motor group enables
        self.node.declare_parameter("loc_enable", True)
        self.node.declare_parameter("arm_enable", True)
        self.node.declare_parameter("camera_enable", True)

        # Motor IDs
        # Locomotion: 1=front_right, 2=back_right, 3=back_left, 4=front_left
        # Arm: 5=base_rotation, 6=shoulder, 7=elbow, 8=wrist, 9=wrist_rotation, 10=gripper
        # Camera: 11=tilt
        self.node.declare_parameter("loc_ids", [1, 2, 3, 4])
        self.node.declare_parameter("arm_ids", [5, 6, 7, 8, 9, 10])
        self.node.declare_parameter("camera_ids", [11])

        # Acceleration settings
        self.node.declare_parameter("loc_accel", [15, 15, 15, 15])
        self.node.declare_parameter("arm_accel", [20, 20, 15, 15, 25, 25])
        self.node.declare_parameter("camera_accel", [15])

        # Speed scaling
        self.node.declare_parameter("loc_speed_scale", 0.2)
        self.node.declare_parameter("arm_speed_scale", 1.0)
        self.node.declare_parameter("camera_speed_scale", 1.0)

        # Braking parameters
        self.node.declare_parameter("brake_method", "torque_disable")
        self.node.declare_parameter("velocity_ramp_time", 0.5)
        self.node.declare_parameter("brake_acceleration", 100)
    
    def _load_parameters(self):
        """Load all parameters from ROS parameter server"""
        # Communication parameters
        self.port = self.node.get_parameter("port").get_parameter_value().string_value
        self.baud = self.node.get_parameter("baud").get_parameter_value().integer_value
        self.ticks_per_rev = self.node.get_parameter("ticks_per_rev").get_parameter_value().integer_value
        self.telemetry_rate = self.node.get_parameter("telemetry_rate").get_parameter_value().double_value

        # Motor group enables
        self.loc_enable = self.node.get_parameter("loc_enable").get_parameter_value().bool_value
        self.arm_enable = self.node.get_parameter("arm_enable").get_parameter_value().bool_value
        self.camera_enable = self.node.get_parameter("camera_enable").get_parameter_value().bool_value

        # Motor IDs
        self.loc_ids = self.node.get_parameter("loc_ids").get_parameter_value().integer_array_value
        self.arm_ids = self.node.get_parameter("arm_ids").get_parameter_value().integer_array_value
        self.camera_ids = self.node.get_parameter("camera_ids").get_parameter_value().integer_array_value

        # Acceleration settings
        self.loc_accel = self.node.get_parameter("loc_accel").get_parameter_value().integer_array_value
        self.arm_accel = self.node.get_parameter("arm_accel").get_parameter_value().integer_array_value
        self.camera_accel = self.node.get_parameter("camera_accel").get_parameter_value().integer_array_value

        # Speed scaling
        self.loc_speed_scale = self.node.get_parameter("loc_speed_scale").get_parameter_value().double_value
        self.arm_speed_scale = self.node.get_parameter("arm_speed_scale").get_parameter_value().double_value
        self.camera_speed_scale = self.node.get_parameter("camera_speed_scale").get_parameter_value().double_value

        # Braking parameters
        self.brake_method = self.node.get_parameter("brake_method").get_parameter_value().string_value
        self.velocity_ramp_time = self.node.get_parameter("velocity_ramp_time").get_parameter_value().double_value
        self.brake_acceleration = self.node.get_parameter("brake_acceleration").get_parameter_value().integer_value
    
    def _validate_parameters(self):
        """Validate parameter values"""
        # Validate brake method
        valid_brake_methods = ["torque_disable", "velocity_ramp", "position_brake"]
        if self.brake_method not in valid_brake_methods:
            self.node.get_logger().warn(f"Invalid brake method '{self.brake_method}', using 'torque_disable'")
            self.brake_method = "torque_disable"

        # Validate telemetry rate
        if self.telemetry_rate <= 0:
            self.node.get_logger().warn(f"Invalid telemetry rate {self.telemetry_rate}, using 30.0 Hz")
            self.telemetry_rate = 30.0

        # Validate speed scales
        for attr, name in [("loc_speed_scale", "loc"), ("arm_speed_scale", "arm"), ("camera_speed_scale", "camera")]:
            value = getattr(self, attr)
            if value < 0 or value > 2.0:
                self.node.get_logger().warn(f"Invalid {name} speed scale {value}, clamping to [0, 2.0]")
                setattr(self, attr, max(0.0, min(2.0, value)))
    
    def log_configuration(self):
        """Log the current configuration"""
        self.node.get_logger().info("Configuration loaded:")
        self.node.get_logger().info(f"  Serial Port: {self.port}")
        self.node.get_logger().info(f"  Baud Rate: {self.baud}")
        self.node.get_logger().info(f"  Ticks per Revolution: {self.ticks_per_rev}")
        self.node.get_logger().info(f"  Telemetry Rate: {self.telemetry_rate} Hz")
        self.node.get_logger().info(f"  Brake Method: {self.brake_method}")
        self.node.get_logger().info(f"  Locomotion Enabled: {'YES' if self.loc_enable else 'NO'} (Speed: {self.loc_speed_scale*100:.1f}%)")
        self.node.get_logger().info(f"  Arm Enabled: {'YES' if self.arm_enable else 'NO'} (Speed: {self.arm_speed_scale*100:.1f}%)")
        self.node.get_logger().info(f"  Camera Enabled: {'YES' if self.camera_enable else 'NO'} (Speed: {self.camera_speed_scale*100:.1f}%)")
    
    def get_enabled_motor_ids(self) -> list:
        """Get list of all enabled motor IDs"""
        enabled_ids = []
        if self.loc_enable:
            enabled_ids.extend(self.loc_ids)
        if self.arm_enable:
            enabled_ids.extend(self.arm_ids)
        if self.camera_enable:
            enabled_ids.extend(self.camera_ids)
        return enabled_ids

    def get_motor_counts(self) -> tuple:
        """Get counts of enabled and total motors"""
        enabled_count = (len(self.loc_ids) if self.loc_enable else 0) + \
                       (len(self.arm_ids) if self.arm_enable else 0) + \
                       (len(self.camera_ids) if self.camera_enable else 0)
        total_count = len(self.loc_ids) + len(self.arm_ids) + len(self.camera_ids)
        return enabled_count, total_count