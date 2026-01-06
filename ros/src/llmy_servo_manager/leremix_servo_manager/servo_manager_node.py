#!/usr/bin/env python3
"""
Python-based Servo Manager Node for LLMy Robot
Uses ST3215 library with improved braking logic to prevent power spikes
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray

from .config import ServoManagerConfig
from .motor_manager import MotorManager
from .brake_system import BrakeSystem
from .command_handlers import CommandHandlers

from .telemetry import TelemetrySystem

class ServoManagerNode(Node):
    def __init__(self):
        super().__init__('servo_manager_node_py')
        
        self.get_logger().info("=== LLMy Python Servo Manager Node Starting ===")
        
        # Load configuration
        self.get_logger().info("Loading servo manager configuration...")
        self.config = ServoManagerConfig(self)
        self.config.log_configuration()
        
        # Initialize motor manager
        self.motor_manager = MotorManager(self, self.config.port, self.config.baud)
        
        # Connect to motor bus
        self.get_logger().info("Attempting to connect to FEETECH servo bus...")
        if not self.motor_manager.connect():
            raise RuntimeError("Failed to connect to FEETECH servo bus")
        
        # Test connectivity and initialize motors
        enabled_ids = self.config.get_enabled_motor_ids()
        connected_count = self.motor_manager.test_connectivity(enabled_ids)
        if connected_count > 0:
            self.motor_manager.run_test_sequences(enabled_ids, self.config.loc_ids,
                                                self.config.arm_ids, self.config.camera_ids,
                                                self.config.loc_enable)
        
        # Add delay between test sequences and initialization to prevent bus congestion
        import time
        time.sleep(0.5)
        
        self._initialize_motors()
        
        # Initialize brake system
        self.brake_system = BrakeSystem(self, self.motor_manager, self.config.brake_method,
                                       self.config.velocity_ramp_time, self.config.brake_acceleration)
        
        # Initialize command handlers
        self.command_handlers = CommandHandlers(self, self.motor_manager, self.config)
        
        # Initialize telemetry system
        self.telemetry = TelemetrySystem(self, self.motor_manager, self.config)
        
        self._setup_ros_communication()
        
        # Publish initial joint states immediately to inform ros2_control of current positions
        self.get_logger().info("Publishing initial joint states for ros2_control awareness...")
        self.telemetry.publish_telemetry()
        import time
        time.sleep(0.1)  # Brief delay to ensure message is published
        self.telemetry.publish_telemetry()  # Publish twice to ensure ros2_control receives it
        
        self._log_startup_summary()
    
    def _initialize_motors(self):
        """Initialize all motor groups"""
        self.get_logger().info("Initializing motor modes and positions...")

        if self.config.loc_enable:
            self.motor_manager.initialize_locomotion_motors(self.config.loc_ids, self.config.loc_accel)

        if self.config.arm_enable:
            self.motor_manager.initialize_arm_motors(self.config.arm_ids, self.config.arm_accel, self.config.ticks_per_rev)

        if self.config.camera_enable:
            self.motor_manager.initialize_camera_motors(self.config.camera_ids, self.config.camera_accel, self.config.ticks_per_rev)

        self.get_logger().info("Motor initialization complete!")
    
    def _setup_ros_communication(self):
        """Setup ROS subscribers, publishers and timers"""
        self.get_logger().info("Setting up ROS2 communication...")

        # Create QoS profile matching C++ version
        cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Create subscribers
        self.get_logger().info("Creating command subscribers:")
        self.base_cmd_sub = self.create_subscription(
            Float64MultiArray,
            "/motor_manager/base_cmd",
            self.command_handlers.handle_base_command,
            cmd_qos
        )
        self.get_logger().info("  Base command subscriber: /motor_manager/base_cmd")

        self.arm_cmd_sub = self.create_subscription(
            Float64MultiArray,
            "/motor_manager/arm_cmd",
            self.command_handlers.handle_arm_command,
            cmd_qos
        )
        self.get_logger().info("  Arm command subscriber: /motor_manager/arm_cmd")

        self.camera_cmd_sub = self.create_subscription(
            Float64MultiArray,
            "/motor_manager/camera_cmd",
            self.command_handlers.handle_camera_command,
            cmd_qos
        )
        self.get_logger().info("  Camera command subscriber: /motor_manager/camera_cmd")

        # Create timer for telemetry
        timer_period = 1.0 / self.config.telemetry_rate
        self.timer = self.create_timer(timer_period, self.telemetry.publish_telemetry)
        self.get_logger().info(f"  Telemetry timer started at {self.config.telemetry_rate} Hz")
    
    def _log_startup_summary(self):
        """Log startup summary information"""
        self.get_logger().info("=== Python Servo Manager Node Successfully Started ===")
        self.get_logger().info("System Summary:")
        self.get_logger().info(f"  Locomotion motors: {len(self.config.loc_ids)} ({'ENABLED' if self.config.loc_enable else 'DISABLED'})")
        self.get_logger().info(f"  Arm motors: {len(self.config.arm_ids)} ({'ENABLED' if self.config.arm_enable else 'DISABLED'})")
        self.get_logger().info(f"  Camera motors: {len(self.config.camera_ids)} ({'ENABLED' if self.config.camera_enable else 'DISABLED'})")

        enabled_count, total_count = self.config.get_motor_counts()
        self.get_logger().info(f"  Total motors enabled: {enabled_count}/{total_count}")
        self.get_logger().info("----------------------------------")
        self.get_logger().info("Ready to receive motor commands and publish telemetry!")

def main(args=None):
    rclpy.init(args=args)

    try:
        node = ServoManagerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error during servo manager initialization: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()