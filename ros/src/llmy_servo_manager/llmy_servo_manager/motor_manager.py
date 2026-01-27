#!/usr/bin/env python3
"""
Motor Manager for LLMy Robot
Handles low-level motor communication and control using ST3215 library
"""

import math
import time
from st3215 import ST3215
from rclpy.node import Node


class MotorManager:
    """Manages communication and control of FEETECH servo motors"""
    
    def __init__(self, node: Node, port: str, baud: int = 1000000):
        self.node = node
        self.port = port
        self.baud = baud
        self.motor_manager = None
        self.velocity_mode_initialized = set()
        self.running_motors = {}  # motor_id -> current_speed
        
    def connect(self):
        """Connect to the motor bus"""
        try:
            self.motor_manager = ST3215(self.port)
            self.node.get_logger().info(f"✅ Successfully connected to serial port: {self.port}")
            return True
        except Exception as e:
            self.node.get_logger().error(f"❌ Failed to open serial port: {e}")
            self.node.get_logger().error("Please check:")
            self.node.get_logger().error("  - Serial port exists and has correct permissions")
            self.node.get_logger().error("  - Device is connected and powered on")
            self.node.get_logger().error("  - No other processes are using the port")
            return False
    
    def test_connectivity(self, enabled_ids: list):
        """Test connectivity with enabled motors"""
        self.node.get_logger().info("Testing enabled motor connectivity...")
        
        if not enabled_ids:
            self.node.get_logger().warn("No motor groups enabled - skipping connectivity test")
            return 0
            
        connected_motors = 0
        for motor_id in enabled_ids:
            if self.motor_manager.PingServo(motor_id):
                connected_motors += 1
                self.node.get_logger().info(f"  Motor ID {motor_id}: ✅ ONLINE")
            else:
                self.node.get_logger().warn(f"  Motor ID {motor_id}: ❌ OFFLINE")
                
        self.node.get_logger().info(f"Motor connectivity test complete: {connected_motors}/{len(enabled_ids)} enabled motors online")
        return connected_motors
    
    def run_test_sequences(self, motor_ids: list, loc_ids: list, arm_ids: list, camera_ids: list, loc_enable: bool):
        """Run short test sequences to verify motor communication"""
        self.node.get_logger().info("Running motor communication test sequences...")

        for motor_id in motor_ids:
            if not self.motor_manager.PingServo(motor_id):
                continue

            try:
                self.node.get_logger().info(f"Testing motor {motor_id}...")

                # Test 1: Brief spin in velocity mode for locomotion motors
                if motor_id in loc_ids and loc_enable:
                    self.motor_manager.SetMode(motor_id, 1)  # Velocity mode
                    self.motor_manager.StartServo(motor_id)
                    self.motor_manager.Rotate(motor_id, 500)  # Slow spin
                    time.sleep(1.0)
                    self.motor_manager.Rotate(motor_id, 0)
                    # Don't stop servo - it will be reinitialized later
                    self.node.get_logger().info(f"  Motor {motor_id}: Velocity test passed")

                # Test 2: Small position move for arm/camera motors
                elif motor_id in arm_ids + camera_ids:
                    self.motor_manager.SetMode(motor_id, 0)  # Position mode
                    self.motor_manager.StartServo(motor_id)

                    # Read current position and move slightly
                    current_pos = self.motor_manager.ReadPosition(motor_id)
                    if current_pos is not None:
                        test_pos = current_pos + 100  # Small movement
                        test_pos = max(0, min(4095, test_pos))

                        self.motor_manager.MoveTo(motor_id, test_pos, 500, 200)
                        time.sleep(0.5)
                        self.motor_manager.MoveTo(motor_id, current_pos, 500, 200)
                        self.node.get_logger().info(f"  Motor {motor_id}: Position test passed")

                time.sleep(0.2)  # Brief pause between motors

            except Exception as e:
                self.node.get_logger().warn(f"Test failed for motor {motor_id}: {e}")

        self.node.get_logger().info("Motor test sequences complete!")
    
    def initialize_locomotion_motors(self, loc_ids: list, loc_accel: list):
        """Initialize locomotion motors for velocity mode"""
        self.node.get_logger().info("Setting locomotion motors to velocity mode...")
        
        for i, motor_id in enumerate(loc_ids):
            accel = loc_accel[i] if i < len(loc_accel) else 50
            
            try:
                self.motor_manager.SetMode(motor_id, 1)  # Velocity mode
                self.motor_manager.StartServo(motor_id)
                
                self.velocity_mode_initialized.add(motor_id)
                self.running_motors[motor_id] = 0
                self.node.get_logger().info(f"Initialized locomotion motor {motor_id}")
            except Exception as e:
                self.node.get_logger().warn(f"Failed to initialize locomotion motor {motor_id}: {e}")
    
    def initialize_arm_motors(self, arm_ids: list, arm_accel: list, ticks_per_rev: int):
        """Initialize arm motors for position mode - reads current positions and sets holding torque"""
        self.node.get_logger().info("Setting arm motors to position mode...")
        self.node.get_logger().info("🛡️  SAFETY: Reading current arm positions and enabling torque holding")
        
        # Store current positions for safety - no movements during initialization
        self.arm_current_positions = {}
        
        for i, motor_id in enumerate(arm_ids):
            accel = arm_accel[i] if i < len(arm_accel) else 20
            
            try:
                # Set mode and acceleration but don't start servo yet
                self.motor_manager.SetMode(motor_id, 0)
                self.motor_manager.SetAcceleration(motor_id, accel)
                
                # Read current position instead of moving to home
                current_pos_ticks = self.motor_manager.ReadPosition(motor_id)
                if current_pos_ticks is not None:
                    # Convert to radians and degrees for logging
                    servo_center = 2048
                    current_angle_rad = (current_pos_ticks - servo_center) / ticks_per_rev * 2.0 * math.pi
                    current_angle_deg = current_angle_rad * 180.0 / math.pi
                    
                    # Store current position
                    self.arm_current_positions[motor_id] = current_pos_ticks
                    
                    # Start servo and immediately command it to hold current position
                    self.motor_manager.StartServo(motor_id)
                    time.sleep(0.05)  # Brief delay to ensure servo is started
                    self.motor_manager.MoveTo(motor_id, current_pos_ticks, 100, accel)
                    
                    self.node.get_logger().info(f"Arm motor {motor_id} holding at: {current_angle_deg:.1f}° ({current_angle_rad:.3f} rad, {current_pos_ticks} ticks)")
                else:
                    self.node.get_logger().warn(f"Could not read current position for arm motor {motor_id}")
                    
                time.sleep(0.1)  # Small delay to prevent bus congestion
                    
            except Exception as e:
                self.node.get_logger().warn(f"Failed to initialize arm motor {motor_id}: {e}")
        
        self.node.get_logger().info("✅ Arm motors initialized with torque holding at current positions")
    
    def initialize_camera_motors(self, camera_ids: list, camera_accel: list, ticks_per_rev: int):
        """Initialize camera motors for position mode - reads current positions and sets holding torque"""
        self.node.get_logger().info("Setting camera motors to position mode...")
        self.node.get_logger().info("SAFETY: Reading current camera positions and enabling torque holding")

        # Store current positions for safety - no movements during initialization
        self.camera_current_positions = {}

        for i, motor_id in enumerate(camera_ids):
            accel = camera_accel[i] if i < len(camera_accel) else 30

            try:
                # Set mode and acceleration but don't start servo yet
                self.motor_manager.SetMode(motor_id, 0)
                self.motor_manager.SetAcceleration(motor_id, accel)

                # Read current position instead of moving to home
                current_pos_ticks = self.motor_manager.ReadPosition(motor_id)
                if current_pos_ticks is not None:
                    # Convert to radians and degrees for logging
                    servo_center = 2048
                    current_angle_rad = (current_pos_ticks - servo_center) / ticks_per_rev * 2.0 * math.pi
                    current_angle_deg = current_angle_rad * 180.0 / math.pi

                    # Store current position
                    self.camera_current_positions[motor_id] = current_pos_ticks

                    # Start servo and immediately command it to hold current position
                    self.motor_manager.StartServo(motor_id)
                    time.sleep(0.05)  # Brief delay to ensure servo is started
                    self.motor_manager.MoveTo(motor_id, current_pos_ticks, 100, accel)

                    self.node.get_logger().info(f"Camera motor {motor_id} holding at: {current_angle_deg:.1f} deg ({current_angle_rad:.3f} rad, {current_pos_ticks} ticks)")
                else:
                    self.node.get_logger().warn(f"Could not read current position for camera motor {motor_id}")

                time.sleep(0.1)  # Small delay to prevent bus congestion

            except Exception as e:
                self.node.get_logger().warn(f"Failed to initialize camera motor {motor_id}: {e}")

        self.node.get_logger().info("Camera motors initialized with torque holding at current positions")
    
    def send_velocity_command(self, motor_id: int, speed_raw: int):
        """Send velocity command to a motor"""
        # Scale speed to 25% to limit max velocity (does not affect odometry readings)
        speed_raw = int(speed_raw * 0.25)

        self.node.get_logger().info(f"send_velocity_command: motor_id={motor_id}, speed_raw={speed_raw}")

        try:
            if speed_raw == 0:
                # Zero velocity: set zero velocity, then stop motor
                self.node.get_logger().info(f"  Stopping motor {motor_id}")
                self.motor_manager.Rotate(motor_id, 0)
                time.sleep(0.01)
                self.motor_manager.StopServo(motor_id)
                
                # Remove from running motors
                if motor_id in self.running_motors:
                    del self.running_motors[motor_id]
            else:
                # Non-zero motor speed: set velocity (motor should already be started)
                # Check if motor needs to be restarted
                if motor_id not in self.running_motors:
                    self.node.get_logger().info(f"  Starting motor {motor_id}")
                    self.motor_manager.StartServo(motor_id)
                
                # Set the speed
                self.running_motors[motor_id] = speed_raw
                self.node.get_logger().info(f"  Setting motor {motor_id} speed to {speed_raw}")
                self.motor_manager.Rotate(motor_id, speed_raw)
                
        except Exception as e:
            self.node.get_logger().error(f"Exception in send_velocity_command for motor {motor_id}: {e}")
    
    def send_position_command(self, motor_id: int, pos_ticks: int, speed: int, accel: int):
        """Send position command to a motor"""
        self.motor_manager.MoveTo(motor_id, pos_ticks, speed, accel)
    
    def read_motor_state(self, motor_id: int):
        """Read motor position and velocity"""
        try:
            pos = self.motor_manager.ReadPosition(motor_id)
            speed_data = self.motor_manager.ReadSpeed(motor_id)

            if pos is not None and speed_data is not None:
                # ReadSpeed returns (speed, comm_result, error)
                # comm_result == 0 means success, error == 0 means no error
                if isinstance(speed_data, tuple) and len(speed_data) >= 3:
                    speed, comm_result, error = speed_data[0], speed_data[1], speed_data[2]
                    # Only use speed if communication was successful
                    if comm_result == 0 and error == 0:
                        return pos, speed
                    else:
                        # Communication error - return position but no velocity
                        return pos, 0
                else:
                    speed = speed_data[0] if isinstance(speed_data, tuple) else speed_data
                    return pos, speed
        except Exception:
            pass

        return None, None

    def read_motor_telemetry(self, motor_id: int) -> dict:
        """Read detailed telemetry from a motor (current, voltage, load, temperature)

        Returns dict with keys: current, voltage, load, temperature
        Values are None if read fails
        """
        telemetry = {
            'current': None,      # mA
            'voltage': None,      # V
            'load': None,         # %
            'temperature': None,  # C
        }

        try:
            telemetry['current'] = self.motor_manager.ReadCurrent(motor_id)
            telemetry['voltage'] = self.motor_manager.ReadVoltage(motor_id)
            telemetry['load'] = self.motor_manager.ReadLoad(motor_id)
            telemetry['temperature'] = self.motor_manager.ReadTemperature(motor_id)
        except Exception as e:
            self.node.get_logger().debug(f"Failed to read telemetry for motor {motor_id}: {e}")

        return telemetry