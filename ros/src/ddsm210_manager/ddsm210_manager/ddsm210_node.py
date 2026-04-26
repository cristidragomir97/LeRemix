#!/usr/bin/env python3
"""ROS2 node for Waveshare DDSM210 direct drive servo motors.

Config-driven: define motor IDs, joint names, direction multipliers,
and index mapping in YAML. Each motor gets its own serial port to avoid
TX line conflicts on the UART bus.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from .ddsm210 import (
    DDSM210, MODE_VELOCITY, MODE_POSITION,
    MIN_SPEED_RAW, MAX_SPEED_RAW, ENCODER_TICKS,
)


class DDSM210Node(Node):
    def __init__(self):
        super().__init__('ddsm210_node')
        self.get_logger().info("=== DDSM210 Motor Manager Starting ===")

        self._declare_and_load_params()
        self._connect()
        self._initialize_motors()
        self._setup_ros()
        self._log_startup()

    # -- parameter loading ----------------------------------------------------

    def _declare_and_load_params(self):
        self.declare_parameter("ports", ["/dev/ttyUSB0"])
        self.declare_parameter("baud", 115200)
        self.declare_parameter("command_rate", 50.0)
        self.declare_parameter("telemetry_rate", 50.0)
        self.declare_parameter("joint_states_topic", "/motor_manager/joint_states")
        self.declare_parameter("command_topic", "/motor_manager/base_cmd")
        self.declare_parameter("total_joints", 0)
        self.declare_parameter("mode", "velocity")
        self.declare_parameter("accel_time", 0)
        self.declare_parameter("max_rpm", 210)  # hardware max is 210 RPM

        self.declare_parameter("motor_ids", [1, 2])
        self.declare_parameter("joint_names", ["right_wheel_joint", "left_wheel_joint"])
        self.declare_parameter("direction", [1, -1])
        self.declare_parameter("command_indices", [0, 1])
        self.declare_parameter("state_indices", [0, 1])
        self.declare_parameter("speed_scale", 1.0)

        get = self.get_parameter
        self.ports = list(get("ports").get_parameter_value().string_array_value)
        self.baud = get("baud").get_parameter_value().integer_value
        self.command_rate = get("command_rate").get_parameter_value().double_value
        self.telemetry_rate = get("telemetry_rate").get_parameter_value().double_value
        self.joint_states_topic = get("joint_states_topic").get_parameter_value().string_value
        self.command_topic = get("command_topic").get_parameter_value().string_value
        self.total_joints = get("total_joints").get_parameter_value().integer_value
        self.mode = get("mode").get_parameter_value().string_value
        self.accel_time = get("accel_time").get_parameter_value().integer_value
        self.max_rpm = get("max_rpm").get_parameter_value().integer_value

        self.motor_ids = list(get("motor_ids").get_parameter_value().integer_array_value)
        self.joint_names = list(get("joint_names").get_parameter_value().string_array_value)
        self.direction = list(get("direction").get_parameter_value().integer_array_value)
        self.command_indices = list(get("command_indices").get_parameter_value().integer_array_value)
        self.state_indices = list(get("state_indices").get_parameter_value().integer_array_value)
        self.speed_scale = get("speed_scale").get_parameter_value().double_value

        n = len(self.motor_ids)
        for field, name in [(self.joint_names, "joint_names"),
                            (self.direction, "direction"),
                            (self.command_indices, "command_indices"),
                            (self.state_indices, "state_indices")]:
            if len(field) != n:
                raise ValueError(f"{name} has {len(field)} entries, expected {n}")

        # Ports: one per motor, or one shared by all
        if len(self.ports) == 1:
            self.ports = self.ports * n  # expand single port to all motors
        elif len(self.ports) != n:
            raise ValueError(
                f"ports has {len(self.ports)} entries, expected 1 or {n}")

        # Auto-calculate total_joints if not set
        if self.total_joints <= 0 and self.state_indices:
            self.total_joints = max(self.state_indices) + 1

        self.motor_count = n

    # -- connection and init --------------------------------------------------

    def _connect(self):
        # One DDSM210 driver per unique port
        self._port_drivers: dict[str, DDSM210] = {}
        self._motor_driver: dict[int, DDSM210] = {}

        for i, mid in enumerate(self.motor_ids):
            port = self.ports[i]
            if port not in self._port_drivers:
                driver = DDSM210(port, self.baud)
                self._port_drivers[port] = driver
                self.get_logger().info(f"Connected to {port} @ {self.baud}")
            self._motor_driver[mid] = self._port_drivers[port]

        # Verify connectivity
        mode_names = {0x00: "open_loop", 0x02: "velocity", 0x03: "position"}
        for i, mid in enumerate(self.motor_ids):
            mode = self._motor_driver[mid].get_mode(mid)
            if mode is not None:
                self.get_logger().info(
                    f"  Motor {mid} ({self.ports[i]}): ONLINE "
                    f"(mode={mode_names.get(mode, hex(mode))})")
            else:
                self.get_logger().warn(
                    f"  Motor {mid} ({self.ports[i]}): no response")

    def _initialize_motors(self):
        mode_val = MODE_VELOCITY if self.mode == "velocity" else MODE_POSITION
        mode_name = "velocity" if self.mode == "velocity" else "position"

        self.get_logger().info(f"Setting motors to {mode_name} loop mode...")
        for mid in self.motor_ids:
            resp = self._motor_driver[mid].set_mode(mid, mode_val)
            if resp is not None:
                self.get_logger().info(f"  Motor {mid}: mode set OK")
            else:
                self.get_logger().warn(f"  Motor {mid}: mode set got no response")

        self._last_commands: list[float] = []

    # -- ROS setup ------------------------------------------------------------

    def _setup_ros(self):
        cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT)

        self._cmd_sub = self.create_subscription(
            Float64MultiArray, self.command_topic,
            self._handle_command, cmd_qos)
        self.get_logger().info(f"  Subscriber: {self.command_topic}")

        self._state_pub = self.create_publisher(
            JointState, self.joint_states_topic, 10)
        self.get_logger().info(f"  Publisher: {self.joint_states_topic}")

        self._telemetry_timer = self.create_timer(
            1.0 / self.telemetry_rate, self._publish_telemetry)
        self.get_logger().info(f"  Telemetry timer: {self.telemetry_rate} Hz")

    # -- command handling -----------------------------------------------------

    def _handle_command(self, msg: Float64MultiArray):
        self._cmd_rx_count = getattr(self, "_cmd_rx_count", 0) + 1
        max_idx = max(self.command_indices)
        if len(msg.data) <= max_idx:
            self.get_logger().warn(
                f"Command too short: need index {max_idx}, got length {len(msg.data)}")
            return

        values = [msg.data[ci] for ci in self.command_indices]

        # Log raw values every Nth message, and every time they change.
        changed = (len(self._last_commands) != len(values)
                   or any(abs(a - b) >= 1e-6
                          for a, b in zip(values, self._last_commands)))
        if changed or self._cmd_rx_count % 100 == 1:
            self.get_logger().info(
                f"cmd rx #{self._cmd_rx_count}: raw={list(msg.data)} "
                f"picked={values} changed={changed}")

        # Skip if unchanged
        if not changed:
            return
        self._last_commands = list(values)

        for i, mid in enumerate(self.motor_ids):
            value = values[i] * self.direction[i]
            driver = self._motor_driver[mid]

            if self.mode == "velocity":
                rpm = value * self.speed_scale * 60.0 / (2.0 * math.pi)
                self.get_logger().info(
                    f"  -> motor {mid}: {value:+.3f} rad/s (dir={self.direction[i]}) => {rpm:+.1f} RPM")
                self._send_velocity(driver, mid, value)
            else:
                self.get_logger().info(
                    f"  -> motor {mid}: {value:+.3f} rad (dir={self.direction[i]})")
                self._send_position(driver, mid, value)

    def _send_velocity(self, driver: DDSM210, motor_id: int, rad_per_sec: float):
        """Convert rad/s to DDSM210 speed units (0.1 RPM) and send."""
        rpm = rad_per_sec * self.speed_scale * 60.0 / (2.0 * math.pi)
        rpm_x10 = int(rpm * 10.0)
        # Cap at the user-configured max (converted to 0.1-RPM units), within hardware bounds.
        user_cap = min(abs(self.max_rpm) * 10, MAX_SPEED_RAW)
        clamped = max(-user_cap, min(user_cap, rpm_x10))
        resp = driver.set_velocity(motor_id, clamped, self.accel_time)
        if resp is None:
            self.get_logger().warn(
                f"     motor {motor_id}: no serial response to set_velocity({clamped})")
        else:
            self.get_logger().debug(
                f"     motor {motor_id}: wrote {clamped} (feedback={resp})")

    def _send_position(self, driver: DDSM210, motor_id: int, angle_rad: float):
        """Convert radians to DDSM210 position units (0-32767) and send."""
        angle = angle_rad % (2.0 * math.pi)
        pos = int(angle / (2.0 * math.pi) * 32767)
        pos = max(0, min(32767, pos))
        driver.set_position(motor_id, pos)

    # -- telemetry ------------------------------------------------------------

    def _publish_telemetry(self):
        names = [""] * self.total_joints
        positions = [0.0] * self.total_joints
        velocities = [0.0] * self.total_joints

        for i, mid in enumerate(self.motor_ids):
            state_idx = self.state_indices[i]
            direction = self.direction[i]
            driver = self._motor_driver[mid]

            odom = driver.get_odometry(mid)
            if odom is None:
                continue

            mileage = odom['mileage_laps']
            enc_pos = odom['position']  # 0-65535 = 0-360 deg

            fractional_rad = (enc_pos / ENCODER_TICKS) * 2.0 * math.pi
            total_rad = mileage * 2.0 * math.pi + fractional_rad

            names[state_idx] = self.joint_names[i]
            positions[state_idx] = total_rad * direction
            velocities[state_idx] = 0.0

        if not any(n for n in names):
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        msg.velocity = velocities
        self._state_pub.publish(msg)

    # -- startup log ----------------------------------------------------------

    def _log_startup(self):
        self.get_logger().info("=== DDSM210 Motor Manager Ready ===")
        unique_ports = list(self._port_drivers.keys())
        self.get_logger().info(f"  Serial ports: {unique_ports}")
        for i, mid in enumerate(self.motor_ids):
            self.get_logger().info(
                f"  Motor {mid}: {self.joint_names[i]} | "
                f"port={self.ports[i]} | "
                f"dir={self.direction[i]} | "
                f"cmd_idx={self.command_indices[i]} | "
                f"state_idx={self.state_indices[i]}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = DDSM210Node()
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
