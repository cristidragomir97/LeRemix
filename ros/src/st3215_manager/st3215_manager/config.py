#!/usr/bin/env python3
"""Configuration loader for the ST3215 servo manager.

Reads global parameters and a list of motor groups from ROS2 dotted parameters.
"""

from rclpy.node import Node
from rclpy.parameter import Parameter
from .motor_group import MotorGroup


class ServoManagerConfig:
    """Loads and validates all servo manager parameters."""

    def __init__(self, node: Node):
        self.node = node
        self._declare_global_parameters()
        self._load_global_parameters()
        self._load_groups()
        self._validate()

    # -- global params --------------------------------------------------------

    def _declare_global_parameters(self):
        self.node.declare_parameter("port", "/dev/ttyTHS1")
        self.node.declare_parameter("baud", 1000000)
        self.node.declare_parameter("ticks_per_rev", 4096)
        self.node.declare_parameter("telemetry_rate", 50.0)
        self.node.declare_parameter("joint_states_topic", "/motor_manager/joint_states")
        self.node.declare_parameter("total_joints", 0)
        self.node.declare_parameter("brake_method", "torque_disable")
        self.node.declare_parameter("velocity_ramp_time", 0.1)
        self.node.declare_parameter("brake_acceleration", 100)
        self.node.declare_parameter("motor_telemetry_enable", False)
        self.node.declare_parameter("motor_telemetry_rate", 10.0)
        self.node.declare_parameter("test_on_startup", False)
        self.node.declare_parameter("group_names", ["base"])

    def _load_global_parameters(self):
        get = self.node.get_parameter
        self.port = get("port").get_parameter_value().string_value
        self.baud = get("baud").get_parameter_value().integer_value
        self.ticks_per_rev = get("ticks_per_rev").get_parameter_value().integer_value
        self.telemetry_rate = get("telemetry_rate").get_parameter_value().double_value
        self.joint_states_topic = get("joint_states_topic").get_parameter_value().string_value
        self.total_joints = get("total_joints").get_parameter_value().integer_value
        self.brake_method = get("brake_method").get_parameter_value().string_value
        self.velocity_ramp_time = get("velocity_ramp_time").get_parameter_value().double_value
        self.brake_acceleration = get("brake_acceleration").get_parameter_value().integer_value
        self.motor_telemetry_enable = get("motor_telemetry_enable").get_parameter_value().bool_value
        self.motor_telemetry_rate = get("motor_telemetry_rate").get_parameter_value().double_value
        self.test_on_startup = get("test_on_startup").get_parameter_value().bool_value
        self.group_names = list(get("group_names").get_parameter_value().string_array_value)

    # -- per-group params -----------------------------------------------------

    def _load_groups(self):
        self.groups: list[MotorGroup] = []

        for name in self.group_names:
            p = name  # prefix
            self.node.declare_parameter(f"{p}.enable", True)
            self.node.declare_parameter(f"{p}.mode", "velocity")
            self.node.declare_parameter(f"{p}.topic", f"/motor_manager/{name}_cmd")
            self.node.declare_parameter(f"{p}.motor_ids", Parameter.Type.INTEGER_ARRAY)
            self.node.declare_parameter(f"{p}.joint_names", Parameter.Type.STRING_ARRAY)
            self.node.declare_parameter(f"{p}.direction", Parameter.Type.INTEGER_ARRAY)
            self.node.declare_parameter(f"{p}.command_indices", Parameter.Type.INTEGER_ARRAY)
            self.node.declare_parameter(f"{p}.state_indices", Parameter.Type.INTEGER_ARRAY)
            self.node.declare_parameter(f"{p}.speed_scale", 1.0)
            self.node.declare_parameter(f"{p}.accel", Parameter.Type.INTEGER_ARRAY)
            self.node.declare_parameter(f"{p}.unwrap_position", False)
            self.node.declare_parameter(f"{p}.position_speed", 200)
            self.node.declare_parameter(f"{p}.position_accel", 200)

            get = self.node.get_parameter
            group = MotorGroup(
                name=name,
                enable=get(f"{p}.enable").get_parameter_value().bool_value,
                mode=get(f"{p}.mode").get_parameter_value().string_value,
                topic=get(f"{p}.topic").get_parameter_value().string_value,
                motor_ids=list(get(f"{p}.motor_ids").get_parameter_value().integer_array_value),
                joint_names=list(get(f"{p}.joint_names").get_parameter_value().string_array_value),
                direction=list(get(f"{p}.direction").get_parameter_value().integer_array_value),
                command_indices=list(get(f"{p}.command_indices").get_parameter_value().integer_array_value),
                state_indices=list(get(f"{p}.state_indices").get_parameter_value().integer_array_value),
                speed_scale=get(f"{p}.speed_scale").get_parameter_value().double_value,
                accel=list(get(f"{p}.accel").get_parameter_value().integer_array_value),
                unwrap_position=get(f"{p}.unwrap_position").get_parameter_value().bool_value,
                position_speed=get(f"{p}.position_speed").get_parameter_value().integer_value,
                position_accel=get(f"{p}.position_accel").get_parameter_value().integer_value,
            )
            self.groups.append(group)

    # -- validation -----------------------------------------------------------

    def _validate(self):
        valid_brake_methods = ("torque_disable", "velocity_ramp", "position_brake")
        if self.brake_method not in valid_brake_methods:
            self.node.get_logger().warn(
                f"Invalid brake method '{self.brake_method}', using 'torque_disable'")
            self.brake_method = "torque_disable"

        if self.telemetry_rate <= 0:
            self.node.get_logger().warn("Invalid telemetry rate, using 50.0 Hz")
            self.telemetry_rate = 50.0

        if self.motor_telemetry_rate <= 0:
            self.node.get_logger().warn("Invalid motor telemetry rate, using 10.0 Hz")
            self.motor_telemetry_rate = 10.0

        # check for duplicate motor IDs across groups
        all_ids: list[int] = []
        for group in self.groups:
            for mid in group.motor_ids:
                if mid in all_ids:
                    raise ValueError(
                        f"Motor ID {mid} appears in multiple groups — IDs must be unique")
                all_ids.append(mid)

        # check for duplicate state_indices across groups
        all_state_indices: list[int] = []
        for group in self.groups:
            if not group.enable:
                continue
            for si in group.state_indices:
                if si in all_state_indices:
                    raise ValueError(
                        f"state_index {si} used by multiple motors — indices must be unique")
                all_state_indices.append(si)

        # auto-calculate total_joints if not set
        if self.total_joints <= 0 and all_state_indices:
            self.total_joints = max(all_state_indices) + 1
            self.node.get_logger().info(
                f"total_joints auto-calculated as {self.total_joints}")

        # validate state_indices fit within total_joints
        for si in all_state_indices:
            if si >= self.total_joints:
                raise ValueError(
                    f"state_index {si} >= total_joints ({self.total_joints})")

    # -- helpers --------------------------------------------------------------

    def get_enabled_groups(self) -> list[MotorGroup]:
        return [g for g in self.groups if g.enable]

    def get_all_enabled_motor_ids(self) -> list[int]:
        ids: list[int] = []
        for g in self.get_enabled_groups():
            ids.extend(g.motor_ids)
        return ids

    def log_configuration(self):
        log = self.node.get_logger().info
        log("Configuration loaded:")
        log(f"  Serial: {self.port} @ {self.baud}")
        log(f"  Ticks/rev: {self.ticks_per_rev}")
        log(f"  Telemetry: {self.telemetry_rate} Hz")
        log(f"  Joint states topic: {self.joint_states_topic}")
        log(f"  Total joints: {self.total_joints}")
        log(f"  Brake method: {self.brake_method}")
        log(f"  Test on startup: {self.test_on_startup}")
        log(f"  Motor telemetry: {'ON' if self.motor_telemetry_enable else 'OFF'}"
            f" ({self.motor_telemetry_rate} Hz)")
        log(f"  Groups ({len(self.groups)}):")
        for g in self.groups:
            status = "ENABLED" if g.enable else "DISABLED"
            log(f"    [{g.name}] {status} | mode={g.mode} | "
                f"motors={g.motor_ids} | cmd_idx={g.command_indices} | "
                f"state_idx={g.state_indices} | topic={g.topic}")
