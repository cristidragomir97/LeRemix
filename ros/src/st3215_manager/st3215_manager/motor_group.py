#!/usr/bin/env python3
"""MotorGroup dataclass — the central abstraction for the ST3215 servo manager."""

from dataclasses import dataclass


@dataclass
class MotorGroup:
    """Describes one group of ST3215 motors sharing a control mode and ROS topic."""

    name: str
    enable: bool
    mode: str                   # "velocity" or "position"
    topic: str                  # ROS topic to subscribe on
    motor_ids: list[int]
    joint_names: list[str]
    direction: list[int]        # per-motor direction multiplier (1 or -1)
    command_indices: list[int]  # which indices in incoming Float64MultiArray to read
    state_indices: list[int]    # which indices in outgoing JointState to write
    speed_scale: float          # applied to incoming commands before sending
    accel: list[int]            # per-motor acceleration (0-255)
    unwrap_position: bool       # track continuous rotation (for wheels)
    position_speed: int = 200   # base speed for position-mode MoveTo
    position_accel: int = 200   # base accel for position-mode MoveTo

    def __post_init__(self):
        n = len(self.motor_ids)
        for field_name in ("joint_names", "direction", "command_indices",
                           "state_indices", "accel"):
            field_val = getattr(self, field_name)
            if len(field_val) != n:
                raise ValueError(
                    f"Group '{self.name}': {field_name} has {len(field_val)} "
                    f"entries, expected {n}")
        if self.mode not in ("velocity", "position"):
            raise ValueError(
                f"Group '{self.name}': invalid mode '{self.mode}', "
                f"must be 'velocity' or 'position'")

    @property
    def motor_count(self) -> int:
        return len(self.motor_ids)

    def joint_name(self, motor_id: int) -> str:
        idx = self.motor_ids.index(motor_id)
        return self.joint_names[idx]

    def motor_direction(self, motor_id: int) -> int:
        idx = self.motor_ids.index(motor_id)
        return self.direction[idx]

    def motor_accel(self, motor_id: int) -> int:
        idx = self.motor_ids.index(motor_id)
        return self.accel[idx]
