# st3215_manager

Generic, config-driven ROS2 driver for FEETECH ST3215 servo motors. Define motor groups in YAML — no code changes needed to add motors or support a new robot.

## Quick Start

```bash
# Build
colcon build --packages-select st3215_manager

# Launch with default config
ros2 launch st3215_manager servo_manager.launch.py

# Launch with a custom config
ros2 launch st3215_manager servo_manager.launch.py \
  config_file:=/path/to/your/config.yaml
```

## Configuration

All configuration lives in a single YAML file. See `config/servo_manager.yaml` for the LLMy default and `config/so101_arm.yaml` for a standalone SO-101 arm example.

### Global Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port` | string | `/dev/ttyTHS1` | Serial port for the Feetech bus |
| `baud` | int | `1000000` | Baud rate |
| `ticks_per_rev` | int | `4096` | Encoder ticks per revolution |
| `telemetry_rate` | float | `50.0` | Joint state publish rate (Hz) |
| `joint_states_topic` | string | `/motor_manager/joint_states` | Topic for JointState output |
| `total_joints` | int | `0` | Size of JointState arrays. Set to `0` to auto-calculate from highest `state_index + 1` |
| `brake_method` | string | `torque_disable` | `torque_disable`, `velocity_ramp`, or `position_brake` |
| `velocity_ramp_time` | float | `0.1` | Ramp time for velocity_ramp brake (s) |
| `brake_acceleration` | int | `100` | Acceleration for position_brake method |
| `motor_telemetry_enable` | bool | `false` | Publish per-motor current/voltage/load/temperature |
| `motor_telemetry_rate` | float | `10.0` | Per-motor telemetry rate (Hz) |
| `test_on_startup` | bool | `false` | Run motor test sequences on startup |
| `group_names` | string[] | `["base"]` | List of motor group names to load |

### Motor Group Parameters

Each entry in `group_names` becomes a parameter prefix. For a group named `arm`, the parameters are `arm.enable`, `arm.mode`, etc.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `<name>.enable` | bool | `true` | Enable this group |
| `<name>.mode` | string | `velocity` | `velocity` or `position` |
| `<name>.topic` | string | `/motor_manager/<name>_cmd` | ROS topic to subscribe for commands |
| `<name>.motor_ids` | int[] | `[]` | ST3215 motor IDs on the bus |
| `<name>.joint_names` | string[] | `[]` | Joint names for JointState messages |
| `<name>.direction` | int[] | `[]` | Per-motor direction multiplier (`1` or `-1`) |
| `<name>.command_indices` | int[] | `[]` | Which indices in the incoming `Float64MultiArray` to read from |
| `<name>.state_indices` | int[] | `[]` | Which indices in the outgoing `JointState` arrays to write to |
| `<name>.speed_scale` | float | `1.0` | Speed multiplier applied to incoming commands |
| `<name>.accel` | int[] | `[]` | Per-motor acceleration (0-255, lower = smoother) |
| `<name>.unwrap_position` | bool | `false` | Track continuous rotation (needed for wheels/odometry) |
| `<name>.position_speed` | int | `200` | Base speed for position-mode MoveTo commands |
| `<name>.position_accel` | int | `200` | Base accel for position-mode MoveTo commands |

All per-motor arrays (`motor_ids`, `joint_names`, `direction`, `command_indices`, `state_indices`, `accel`) must have the same length.

### Index Mapping

`command_indices` and `state_indices` decouple the motor ordering from message layout:

- **`command_indices`**: Motor `i` reads its command from `msg.data[command_indices[i]]`. This allows the incoming array to contain joints from other packages, or to reorder joints without changing motor wiring.

- **`state_indices`**: Motor `i` writes its state to `JointState.position[state_indices[i]]`. This lets multiple packages publish to the same topic with non-overlapping indices.

#### Example: Shared topic with another motor driver

If a DSSM210 driver handles wheels at indices 0-1, and this package handles the arm at indices 2-7:

```yaml
total_joints: 8   # wheels (0,1) + arm (2-7)

arm:
  motor_ids: [3, 4, 5, 6, 7, 8]
  command_indices: [0, 1, 2, 3, 4, 5]   # arm topic only has arm joints
  state_indices: [2, 3, 4, 5, 6, 7]     # slots 0-1 reserved for wheels
```

The DSSM210 package would publish wheels at indices 0-1 on the same `joint_states_topic`. Consumers match by joint name, and the fixed indices ensure a predictable array layout.

## Topics

### Subscribed (one per enabled group)

Configured via `<group>.topic`. Expects `std_msgs/Float64MultiArray`.

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `<joint_states_topic>` | `sensor_msgs/JointState` | Joint positions and velocities |
| `/motor_telemetry/<joint_name>/current` | `std_msgs/Float32` | Motor current (mA) |
| `/motor_telemetry/<joint_name>/voltage` | `std_msgs/Float32` | Motor voltage (V) |
| `/motor_telemetry/<joint_name>/load` | `std_msgs/Float32` | Motor load (%) |
| `/motor_telemetry/<joint_name>/temperature` | `std_msgs/Int32` | Motor temperature (C) |

Motor telemetry topics are only created when `motor_telemetry_enable: true`.

## Adding a New Robot

1. Copy `config/so101_arm.yaml` as a starting point
2. Set `port` to your serial device
3. Define your motor groups with IDs, joint names, and indices
4. Launch with `config_file:=/path/to/your/config.yaml`

No code changes required.
