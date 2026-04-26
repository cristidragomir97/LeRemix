# ddsm210_manager

ROS2 driver for Waveshare DDSM210 direct drive servo motors. Includes a from-scratch serial protocol implementation — no external motor library required.

## Quick Start

```bash
colcon build --packages-select ddsm210_manager

ros2 launch ddsm210_manager ddsm210.launch.py

# Custom config
ros2 launch ddsm210_manager ddsm210.launch.py config_file:=/path/to/config.yaml
```

## Hardware

- **Motor**: Waveshare DDSM210 (integrated brushless motor + encoder + FOC driver)
- **Interface**: UART (TX, RX, GND, VCC)
- **Baud**: 115200, 8N1
- **Voltage**: 11-22V DC
- **Speed**: -210..210 RPM
- **Encoder**: 4096 resolution (relative), 65536 ticks/rev (position feedback)

## Protocol

The driver implements the full DDSM210 UART protocol (10-byte frames, CRC-8/MAXIM):

| Protocol | Command | Description |
|----------|---------|-------------|
| 1 (0x64) | Drive | Set velocity/position, get speed+position+temp feedback |
| 2 (0x74) | Odometry | Get mileage laps (continuous) + absolute position |
| 3 (0xA0) | Set Mode | Open loop (0x00), velocity loop (0x02), position loop (0x03) |
| 4 | Set ID | Change motor ID (send 5x, one motor on bus) |
| 5 (0x75) | Query Mode | Get current operating mode |

## Wiring: Multi-Motor Setup

The DDSM210 uses separate TX/RX UART lines. When connecting multiple motors, their TX outputs will conflict if wired together directly. Two options:

**Option 1 — Separate ports (recommended for direct wiring):**
Each motor gets its own USB-to-UART adapter. Configure `ports` with one entry per motor:
```yaml
ports: ["/dev/ttyUSB0", "/dev/ttyUSB1"]
motor_ids: [1, 2]
```

**Option 2 — Shared bus (requires AND gates):**
Use a bus board like the Waveshare DDSM Driver HAT, which combines motor TX lines through AND gates (SN74LVC1G08). Configure a single port:
```yaml
ports: ["/dev/ttyUSB0"]
motor_ids: [1, 2]
```

## Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `ports` | string[] | `["/dev/ttyUSB0"]` | Serial port(s) — one per motor, or one shared by all |
| `baud` | int | `115200` | Baud rate |
| `command_rate` | float | `50.0` | Command send rate (Hz) |
| `telemetry_rate` | float | `50.0` | JointState publish rate (Hz) |
| `joint_states_topic` | string | `/motor_manager/joint_states` | JointState output topic |
| `command_topic` | string | `/motor_manager/base_cmd` | Float64MultiArray command input topic |
| `total_joints` | int | `0` | JointState array size (0 = auto from state_indices) |
| `mode` | string | `velocity` | `velocity` or `position` |
| `accel_time` | int | `0` | Acceleration per 1 RPM in 0.1ms units |
| `motor_ids` | int[] | `[1, 2]` | Motor IDs on the bus |
| `joint_names` | string[] | `[right_wheel_joint, left_wheel_joint]` | Joint names for JointState |
| `direction` | int[] | `[1, -1]` | Per-motor direction multiplier |
| `command_indices` | int[] | `[0, 1]` | Which indices to read from incoming command array |
| `state_indices` | int[] | `[0, 1]` | Which indices to write in JointState arrays |
| `speed_scale` | float | `1.0` | Multiplier applied to incoming velocity commands |

## Sharing the JointState Topic

This package publishes JointState to the same topic as the `st3215_servo_manager`. Each package fills in its own `state_indices` slots:

```
DDSM210 (wheels):     state_indices: [0, 1]
ST3215 (arm+camera):  state_indices: [2, 3, 4, 5, 6, 7, 8]
```

Both publish to `/motor_manager/joint_states`. Consumers match by joint name.

## Low-Level API

The `ddsm210.py` module can be used standalone without ROS:

```python
from ddsm210_manager.ddsm210 import DDSM210, MODE_VELOCITY

motor = DDSM210("/dev/ttyUSB0")
motor.set_mode(1, MODE_VELOCITY)
motor.set_velocity(1, 500)   # 50 RPM
motor.brake(1)
odom = motor.get_odometry(1) # {'mileage_laps': ..., 'position': ..., 'error_code': ...}
motor.close()
```

## Interactive Terminal

Test motors without ROS using the built-in terminal:

```bash
# Single port
python3 scripts/test_motor.py --port /dev/ttyUSB0

# Two ports (one per motor)
python3 scripts/test_motor.py --port /dev/ttyUSB0 --port /dev/ttyUSB1
```

Commands: `scan`, `ping`, `speed`, `brake`, `odom`, `mode`, `pos`, `set_id`, `query_id`, `ports`, `quit`.
