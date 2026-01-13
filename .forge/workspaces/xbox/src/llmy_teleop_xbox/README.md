# llmy_teleop_xbox

Xbox controller teleoperation for LLMy robot, providing intuitive control of the omnidirectional base, 6-DOF robotic arm, and pan-tilt camera head.

## Features

- **Right stick base control** - Forward/backward and rotation
- **Button-based arm control** - All 6 joints controlled via buttons and triggers
- **Button-based arm control** - Joints 3-6 with single press and hold-to-repeat
- **D-pad camera control** - Pan (up/down) and tilt (left/right)
- **Smooth acceleration** - Gradual velocity changes for base movement
- **Deadband filtering** - Prevents controller drift
- **Joint limit enforcement** - Prevents invalid camera positions

## Control Mapping

### 🏎️ Base Movement
- **Right Stick Forward/Back** - Linear movement (forward/backward)
- **Right Stick Left/Right** - Rotation (turn left/right)
- **Left Stick Left/Right** - Lateral movement (strafe left/right)

### 🦾 Arm Control

**Joint 1 (Base Rotation):**
- **RB (Right Bumper)** - Increase joint 1 (+)
- **LB (Left Bumper)** - Decrease joint 1 (-)

**Joint 2 (Shoulder):**
- **RT (Right Trigger)** - Increase joint 2 (+)
- **LT (Left Trigger)** - Decrease joint 2 (-)

**Joint 3 (Elbow):**
- **Y Button** - Increase joint 3 (+)
- **A Button** - Decrease joint 3 (-)

**Joint 4 (Wrist 1):**
- **B Button** - Increase joint 4 (+)
- **X Button** - Decrease joint 4 (-)

**Joint 5 (Wrist 2):**
- **Start Button** - Increase joint 5 (+)
- **Back Button** - Decrease joint 5 (-)

**Joint 6 (Wrist 3):**
- **Right Stick Click** - Increase joint 6 (+)
- **Left Stick Click** - Decrease joint 6 (-)

### 📷 Camera Control

- **D-pad Left** - Tilt down (-)
- **D-pad Right** - Tilt up (+)
- **D-pad Up** - Pan right (+)
- **D-pad Down** - Pan left (-)

**Note:** All buttons support both single press (small increment) and hold-to-repeat (continuous movement).

## Configuration

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cmd_vel_topic` | `/cmd_vel_xbox` | Base velocity command topic |
| `arm_cmd_topic` | `/arm_controller/commands` | Arm command topic |
| `head_cmd_topic` | `/head_controller/commands` | Head command topic |
| `linear_scale` | `0.2` | Forward/backward speed scaling |
| `angular_scale` | `0.5` | Rotation speed scaling |
| `arm_increment` | `0.1745` (10°) | Standard joint increment |
| `arm_increment_fast` | `0.1396` (8°) | Fast joint increment (joints 1-4) |
| `stick_deadband` | `0.1` | Analog stick deadband threshold |
| `trigger_threshold` | `0.3` | Trigger press threshold |
| `arm_rate` | `30.0` | Arm/head command rate (Hz) |
| `base_rate` | `50.0` | Base command rate (Hz) |
| `acceleration_limit` | `2.0` | Max acceleration for base (m/s²) |

### Joint Limits

Camera head has enforced joint limits:
- **camera_pan**: -3.14 to 3.14 rad (full rotation)
- **camera_tilt**: -1.57 to 0.86 rad (tilt range)

## Usage

### Hardware Setup

1. Connect Xbox controller via USB or wireless adapter
2. Verify controller detection:
```bash
ls /dev/input/js*
ros2 run joy joy_enumerate_devices
```

### Launch Teleoperation

```bash
# Launch with joy node included
ros2 launch llmy_teleop_xbox teleop_xbox.launch.py

# Or launch manually
ros2 run joy joy_node &
ros2 run llmy_teleop_xbox teleop_xbox.py
```

### Custom Parameters

```bash
# Faster base movement
ros2 run llmy_teleop_xbox teleop_xbox.py --ros-args \
  -p linear_scale:=0.4 \
  -p angular_scale:=0.8

# Larger arm increments
ros2 run llmy_teleop_xbox teleop_xbox.py --ros-args \
  -p arm_increment:=0.2618  # 15 degrees
```

### Monitoring

```bash
# Monitor base commands
ros2 topic echo /cmd_vel_xbox

# Monitor arm commands
ros2 topic echo /arm_controller/commands

# Monitor head commands
ros2 topic echo /head_controller/commands

# Check joystick input
ros2 topic echo /joy
```

## Integration

This package publishes to topics that are multiplexed by the main bringup system:

- **Base commands** → `/cmd_vel_xbox` → Multiplexed to `/omnidirectional_controller/cmd_vel_unstamped`
- **Arm commands** → `/arm_controller/commands` → Processed by arm controller
- **Head commands** → `/head_controller/commands` → Processed by head controller

The `twist_mux` in `llmy_bringup` handles command prioritization, with teleop having priority 10 (medium).

## Safety Features

- **Smooth acceleration** - Base velocity changes gradually to prevent jerky motion
- **Deadband filtering** - Ignores small stick movements from drift
- **Joint limits** - Camera joints clamped to safe ranges
- **Rate limiting** - Commands published at controlled frequency
- **Zero velocity on release** - Base stops when stick returns to center

## Troubleshooting

### Controller Not Detected

```bash
# Check if controller is connected
ls /dev/input/js*

# Test joy node
ros2 run joy joy_enumerate_devices
ros2 topic echo /joy

# Check permissions
sudo chmod 666 /dev/input/js0
```

### Robot Not Responding

```bash
# Verify topics
ros2 topic list | grep -E "(cmd_vel|commands)"

# Check twist_mux status
ros2 topic echo /omnidirectional_controller/cmd_vel_unstamped

# Verify controllers are active
ros2 control list_controllers

# Enable debug logging
ros2 run llmy_teleop_xbox teleop_xbox.py --ros-args --log-level debug
```

### Arm Moving Unexpectedly

- Check stick calibration with `ros2 topic echo /joy`
- Increase `stick_deadband` parameter
- Verify arm controller is receiving commands

### Base Not Moving Smoothly

- Adjust `linear_scale` and `angular_scale`
- Check `acceleration_limit` parameter
- Verify omnidirectional controller configuration

## Dependencies

- `rclpy` - ROS2 Python client library
- `sensor_msgs` - Joy messages
- `geometry_msgs` - Twist messages
- `std_msgs` - Float64MultiArray messages
- `joy` - ROS2 joystick driver package

## Technical Details

### Command Publishing

- **Base**: Published at 50Hz with smooth acceleration limiting
- **Arm/Head**: Published at 30Hz with incremental position updates
- **Joystick**: Processed on every Joy message (typically 100Hz)

### Coordinate Frames

- Base commands use robot frame (forward = +X, left = +Y, CCW = +Z)
- Arm/head commands are joint angles in radians
- Camera pan: positive = right, negative = left
- Camera tilt: positive = up, negative = down

## License

Part of the LLMy Robot project. See main repository for license information.
