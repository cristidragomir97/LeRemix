# LLMy Servo Manager

Servo control system for the LLMy Robot.

- **Multi-Motor Support**: Controls locomotion, arm, and head motor groups independently
- **Real-time Telemetry**: Publishes motor states at configurable rates
- **Robust Error Handling**: Comprehensive connectivity testing and graceful error recovery


## Architecture

### Core Modules

- **`servo_manager_node.py`**: Main ROS2 node orchestrating all components
- **`config.py`**: Configuration management and parameter validation
- **`motor_manager.py`**: Low-level motor communication and control
- **`brake_system.py`**: Safe motor braking with multiple strategies
- **`command_handlers.py`**: ROS message processing and motor command conversion
- **`telemetry.py`**: Motor state publishing and monitoring

## Configuration

The servo manager is configured via ROS2 parameters, typically loaded from `config/servo_manager.yaml`:

### Communication Parameters
```yaml
port: "/dev/ttyTHS1"          # Serial port for motor communication
baud: 1000000                 # Baud rate (default: 1 Mbps)
ticks_per_rev: 4096           # Encoder ticks per revolution
telemetry_rate: 30.0          # Telemetry publishing rate (Hz)
```

### Motor Group Configuration
```yaml
# Enable/disable motor groups
loc_enable: true              # Locomotion motors
arm_enable: true              # Arm motors  
head_enable: true             # Head motors

# Motor IDs for each group
loc_ids: [1, 2, 3]           # Locomotion motor IDs
arm_ids: [4, 5, 6, 7, 8, 9]  # Arm motor IDs
head_ids: [10, 11]           # Head motor IDs

# Acceleration settings (per motor)
loc_accel: [15, 15, 15]
arm_accel: [20, 20, 15, 15, 25, 25]
head_accel: [15, 15]

# Speed scaling factors (0.0 to 2.0)
loc_speed_scale: 0.2         # Conservative for locomotion
arm_speed_scale: 1.0         # Normal for arm
head_speed_scale: 1.0        # Normal for head
```

### Braking System Configuration
```yaml
# Braking method: "torque_disable", "velocity_ramp", "position_brake"
brake_method: "torque_disable"  # Gentlest method (recommended)
velocity_ramp_time: 0.5         # Ramp down time for velocity_ramp (seconds)
brake_acceleration: 100         # Acceleration for position_brake method
```

## ROS2 Interface

### Subscribed Topics

- **`/motor_manager/base_cmd`** (`Float64MultiArray`)
  - Velocity commands for locomotion motors (rad/s)
  - Array size must match number of locomotion motors

- **`/motor_manager/arm_cmd`** (`Float64MultiArray`)  
  - Position commands for arm motors (radians)
  - Array size must match number of arm motors

- **`/motor_manager/head_cmd`** (`Float64MultiArray`)
  - Position commands for head motors (radians)
  - Array size must match number of head motors

### Published Topics

- **`/motor_manager/joint_states`** (`JointState`)
  - Current motor positions and velocities
  - Published at configured telemetry rate
  - Contains all enabled motors with naming: `motor_{id}`

## Usage

### Basic Launch
```bash
ros2 launch llmy_servo_manager servo_manager.launch.py
```

### With Custom Configuration
```bash
ros2 launch llmy_servo_manager servo_manager.launch.py \
    config_file:=custom_servo_config.yaml
```


## Monitoring and Diagnostics

### View Motor States
```bash
ros2 topic echo /motor_manager/joint_states
```

### Check Node Status
```bash
ros2 node info /servo_manager_node
```

### Monitor Motor Connectivity
The node automatically tests motor connectivity on startup and logs detailed status information.

## Troubleshooting

### Common Issues

1. **Serial Port Permission Denied**
   ```bash
   sudo usermod -a -G dialout $USER
   # Then logout and login again
   ```

2. **Motors Not Responding**
   - Check physical connections
   - Verify motor IDs in configuration
   - Ensure proper power supply
   - Check serial port settings


### Debug Mode
Enable debug logging to see detailed brake operations:
```bash
ros2 run llmy_servo_manager servo_manager_node --ros-args --log-level DEBUG
```
