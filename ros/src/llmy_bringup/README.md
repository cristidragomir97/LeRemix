# llmy_bringup

The LLMy Bringup package provides modular launch files that coordinate all LLMy subsystems to bring up the complete robot. The system is organized into three main subsystems for flexible deployment and debugging.

## Modular Architecture

The bringup system is organized into three main categories:

### 1. **Sensors** (`sensors.launch.py`)
- **Cameras**: RealSense RGB-D head camera and USB wrist camera
- **IMU**: Orientation and motion sensing
- **RPLidar**: 2D laser scanner (optional)

### 2. **Motion** (`motion.launch.py`) 
- **Servo Manager**: Low-level motor control via serial communication
- **ROS2 Control Stack**: Robot state publisher, controller manager, and spawners
- **Teleop**: Xbox controller interface
- **Command Velocity Multiplexing**: twist_mux for multiple control sources

### 3. **Communication** (`communication.launch.py`)
- **Rosbridge Server**: WebSocket interface for web applications
- **Rosboard**: Web-based dashboard for monitoring and visualization
- **Image Compression**: Bandwidth optimization for camera feeds
- **Depth-to-Scan**: Convert depth images to laser scan format

## Launch Files

### `bringup_robot.launch.py` (Main Launcher)

Coordinates all three subsystems with high-level enable/disable flags.

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sensors` | `true` | Enable all sensor systems |
| `use_motion` | `true` | Enable motion and control systems |
| `use_communication` | `true` | Enable communication and web interfaces |

### `sensors.launch.py`

| Argument | Default | Description |
|----------|---------|-------------|
| `use_camera` | `true` | Enable camera system (RealSense + wrist cam) |
| `use_imu` | `true` | Enable IMU sensor |
| `use_rplidar` | `false` | Enable RPLidar C1 |

### `motion.launch.py`

| Argument | Default | Description |
|----------|---------|-------------|
| `use_base_systems` | `true` | Enable servo manager (motor control) |
| `use_control_stack` | `true` | Enable robot state publisher and controllers |
| `use_xbox` | `true` | Enable Xbox controller teleoperation |
| `servo_port` | `/dev/ttyTHS1` | Serial port for servo communication |
| `servo_baud` | `1000000` | Baudrate for servo communication |

### `communication.launch.py`

| Argument | Default | Description |
|----------|---------|-------------|
| `use_rosbridge` | `true` | Enable rosbridge server for web interfaces |
| `use_rosboard` | `true` | Enable rosboard web dashboard |
| `use_image_compression` | `true` | Enable image compression and depth-to-scan |

## Usage Examples

### Full System Startup

```bash
# Launch complete robot with all systems
ros2 launch llmy_bringup bringup_robot.launch.py
```

### Modular System Startup

```bash
# Launch only sensors (cameras, IMU, optional LiDAR)
ros2 launch llmy_bringup sensors.launch.py

# Launch only motion systems (servos, control, teleop)
ros2 launch llmy_control motion.launch.py

# Launch only communication (web interfaces, compression)
ros2 launch llmy_bringup communication.launch.py
```

### Custom Configurations

```bash
# Launch without communication systems
ros2 launch llmy_bringup bringup_robot.launch.py use_communication:=false

# Launch with RPLidar enabled
ros2 launch llmy_bringup sensors.launch.py use_rplidar:=true

# Launch motion without Xbox controller
ros2 launch llmy_control motion.launch.py use_xbox:=false

# Launch sensors without cameras (IMU only)
ros2 launch llmy_bringup sensors.launch.py use_camera:=false

# Launch with custom serial port
ros2 launch llmy_control motion.launch.py servo_port:=/dev/ttyUSB0
```

### Development Scenarios

```bash
# Sensors + Communication (no motion for safety)
ros2 launch llmy_bringup bringup_robot.launch.py use_motion:=false

# Motion only (for control testing)
ros2 launch llmy_control motion.launch.py

# Full sensor suite with LiDAR
ros2 launch llmy_bringup sensors.launch.py use_rplidar:=true

# Communication with image compression disabled
ros2 launch llmy_bringup communication.launch.py use_image_compression:=false
```

## Command Velocity Multiplexing

The motion launch file includes a `twist_mux` node that multiplexes command velocity from multiple sources:

| Source | Topic | Priority | Description |
|--------|-------|----------|-------------|
| Xbox | `/cmd_vel_xbox` | 15 (highest) | Xbox controller input |
| Teleop | `/cmd_vel_teleop` | 10 | Manual teleoperation interface |
| Nav | `/cmd_vel_nav` | 5 (lowest) | Navigation stack commands |

All sources are multiplexed to `/omnidirectional_controller/cmd_vel_unstamped` with a 0.5 second timeout.

## Web Interfaces

### Rosboard Dashboard
Access the web dashboard at: `http://localhost:8888`

Features:
- Topic visualization and monitoring
- TF tree viewer
- Live camera feeds
- Service/action inspection
- Parameter configuration

### Rosbridge WebSocket
WebSocket server available at: `ws://localhost:9090`

Enables:
- Web application integration
- Remote monitoring and control
- Real-time data streaming
