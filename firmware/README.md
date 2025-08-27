# LeRemix ESP32 Micro-ROS Firmware

ESP32 firmware for the LeRemix robot that bridges ROS2 commands to Waveshare servo motors via micro-ROS. Controls both omnidirectional base movement and 6-DOF robotic arm with camera tilt.

## Features

- **Micro-ROS Integration**: Full micro-ROS client for ESP32
- **Dual Servo Control**: Base servos (continuous rotation) + Arm servos (position control)
- **High-Frequency Control**: 200Hz joint state feedback and motor control
- **Motor Braking**: Automatic braking when zero speed is commanded
- **Watchdog Safety**: Automatic motor stop when commands timeout (500ms)
- **Emergency Recovery**: Automatic torque re-enable when commands resume
- **Real-time Feedback**: Joint state publishing for ros2_control integration
- **OLED Display**: Status display for debugging and monitoring
- **Hardware Interface**: Compatible with leremix_control_plugin

## Hardware Requirements

- ESP32 development board
- Waveshare servo driver board
- 3x SMS_STS servos for omnidirectional base (IDs: 1, 2, 3)
- 7x SCSCL servos for arm + camera tilt (IDs: 4-10)
- SSD1306 OLED display (128x32)
- Serial connection for micro-ROS communication

## Pin Configuration

```cpp
// Servo Communication
#define S_RXD 18    // Servo RX pin
#define S_TXD 19    // Servo TX pin

// OLED Display (I2C)
#define S_SCL 22    // I2C Clock
#define S_SDA 21    // I2C Data

// RGB LED (Optional)
#define RGB_LED 23
```

## Topics and Interface

### Subscribed Topics (Commands from ROS2)

- `/esp32/base_cmd` - `std_msgs/Float64MultiArray`
  - Array of 3 values: wheel velocities in rad/s
  - Index 0: back_motor_rotation
  - Index 1: left_motor_rotation  
  - Index 2: right_motor_rotation

- `/esp32/arm_cmd` - `std_msgs/Float64MultiArray`
  - Array of 7 values: joint positions in radians
  - Indices 0-5: arm joints 1-6
  - Index 6: camera_tilt

### Published Topics (Feedback to ROS2)

- `/esp32/joint_states` - `sensor_msgs/JointState`
  - Complete joint state feedback for all servos
  - Includes position and velocity for each joint
  - Joint names: ["back_motor_rotation", "left_motor_rotation", "right_motor_rotation", "1", "2", "3", "4", "5", "6", "camera_tilt"]


## Servo Configuration

### Base Servos (SMS_STS - Continuous Rotation)
- **IDs**: 1, 2, 3 (back, left, right)
- **Mode**: Wheel mode (continuous rotation)
- **Control**: Velocity commands in rad/s
- **Braking**: Automatic braking when speed < 0.01 rad/s
- **Feedback**: Position and velocity at 200Hz

### Arm Servos (SCSCL - Position Control)  
- **IDs**: 4, 5, 6, 7, 8, 9, 11 (joints 1-6 + camera_tilt)
- **Mode**: Position control mode
- **Control**: Position commands in radians
- **Range**: Calibrated per-joint limits (see calibration section below)
- **Feedback**: Position and velocity at 200Hz

## Servo Calibration

The robot arm servos have been physically calibrated with the arm in a fully erect position. Each servo has specific encoder count limits and relaxed positions measured from the physical robot.

### Calibration Values

| Joint | Physical Servo ID | Range (Encoder Counts) | Relaxed Position | Description |
|-------|------------------|------------------------|------------------|-------------|
| Joint 1 (pan) | 4 | 1024 - 3072 | 2048 | Full left to full right |
| Joint 2 (shoulder) | 5 | 880 - 3072 | 1024 | Full up to full down |
| Joint 3 (elbow) | 6 | 2024 - 3880 | 3880 | Full up to full down |
| Joint 4 (wrist tilt) | 7 | 922 - 3072 | 909 | Full up to full down |
| Joint 5 (wrist rotation) | 8 | 1372 - 4096 | 2048 | Full left to full right |
| Joint 6 (gripper) | 9 | 2048 - 3446 | 2048 | Closed to open |
| Camera tilt | 11 | 1024 - 2611 | 2048 | Full down to full up |

### Calibration Notes

- **Encoder Range**: All servos use 12-bit encoders (0-4095 counts)
- **Relaxed Position**: The natural position when robot powers on, representing fully erect arm
- **Joint Limits**: Hardware limits are enforced in firmware to prevent mechanical damage
- **Conversion**: Firmware automatically converts between radians and encoder counts using calibration data
- **Startup Behavior**: Robot automatically moves to relaxed positions on boot for consistent initialization

## Building and Flashing

### Prerequisites
```bash
# Install PlatformIO
pip install platformio

# Install required libraries (automatically handled by platformio.ini)
# - Adafruit SSD1306
# - micro_ros_platformio
# - SCSTServoLibrary
```

### Build Process
```bash
cd /path/to/leremix_ws/firmware

# Clean build
pio run --target clean

# Build firmware
pio run

# Upload to ESP32
pio run --target upload

# Monitor serial output
pio device monitor
```

## Usage

### 1. Hardware Setup
1. Connect ESP32 to servo driver board
2. Wire OLED display to I2C pins
3. Connect servos with proper IDs (1-10)
4. Power on the system

### 2. Micro-ROS Agent Setup
```bash
# Install micro-ROS agent
sudo apt install ros-humble-micro-ros-agent

# Start agent (USB connection)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

# Or start agent (network connection)
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### 3. Launch ROS2 System
```bash
# Start the complete LeRemix system
ros2 launch leremix_control_plugin bringup.launch.py

# Or manually start components
ros2 run controller_manager spawner omnidirectional_controller
ros2 run controller_manager spawner arm_controller
```

### 4. Test Communication
```bash
# Check topics
ros2 topic list | grep esp32

# Monitor joint states
ros2 topic echo /esp32/joint_states

# Send base commands (move forward)
ros2 topic pub /esp32/base_cmd std_msgs/Float64MultiArray \
  "{data: [1.0, 1.0, 1.0]}"

# Send arm commands (move to center position)
ros2 topic pub /esp32/arm_cmd std_msgs/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

## Configuration

### Servo Parameters
Adjust these constants in `main.cpp` for your specific hardware:

```cpp
// Hardware configuration
const float VEL_TO_SERVO_UNIT = 13037; // Velocity conversion factor
const float COUNTS_PER_REV = 4096;     // Encoder counts per revolution
const unsigned long CONTROL_PERIOD_MS = 5;  // 200Hz control frequency

// Servo ID configuration  
const uint8_t BASE_SERVO_IDS[3] = {1, 2, 3};      // Base motor IDs
const uint8_t ARM_SERVO_IDS[7] = {4, 5, 6, 7, 8, 9, 10};  // Arm joint IDs

// Safety parameters
const float BRAKE_THRESHOLD = 0.01;  // rad/s - below this speed, brake is applied
const unsigned long WATCHDOG_TIMEOUT_MS = 500;  // Watchdog timeout period
```
