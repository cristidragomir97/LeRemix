# Simple MicroROS Servo Driver

A simplified, barebones version of the microros_servo_driver with only essential functionality.

## Features

- ✅ MicroROS communication
- ✅ Servo control (arm and base servos)
- ✅ Joint state publishing
- ✅ Serial debugging output
- ❌ No display/OLED support
- ❌ No button input handling
- ❌ No watchdog functionality
- ❌ Minimal dependencies

## Hardware Requirements

- ESP32 development board
- Servo communication via GPIO 18 (RX) and GPIO 19 (TX)
- Serial connection for debugging and MicroROS communication

## Configuration

All servo calibration and hardware settings are defined in `src/config.h`.

## Build and Upload

This project uses PlatformIO:

```bash
# Build the project
pio run

# Upload to ESP32
pio run --target upload

# Monitor serial output
pio device monitor
```

## Dependencies

- MicroROS PlatformIO library
- SCServo library for servo communication

## Usage

1. Flash the firmware to your ESP32
2. Connect via serial at 115200 baud
3. The device will automatically connect to MicroROS and start listening for commands
4. Servos are enabled by default

## Topics

- `/esp32/base_cmd` - Base motor commands (Float64MultiArray)
- `/esp32/arm_cmd` - Arm joint commands (Float64MultiArray) 
- `/esp32/joint_states` - Joint state feedback (JointState)