# LLMy Setup Scripts

Utility scripts for configuring and calibrating your LLMy robot.

## Scripts Overview

### 1. `set_servo_id.py` - Servo ID Configuration

Interactive tool for setting FEETECH STS3215 servo IDs one by one.

**Usage:**

```bash
# List all servos on the bus with telemetry
python3 set_servo_id.py --list

# Interactive mode - configure servos one by one
python3 set_servo_id.py

# Direct ID change
python3 set_servo_id.py --old-id 1 --new-id 5

# Custom serial port
python3 set_servo_id.py --port /dev/ttyACM0 --list
```

**LLMy ID Assignments:**
- Base (locomotion): IDs **1, 2, 3**
- Arm (manipulator): IDs **4, 5, 6, 7, 8, 9**
- Head (pan & tilt): IDs **10, 11**

**Workflow:**
1. Connect servos **one at a time** to avoid ID conflicts
2. Run interactive mode: `python3 set_servo_id.py`
3. For each servo:
   - Connect it to the bus
   - Script detects current ID
   - Enter desired new ID
   - Disconnect and connect next servo
4. Final verification scan shows all configured servos

---

### 2. `calibrate_arm.py` - Arm Calibration

Calibrates arm joint zero positions and movement limits.

**Usage:**

```bash
# Basic calibration (will prompt for save)
python3 calibrate_arm.py

# Auto-save to config file
python3 calibrate_arm.py --config ~/llmy_ws/ros/src/llmy_servo_manager/config/servo_manager.yaml

# Custom serial port
python3 calibrate_arm.py --port /dev/ttyUSB0
```

**⚠️ IMPORTANT:** Stop all ROS2 nodes before running calibration!

```bash
# Kill any running servo manager nodes
pkill -f servo_manager
```

**Calibration Process:**

1. **Zero Position Setup:**
   - Position arm fully erect (vertical)
   - Close gripper completely
   - Center wrist rotation
   - Script records zero positions

2. **Joint Limit Calibration:**
   - For each of 6 arm joints:
     - Move joint to minimum position → record
     - Move joint to maximum position → record
   - Script validates and saves limits

3. **Verification:**
   - Arm returns to zero position
   - Summary table shows all calibration data
   - Option to save to YAML config

**Output Example:**
```
┌─────────────────────────┬──────┬──────────┬─────────┬─────────┐
│ Joint                   │  ID  │ Zero Pos │ Min Pos │ Max Pos │
├─────────────────────────┼──────┼──────────┼─────────┼─────────┤
│ Joint 1 (Base Rotation) │   4  │  2048    │   512   │  3584   │
│ Joint 2 (Shoulder)      │   5  │  2048    │   768   │  3328   │
...
```

**What Gets Saved:**

The calibration tool updates `servo_manager.yaml` with:
- `arm_zero_positions`: Reference position for each joint (6 values)
- `arm_min_limits`: Minimum safe position in ticks (6 values)
- `arm_max_limits`: Maximum safe position in ticks (6 values)

These limits are automatically enforced by the servo manager to prevent damage.

---

## Prerequisites

### Hardware Setup
- Robot powered on (12V to servos)
- Serial connection established (USB or UART)
- Correct serial port identified

### Software Requirements
```bash
# Install st3215 Python library
pip install st3215

# Install PyYAML for config management
pip install pyyaml
```

### Permissions
```bash
# Add your user to dialout group for serial access
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```

---

## Common Serial Ports

| Platform | Port | Description |
|----------|------|-------------|
| Raspberry Pi | `/dev/ttyUSB0` | USB adapter |
| Jetson Nano/Orin | `/dev/ttyTHS1` | Hardware UART |
| Generic Linux | `/dev/ttyACM0` | USB CDC adapter |
| Waveshare adapter | `/dev/ttyUSB0` | FT232 USB-Serial |

**Find your port:**
```bash
# List all serial devices
ls /dev/tty{USB,ACM,THS}* 2>/dev/null

# Monitor kernel messages when plugging in
dmesg | tail
```

---

## Workflow: Full Robot Setup

**Complete setup from scratch:**

```bash
# 1. Set all servo IDs (connect one at a time)
cd ~/llmy_ws/scripts
python3 set_servo_id.py

# 2. Verify all servos are detected
python3 set_servo_id.py --list

# 3. Calibrate arm joints
python3 calibrate_arm.py --config ../ros/src/llmy_servo_manager/config/servo_manager.yaml

# 4. Build and launch ROS workspace
cd ~/llmy_ws/ros
colcon build
source install/setup.bash
ros2 launch llmy_bringup bringup_robot.launch.py
```

---

## Troubleshooting

### Script can't connect to servos
```bash
# Check port exists
ls -l /dev/ttyUSB0

# Check permissions
groups  # Should show 'dialout'

# Try different baud rates
python3 set_servo_id.py --baud 115200
```

### Servo not responding during calibration
- Ensure 12V power supply is connected
- Check servo cables are properly seated
- Verify servo ID matches expected value
- Try power cycling the servo

### Calibration data not saving
- Verify config file path is correct
- Check file permissions (must be writable)
- Review backup file (.backup) for changes
- Manually inspect YAML syntax

### ROS node conflicts
```bash
# Kill all servo-related processes
pkill -f servo_manager
pkill -f llmy

# Verify no nodes running
ros2 node list
```

---

## Advanced Usage

### Batch ID Configuration
```bash
# Script for setting multiple servos
for id in {1..11}; do
  echo "Connect servo for ID $id and press enter..."
  read
  python3 set_servo_id.py --old-id 1 --new-id $id
done
```

### Custom Calibration Workflow
```python
# Manual calibration via Python API
from st3215 import ST3215

servo = ST3215('/dev/ttyUSB0')
pos = servo.ReadPosition(4)  # Read joint 1 position
print(f"Current position: {pos} ticks")
```

### Verify Limits Are Enforced
```bash
# After calibration, test limit enforcement
ros2 topic pub /motor_manager/arm_cmd std_msgs/msg/Float64MultiArray "data: [10.0, 0, 0, 0, 0, 0]"
# Joint should clamp to max limit, not attempt invalid position
```

---

## Support

For issues or questions:
- GitHub Issues: https://github.com/cristidragomir97/llmy/issues
- Documentation: [docs/getting-started.md](../docs/getting-started.md)
