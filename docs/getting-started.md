# Getting Started with LLMy

This guide walks you through setting up LLMy from your first simulation to running on real hardware.

## Prerequisites

### System Requirements
- **OS:** Ubuntu 22.04 LTS
- **ROS2:** Humble Hawksbill
- **Simulator:** Gazebo Classic (for simulation)

### Hardware Requirements (Real Robot Only)
- FEETECH STS3215 servos (12x)
- Serial motor controller (FE-URT-1 or Waveshare adapter)
- USB-C power bank (145W with PD trigger)
- Single-board computer (Raspberry Pi 5 or Jetson Orin Nano)

### Optional Peripherals
- Xbox controller (for teleoperation)
- RealSense D435i / ZED 2i / Orbbec Gemini 2 (RGB-D camera)
- ICM-20948 IMU
- RPLidar C1 (LIDAR)

---

## Installation

### 1. Install ROS2 Humble

If you don't have ROS2 Humble installed:

```bash
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop
```

### 2. Clone LLMy

```bash
git clone https://github.com/cristidragomir97/llmy.git llmy_ws
cd llmy_ws/ros
```

### 3. Install Dependencies

```bash
# Install vcstool for managing repository dependencies
sudo apt install -y python3-vcstool

# Import external ROS2 packages (diff drive controllers, rplidar)
vcs import src < repos.vcs

# Install all workspace dependencies
rosdep install --from-paths src --ignore-src -r -y

# Optional: Install additional hardware support packages
sudo apt install -y \
    ros-humble-realsense2-camera \
    ros-humble-compressed-image-transport \
    ros-humble-depthimage-to-laserscan \
    ros-humble-imu-filter-madgwick
```

### 4. Build the Workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

**Tip:** Add `source ~/llmy_ws/ros/install/setup.bash` to your `~/.bashrc` to automatically source the workspace.

---

## Simulation Quickstart

The fastest way to see LLMy in action is through Gazebo simulation.

### Launch the Simulation

```bash
# Terminal 1: Start Gazebo with the robot
ros2 launch llmy_gazebo sim.launch.py
```

You should see the robot spawn in Gazebo with all controllers loaded.

### Control with Xbox Controller (Optional)

```bash
# Terminal 2: Launch Xbox teleop
ros2 launch llmy_teleop_xbox teleop_xbox.launch.py
```

**Controller Mapping:**
- **Right Stick:** Drive base (forward/back + rotation)
- **RB/LB Buttons:** Arm joint 1 (+/-)
- **RT/LT Triggers:** Arm joint 2 (+/-)
- **Y/A Buttons:** Arm joint 3 (+/-)
- **B/X Buttons:** Arm joint 4 (+/-)
- **Start/Back Buttons:** Arm joint 5 (+/-)
- **Stick Clicks:** Arm joint 6 (+/-)
- **D-Pad Up/Down:** Camera pan
- **D-Pad Left/Right:** Camera tilt

### Verify Controllers are Running

```bash
# List active controllers
ros2 control list_controllers

# Expected output:
# arm_controller[joint_trajectory_controller/JointTrajectoryController] active
# diff_drive_controller[diff_drive_controller/DiffDriveController] active
# head_controller[joint_trajectory_controller/JointTrajectoryController] active
```

---

## Hardware Setup

### Step 1: Configure Servo IDs

Before assembly, configure all servo IDs and mark them physically.

```bash
cd ~/llmy_ws/scripts

# Interactive mode - connect servos one at a time
python3 set_servo_id.py
```

**ID Assignment:**
- Base (drivetrain): **1, 2, 3, 4**
- Arm joints: **5, 6, 7, 8, 9, 10**
- Head (pan/tilt): **11, 12**

**Mark each servo** with a pen/marker after setting its ID to avoid confusion during assembly.

**Verify all servos:**
```bash
python3 set_servo_id.py --list
```

See [`scripts/README.md`](../scripts/README.md) for detailed instructions.

### Step 2: Build Software & Test Before Assembly

Build the ROS workspace and verify servo communication before physical assembly.

```bash
cd ~/llmy_ws/ros
colcon build --symlink-install
source install/setup.bash
```

**Test servo connectivity:**
```bash
# Connect all servos via daisy-chain temporarily
ros2 launch llmy_servo_manager ping_test.launch.py
```

This ensures all electronics work before you commit to full assembly.

### Step 3: Physical Assembly

**Required Tools:**
- Soldering iron (for heat-set inserts)
- M3 screws and hex wrenches
- Marker/pen (for labeling)

**Assembly Sequence:**

#### Step 1: Prepare Structural Parts
Insert M3 heat-set nuts using a soldering iron into:
- [servo_mount.stl](../parts/base/servo_mount.stl) (base wheel mounts)
- [battery_left.stl](../parts/base/battery_left.stl) (battery holder left side)
- [battery_right.stl](../parts/base/battery_right.stl) (battery holder right side)

#### Step 2: Drivetrain Assembly
- Install [wheel_hub.stl](../parts/base/wheel_hub.stl) on drivetrain servos **(IDs 1, 2, 3, 4)**
- Attach standard wheels to hubs
- Mount servos into [servo_mount.stl](../parts/base/servo_mount.stl)

#### Step 3: Base Platform
- Mount the four drivetrain assemblies to the base plate:
  - **Servo ID 1** → Front-left position
  - **Servo ID 2** → Back-left position
  - **Servo ID 3** → Front-right position
  - **Servo ID 4** → Back-right position
- Install battery mounts ([battery_left.stl](../parts/base/battery_left.stl), [battery_right.stl](../parts/base/battery_right.stl))
- If using LiDAR: mount to [lidar_mount.stl](../parts/base/lidar_mount.stl)
- Slide USB-C power bank into battery holder slot

#### Step 4: Arm Assembly
Assemble the 6-DOF arm following the [official SO-ARM100 instructions](https://www.hiwonder.com/products/so-arm100).
- Use servos **IDs 5-10** in order from base to gripper
- Ensure servo horns are centered before assembly

#### Step 5: Second Floor Integration
Mount on the second floor platform:
- Assembled robot arm
- Single-board computer (Raspberry Pi 5 or Jetson Orin Nano) with [orin_mount.stl](../parts/base/orin_mount.stl)
- USB-PD trigger modules (2x for 12V power) with [adafruit_usb_pd_mount.stl](../parts/base/adafruit_usb_pd_mount.stl)
- IMU sensor (ICM-20948) with [adafruit_icm_mount.stl](../parts/base/adafruit_icm_mount.stl)
- I2C switch module with [adafruit_i2c_switch_mount.stl](../parts/base/adafruit_i2c_switch_mount.stl)
- Motor controller with [fe_urt_1_mount.stl](../parts/base/fe_urt_1_mount.stl)

#### Step 6: Wiring - First Pass
- Route servo wires through cable management holes
- Create daisy-chain starting from **base servo 1** → continuing through all servos → ending at **camera tilt servo (ID 12)**
- Leave one free connector at the start (will connect to motor controller)
- Verify you have a clean daisy-chain from base to camera

#### Step 7: Camera Tower
- Assemble pan-tilt mechanism using head servos **(IDs 11, 12)**
- Mount to camera tower structure
- Attach RGB-D camera (RealSense/ZED/Orbbec)
- Install tower on top floor platform

#### Step 8: Motor Controller Placement
- Mount FE-URT-1 or Waveshare servo controller to the back of the arm base using [fe_urt_1_mount.stl](../parts/base/fe_urt_1_mount.stl) or [WaveShare_Mounting_Plate_SO101.stl](https://github.com/TheRobotStudio/SO-ARM100/blob/main/STL/SO101/Individual/WaveShare_Mounting_Plate_SO101.stl)
- Connect daisy-chain free connector to controller output
- Connect controller to SBC via USB or UART

#### Step 9: Complete Wiring
- **Power:**
  - USB-PD module 1 → SBC (12V or 5V depending on trigger)
  - USB-PD module 2 → Motor controller (12V)
- **Data:**
  - Camera → SBC (USB)
  - IMU → SBC (I2C)
  - Motor controller → SBC (USB or UART)
  - LiDAR → SBC (USB, if installed)
- **Organize cables** with zip ties and ensure nothing interferes with moving parts

**Assembly Tips:**
- Double-check servo IDs before installation
- Test each servo individually before daisy-chaining
- Leave slack in wiring for joint movement
- Use cable management holes to prevent wire snagging
- Take photos during assembly for reference

### Step 4: Calibrate the Arm

After assembly, calibrate the arm's zero position and joint limits.

**⚠️ Important:** Stop any running ROS nodes first:
```bash
pkill -f servo_manager
```

**Run calibration wizard:**
```bash
cd ~/llmy_ws/scripts
python3 calibrate_arm.py --config ~/llmy_ws/ros/src/llmy_servo_manager/config/servo_manager.yaml
```

**Calibration process:**
1. Position arm fully erect (vertical) with gripper closed → set zero position
2. For each joint, move to minimum position → record limit
3. For each joint, move to maximum position → record limit
4. Script saves calibration data to config file

This ensures safe operation by preventing the arm from exceeding physical limits.

See [`scripts/README.md`](../scripts/README.md) for detailed calibration instructions.

### Step 5: Launch the Robot

Configure serial port access:

```bash
# Give user permission for serial port
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect

# Identify your servo controller port
ls /dev/ttyUSB* /dev/ttyACM* /dev/ttyTHS*
```

**Common ports:**
- `/dev/ttyUSB0` - USB adapter
- `/dev/ttyACM0` - USB CDC adapter
- `/dev/ttyTHS1` - Jetson hardware UART

Update port in config if needed:
```bash
nano ~/llmy_ws/ros/src/llmy_servo_manager/config/servo_manager.yaml
# Change: port: "/dev/ttyTHS1" to your port
```

**Launch the complete robot system:**

**Single-command bringup (recommended):**

```bash
ros2 launch llmy_bringup bringup_robot.launch.py
```

**With custom serial configuration:**

```bash
ros2 launch llmy_bringup bringup_robot.launch.py \
    servo_port:=/dev/ttyUSB0 \
    servo_baud:=1000000
```

**Disable specific subsystems:**

```bash
ros2 launch llmy_bringup bringup_robot.launch.py \
    use_camera:=false \
    use_xbox:=false
```

### Launch Components Individually (Advanced)

For debugging or custom configurations, launch subsystems separately:

```bash
# Terminal 1: Servo manager (low-level motor control)
ros2 launch llmy_control base_systems.launch.py

# Terminal 2: Hardware interface & controllers
ros2 launch llmy_control_plugin bringup.launch.py

# Terminal 3: Camera system (optional)
ros2 launch llmy_camera camera.launch.py

# Terminal 4: IMU sensor (optional)
ros2 launch llmy_imu imu.launch.py

# Terminal 5: Xbox controller (optional)
ros2 launch llmy_teleop_xbox teleop_xbox.launch.py
```

---

## Verification & Testing

### Check System Status

```bash
# Verify all controllers are active
ros2 control list_controllers

# List active ROS2 topics
ros2 topic list

# Monitor system diagnostics
ros2 topic echo /diagnostics
```

### Test Motor Communication

```bash
# Check joint states from motor manager
ros2 topic echo /motor_manager/joint_states --once

# Run servo connectivity test
ros2 launch llmy_servo_manager ping_test.launch.py
```

### Verify Sensor Data

```bash
# Camera stream (if enabled)
ros2 topic echo /camera/color/image_raw --once

# IMU data (if enabled)
ros2 topic echo /imu/fused --once

# LIDAR scan (if enabled)
ros2 topic echo /scan --once
```

### Test Manual Control

With Xbox controller connected and teleop running:
1. Gently move the right stick → base should move
2. Move the left stick → arm joints 1 & 2 should respond
3. Press D-pad → camera should pan/tilt

---

## Troubleshooting

### Controllers Not Loading

**Symptom:** `ros2 control list_controllers` shows controllers as `unconfigured` or `inactive`

**Solutions:**
```bash
# Check hardware interface connection
ros2 topic echo /motor_manager/joint_states --once

# Manually load controllers
ros2 control load_controller arm_controller
ros2 control configure_controller arm_controller
ros2 control switch_controllers --activate arm_controller
```

### Servo Communication Errors

**Symptom:** Timeout errors or `Failed to communicate with servo ID X`

**Solutions:**
1. Verify servo IDs match configuration file
2. Check baud rate: `servo_baud:=1000000` (common values: 115200, 1000000)
3. Test individual servos with ping test
4. Check physical connections and power supply

### Camera Not Detected

**Symptom:** No camera topics appearing

**Solutions:**
```bash
# List USB devices
lsusb

# Test RealSense directly
realsense-viewer

# Check launch parameters
ros2 launch llmy_camera camera.launch.py --show-args
```

### Xbox Controller Not Responding

**Symptom:** Robot doesn't move with controller input

**Solutions:**
```bash
# Verify controller is detected
ls /dev/input/js*

# Test controller input
jstest /dev/input/js0

# Check teleop node is publishing
ros2 topic echo /cmd_vel
```

---

## Next Steps

### For Developers

- **Explore Packages:** Read the [detailed package documentation](packages.md)
- **Understand Architecture:** Review the system architecture in the [main README](../README.md#-architecture)
- **Customize Configuration:** Edit YAML files in `ros/src/*/config/`
- **Build Custom Modules:** Create new ROS2 packages in `ros/src/`

### Community & Support

- **Issues:** Report bugs at [github.com/cristidragomir97/llmy/issues](https://github.com/cristidragomir97/llmy/issues)
- **Discussions:** Join conversations in GitHub Discussions
- **Documentation:** Contribute improvements via pull requests
---

**You're ready to start experimenting with LLMy!**

For hardware assembly details, see [Assembly Guide](assembly.md). For BOM and sourcing, check [BOM Documentation](bom.md).
