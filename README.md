
# LeRemix 
![](https://i.ibb.co/5JQqhN0/front.png)

This project is a fully 3D-printed mobile manipulator designed to be affordable, easy to build, flexible and ready for simulation.
Building on top of open-source projects like LeKiwi and LeRobot, it focuses on ROS support and extensibility, making it a solid foundation for AI/VLM experimentation, research, prototyping, and education.

# Getting Started

## Simulation

Get the robot running in Gazebo simulation in just a few commands:

```bash
# Clone and build the workspace
git clone <repository-url> leremix_ws
cd leremix_ws/ros
colcon build
source install/setup.bash

# Launch Gazebo simulation with controllers
ros2 launch leremix_gazebo sim.launch.py

# In a new terminal - start Xbox controller teleoperation
ros2 launch leremix_teleop_xbox teleop_xbox.launch.py
```

The robot will spawn in Gazebo with all controllers active. Use an Xbox controller to drive the base (face buttons) and control the arm (analog sticks + triggers).

## Real Hardware

### Prerequisites


   ```bash
   # Install ROS2 Humble
   sudo apt install ros-humble-desktop
   
   # Install additional packages
   sudo apt install ros-humble-realsense2-camera \
                    ros-humble-compressed-image-transport \
                    ros-humble-depthimage-to-laserscan \
                    ros-humble-imu-filter-madgwick \
                    ros-humble-micro-ros-agent
   ```

### Firmware Setup

1. **Flash ESP32 Firmware**
   ```bash
   # Install PlatformIO
   pip install platformio
   
   # Build and upload firmware
   cd firmware/
   pio run --target upload
   ```

2. **Start micro-ROS Agent**
   ```bash
   # Connect ESP32 via USB and start agent
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
   ```

### Launch Robot System

```bash
# Build workspace
cd leremix_ws/ros
colcon build
source install/setup.bash

# Launch hardware controllers (in separate terminals)
ros2 launch leremix_control_plugin bringup.launch.py    # Hardware interface
ros2 launch leremix_camera camera.launch.py             # Camera system  
ros2 launch leremix_imu imu.launch.py                   # IMU sensor
ros2 launch leremix_teleop_xbox teleop_xbox.launch.py   # Xbox control

# Optional: Monitor system status
ros2 topic list
ros2 control list_controllers
```

### Verification

Check that all systems are running:
```bash
# Verify controllers are active
ros2 control list_controllers

# Check camera streams
ros2 topic echo /camera/color/image_raw --once

# Verify IMU data
ros2 topic echo /imu/fused --once  

# Check joint states from ESP32
ros2 topic echo /esp32/joint_states --once
```

The robot should now respond to Xbox controller inputs with real hardware!

# ROS

## Packages

| Package | Description |
|---------|-------------|
| [**leremix_description**](ros/src/leremix_description/README.md) | Robot URDF/Xacro model, meshes, and physical properties for simulation and hardware |
| [**leremix_gazebo**](ros/src/leremix_gazebo/README.md) | Gazebo simulation environment with physics, sensors, and ros2_control integration |
| [**leremix_control**](ros/src/leremix_control/README.md) | Controller configurations (YAML) for omnidirectional base and 6-DOF arm control | 
| [**leremix_control_plugin**](ros/src/leremix_control_plugin/README.md) | ros2_control plugin for Waveshare ESP32 Servo controller via micro-ROS  | 
| [**leremix_teleop_xbox**](ros/src/leremix_teleop_xbox/README.md) | Xbox controller teleoperation for manual base and arm control | 
| [**leremix_camera**](ros/src/leremix_camera/README.md) | RealSense camera node with compressed transport and depth-to-laser conversion | 
| [**leremix_imu**](ros/src/leremix_imu/README.md) | ICM sensor integration with Madgwick filtering for orientation estimation |

# Firmware
![](https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/s/e/servo-driver-with-esp32-1.jpg)
ESP32 microcontroller running micro-ROS firmware to bridge ROS2 commands to servo hardware at 200Hz. Features safety watchdog, emergency braking, and dual servo protocol support (SMS_STS + SCSCL).

**Key Features:**
- 200Hz real-time control loop
- Safety watchdog with 500ms timeout
- Automatic emergency braking/torque disable
- micro-ROS integration via serial/WiFi
- PlatformIO development environment

📖 [**Read the complete firmware documentation →**](firmware/README.md)

# Hardware

## Parts

## BOM

### Main Platform
| Part | Amount | Unit Cost (US) | Buy (US) | Unit Cost (EU) | Buy (EU) | Total(US) | Total(EU) |
|:---|:---:|:---:|:---|:---:|---:|:-:|-|
| 12v ST3215 Feetech Servo | 10| $13.89 | [Alibaba](https://www.alibaba.com/product-detail/Feetech-STS3215-SO-ARM100-Servo-12V_1601292634404.html?spm=a2700.details.you_may_like.3.5ab1478e45kY42) | €13.38 | [Alibaba](https://www.alibaba.com/product-detail/Feetech-STS3215-SO-ARM100-Servo-12V_1601292634404.html?spm=a2700.details.you_may_like.3.5ab1478e45kY42)  | 138.9$  | €133.8 | 
| 4" Omni wheels | 3 | $9.99 | [VEX Robotics](https://www.vexrobotics.com/omni-wheels.html?srsltid=AfmBOorWdWT-FIiWSAbicYWSxqYr-d5X3CJSGxMkO33WO0thwlTn4DQu) | €24.5 | [RobotShop](https://eu.robotshop.com/products/100mm-omnidirectional-wheel-brass-bearing-rollers) | $29.97 | €73.50 | 
| WaveShare ESP32 Servo Controller | 1 | $23.99 | [Amazon](https://www.amazon.com/Expansion-Bluetooth-Control-Application-Structures/dp/B09TJ3L72Q)| €17.60  |  [Kamami](https://kamami.pl/en/Servo-Controllers/1178586-servo-driver-with-esp32-servo-driver-with-wifi-and-bluetooth-esp32-module-5906623426379.html)| $23.99 | €17.60
| Adafruit USB-PD Trigger | 2 | $5.95 | [Adafruit](https://www.adafruit.com/product/5807?srsltid=AfmBOorGANfsSrXFtplHfJPSbXODZ6rRvPt-WMq5SHoeJAwd-LjJrS7J)| €6.90| [Kamami](https://kamami.pl/en/wyzwalacze-usb-pd-elektonika/1188206-usb-type-c-power-delivery-dummy-breakout-module-with-usb-type-c-power-delivery-husb238-power-supply-controller-5906623469918.html) | $11.90 | €13.80
| Adafruit ICM20948 IMU| 1 | $14.95 | [Adafruit](https://www.adafruit.com/product/4554)| €18.91 | [Kamami](https://kamami.pl/en/sensors-6dof-9dof-10dof/587247-stemma-qt-tdk-invensense-icm-20948-9-dof-imu-module-with-9-dof-sensor-icm-20948-adafruit-4554-5906623430239.html) | $14.95 | €18.91
| Baseus GP12 Battery Pack | 1 | $59.99 | [Amazon](https://www.amazon.com/Baseus-20800mAh-Portable-Charger-Charging/dp/B0DQTYLGYK?th) | €48.99 | [Baseus EU](https://eu.baseus.com/products/energeek-gp12-power-bank-145w-20800mah?srsltid=AfmBOorIXbW42jd_DKrEXTmsDzPih-Fyf3ZeOLM2nc7CQ4IjJVjFf2mJ) | $59.99 | €48.99
| **Total** |  | |  |  |  | $279.70 | €306.60

### RGBD Cameras
The core of this robot’s perception system is an RGB-D camera due to it's flexibility. Unlike a standard RGB camera, it delivers both color and depth, making it a versatile tool for AI inference, manipulation and navigation. 

This means the robot can identify objects while also estimating their distance and size, providing richer scene understanding from a single sensor. At the same time, the depth stream enables multiple SLAM approaches—RGB-SLAM, RGB-D SLAM, or even 2D SLAM via depthimage_to_laserscan.

All the software in this repo is built on top of the Intel RealSense SDK and ROS driver, but other depth cameras can serve as near drop-in replacements depending on needs.


| Part                | Unit Cost (US) | Buy (US) | Unit Cost (EU) | Buy (EU) | Robot + Camera (US) | Robot + Camera (EU) |
|---------------------|---------------:|:---------|---------------:|:---------|-----------------------------:|-----------------------------:|
| **YDLidar HP60C**   |  $202.00        | [RobotShop US](https://www.robotshop.com/products/ydlidar-hp60c-compact-lidar-sensor-02-4m-range-73-8-scan-angle) | €190.00 | [RobotShop EU](https://eu.robotshop.com/products/ydlidar-hp60c-compact-lidar-sensor-02-4m-range-73-8-scan-angle) | **$481.70** | **€496.60** |
| **Orbbec Gemini 2** | $234.00        | [Orbbec Store](https://store.orbbec.com/products/gemini-2) | €230.00 | [Orbbec EU](https://store.orbbec3d.com) | **$513.70** | **€536.60** |
| **Realsense D415**  |  $272.00        | [Realsense Store](https://store.realsenseai.com/buy-intel-realsense-depth-camera-d415.html) | €310.00 | [Mouser EU](https://eu.mouser.com/ProductDetail/Intel/82635AWGDVKPRQ) | **$551.70** | **€616.60**v |
| **Realsense D435**  |  $314.00        | [Realsense Store](https://store.realsenseai.com/buy-intel-realsense-depth-camera-d435.html) | €317.60 | [Mouser EU](https://eu.mouser.com/ProductDetail/Intel/82635AWGDVKPRQ) |**$593.70** | **€624.20** |
| **Stereolabs ZED 2i** |  $499.00        | [Stereolabs Store](https://store.stereolabs.com/products/zed-2i) | **€449.00** | [Stereolabs EU](https://store.stereolabs.com/products/zed-2i) | **$778.70** | **€755.60** |


### SBCs, Computers
My version of the robot is powered by an Nvidia Orin Nano Super, but the battery pack allows up to 65W per power lane and up to 20V, so you could use whatever you want, a Raspberry Pi 5, Nvidia Orin, even a NUC or other Mini-PCs. 


## Credits
* The [Lerobot Team](https://github.com/huggingface/lerobot) 
* The [SIGRobotics-UIUC](https://github.com/SIGRobotics-UIUC) for their work on LeKiwi
* [Pavan Vishwanath](https://github.com/Pavankv92) for his work on the [ROS2 package](https://github.com/Pavankv92/lerobot_ws) for the [LeRobot SO-ARM101](https://github.com/huggingface/lerobot)
* [Mateus Menezes](https://github.com/mateusmenezes95) for his work on the [omnidirectional_controllers](https://github.com/mateusmenezes95/omnidirectional_controllers) ROS control package and [axebot](https://github.com/mateusmenezes95/axebot) which helped me a lot with setting up the simulation for omnidirectional driving