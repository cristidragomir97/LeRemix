# LLMy Launch Files Documentation

This document provides an overview of all launch files in the LLMy ROS 2 workspace and their parameters.

## Main System Launch Files

### llmy_bringup/launch/bringup_robot.launch.py
**Description**: Main modular launch file that coordinates all three subsystems (sensors, motion, communication).

**Parameters**:
- `use_sensors` (bool, default: `true`) - Enable all sensor systems
- `use_motion` (bool, default: `true`) - Enable motion and control systems  
- `use_communication` (bool, default: `true`) - Enable communication and web interfaces

**Includes**: sensors.launch.py, motion.launch.py, communication.launch.py

### llmy_bringup/launch/sensors.launch.py
**Description**: Sensor subsystem launch file (cameras, IMU, RPLidar).

**Parameters**:
- `use_camera` (bool, default: `true`) - Enable camera system (RealSense + wrist cam)
- `use_imu` (bool, default: `true`) - Enable IMU sensor
- `use_rplidar` (bool, default: `false`) - Enable RPLidar C1

**Includes**: camera.launch.py, imu.launch.py, rplidar_c1_launch.py

### llmy_bringup/launch/motion.launch.py
**Description**: Motion subsystem launch file (servos, ros2_control, teleop).

**Parameters**:
- `use_base_systems` (bool, default: `true`) - Enable servo manager (motor control)
- `use_control_stack` (bool, default: `true`) - Enable robot state publisher and controllers
- `use_xbox` (bool, default: `true`) - Enable Xbox controller teleoperation
- `servo_port` (string, default: `/dev/ttyTHS1`) - Serial port for servo communication
- `servo_baud` (string, default: `1000000`) - Baudrate for servo communication

**Includes**: servo_manager.launch.py, control_stack.launch.py, teleop_xbox.launch.py

### llmy_bringup/launch/communication.launch.py
**Description**: Communication subsystem launch file (rosbridge, rosboard, image compression).

**Parameters**:
- `use_rosbridge` (bool, default: `true`) - Enable rosbridge server for web interfaces
- `use_rosboard` (bool, default: `true`) - Enable rosboard web dashboard
- `use_image_compression` (bool, default: `true`) - Enable image compression and depth-to-scan

**Nodes**: rosbridge_websocket, rosboard_node, image compression nodes, depthimage_to_laserscan

## Control System Launch Files

### llmy_control/launch/base_systems.launch.py
**Description**: Launches base hardware systems including servo manager.

**Parameters**:
- `servo_port` (string, default: `/dev/ttyTHS1`) - Serial port for servo manager
- `servo_baud` (string, default: `1000000`) - Baudrate for servo manager
- `servo_config` (string, default: `''`) - Path to servo manager config file (uses package default if empty)

**Includes**: servo_manager.launch.py

### llmy_control/launch/control_stack.launch.py
**Description**: Launches the ros2_control stack with robot description and controllers.

**Parameters**: None

**Nodes**: robot_state_publisher, controller_manager, controller spawners (joint_state_broadcaster, diff_drive_controller, arm_controller, head_controller)


## Camera System Launch Files

### llmy_camera/launch/camera.launch.py
**Description**: Comprehensive camera setup with RealSense and USB wrist camera.

**Parameters**:
- `camera_name` (string, default: `head_camera`) - RealSense camera name
- `wrist_camera_device` (string, default: `/dev/video0`) - Wrist camera device path
- `enable_wrist_camera` (bool, default: `true`) - Enable wrist camera
- `device_type` (string, default: `''`) - RealSense device type (e.g., d435i, d455, l515)
- `enable_compressed` (bool, default: `false`) - Enable compressed image transport
- `enable_laser_scan` (bool, default: `false`) - Enable depth to laser scan conversion
- `laser_scan_min_height` (float, default: `-0.5`) - Minimum height for laser scan slice
- `laser_scan_max_height` (float, default: `0.5`) - Maximum height for laser scan slice
- `laser_scan_angle_min` (float, default: `-1.57`) - Minimum angle for laser scan (-90 degrees)
- `laser_scan_angle_max` (float, default: `1.57`) - Maximum angle for laser scan (+90 degrees)
- `laser_scan_angle_increment` (float, default: `0.005`) - Angular resolution of laser scan
- `laser_scan_range_min` (float, default: `0.2`) - Minimum range for laser scan
- `laser_scan_range_max` (float, default: `10.0`) - Maximum range for laser scan

**Includes**: realsense2_camera/launch/rs_launch.py

**Nodes**: wrist camera (usb_cam), RealSense camera

**Note**: Image compression and depth-to-scan are now handled by communication.launch.py

### llmy_camera/launch/realsense_only.launch.py
**Description**: Minimal RealSense camera setup.

**Parameters**:
- `camera_name` (string, default: `camera`) - RealSense camera name
- `device_type` (string, default: `d435i`) - RealSense device type

**Includes**: realsense2_camera/launch/rs_launch.py

## Sensor Launch Files

### llmy_imu/launch/imu.launch.py
**Description**: IMU sensor setup with Madgwick filter.

**Parameters**:
- `publish_rate` (float, default: `200.0`) - Publishing rate for IMU data in Hz
- `frame_id` (string, default: `imu_link`) - Frame ID for IMU data
- `use_mag` (bool, default: `true`) - Use magnetometer for orientation filtering

**Nodes**: imu_node, imu_filter_madgwick

## Teleop Launch Files

### llmy_teleop_xbox/launch/teleop_xbox.launch.py
**Description**: Xbox controller teleoperation setup.

**Parameters**:
- `joy_dev` (string, default: `0`) - Joystick device ID
- `switch_controllers` (bool, default: `true`) - Switch controllers flag

**Nodes**: joy_node, teleop_xbox (with controller command topics configured)

## Navigation & SLAM Launch Files

### llmy_nav/launch/slam_nav.launch.py
**Description**: Combined SLAM + Navigation launch — builds the map while navigating autonomously. slam_toolbox provides the map and map→odom transform, replacing both map_server and AMCL.

**Parameters**:
- `use_sim_time` (bool, default: `false`) - Use simulation time
- `autostart` (bool, default: `true`) - Automatically start lifecycle nodes

**Nodes**: slam_toolbox (async), planner_server (SmacPlanner2D), controller_server (MPPI), bt_navigator, behavior_server, velocity_smoother, lifecycle_manager_slam_nav

**Remappings**: controller output → `cmd_vel_raw`, smoother output → `cmd_vel_nav`

### llmy_nav/launch/navigation.launch.py
**Description**: Full Nav2 navigation with a pre-built map. Requires a saved map file.

**Parameters**:
- `use_sim_time` (bool, default: `false`) - Use simulation time
- `autostart` (bool, default: `true`) - Automatically start lifecycle nodes
- `map` (string) - Path to map YAML file

**Nodes**: map_server, amcl, planner_server, controller_server, bt_navigator, behavior_server, velocity_smoother, lifecycle_manager_navigation

### llmy_nav/launch/mapping.launch.py
**Description**: SLAM-only mode for building maps. No autonomous navigation — just map building while teleoperating.

**Parameters**:
- `use_sim_time` (bool, default: `false`) - Use simulation time
- `autostart` (bool, default: `true`) - Automatically start lifecycle nodes

**Nodes**: slam_toolbox (async), lifecycle_manager_slam

### llmy_nav/launch/mapfree.launch.py
**Description**: Reactive navigation without any map. Publishes an identity map→odom transform and runs local obstacle avoidance only.

**Parameters**:
- `use_sim_time` (bool, default: `false`) - Use simulation time
- `autostart` (bool, default: `true`) - Automatically start lifecycle nodes

**Nodes**: static_transform_publisher (map→odom), planner_server, controller_server, bt_navigator, behavior_server, velocity_smoother, lifecycle_manager_mapfree

### llmy_nav/launch/mode_manager.launch.py
**Description**: Orchestrates switching between navigation modes at runtime.

**Nodes**: mode_manager

## MoveIt Launch Files

### llmy_moveit/launch/moveit.launch.py
**Description**: MoveIt 2 motion planning for the 6-DOF arm. Launches move_group with OMPL planners and KDL kinematics. Expects the control stack to already be running.

**Parameters**:
- `use_sim_time` (bool, default: `false`) - Use simulation time
- `launch_rviz` (bool, default: `true`) - Launch RViz with MoveIt plugin

**Nodes**: move_group, rviz2_moveit (conditional)

**Note**: Requires `arm_trajectory_controller` to be loaded (spawned inactive by control_stack). Activate with: `ros2 control switch_controller --activate arm_trajectory_controller --deactivate arm_controller`

## Simulation Launch Files

### llmy_gazebo/launch/sim.launch.py
**Description**: Gazebo simulation environment setup.

**Parameters**:
- `gui` (bool, default: `true`) - Enable Gazebo GUI
- `pause` (bool, default: `false`) - Start simulation paused
- `spawn_z` (float, default: `0.05`) - Z position for robot spawn
- `cm_timeout` (string, default: `60`) - Controller manager timeout

**Includes**: gazebo_ros/launch/gzserver.launch.py, gazebo_ros/launch/gzclient.launch.py

**Nodes**: robot_state_publisher, spawn_entity, controller spawners (diff_drive_controller, arm_controller, head_controller - with event handlers for sequencing)

## Hardware Manager Launch Files

### llmy_servo_manager/launch/servo_manager.launch.py
**Description**: Servo motor manager for hardware communication.

**Parameters**:
- `config_file` (string, default: package config file) - Path to the servo manager config file
- `port` (string, default: `/dev/ttyUSB0`) - Serial port for motor communication
- `baud` (string, default: `2000000`) - Baud rate for serial communication

**Nodes**: servo_manager_node

### llmy_servo_manager/launch/test_brake_methods.launch.py
**Description**: Servo brake testing utility.

**Parameters**: 
- `config_file` (string, default: package config file) - Path to the servo manager config file
- `port` (string, default: `/dev/ttyUSB0`) - Serial port for motor communication  
- `baud` (string, default: `2000000`) - Baud rate for serial communication

**Nodes**: test_brake_methods

## Usage Examples

### Launch the complete robot system:
```bash
ros2 launch llmy_bringup bringup_robot.launch.py
```

### Launch with custom servo port:
```bash
ros2 launch llmy_bringup bringup_robot.launch.py servo_port:=/dev/ttyUSB1
```

### Launch desktop development environment:
```bash
ros2 launch llmy_bringup bringup_desktop.launch.py
```

### Launch simulation:
```bash
ros2 launch llmy_gazebo sim.launch.py
```

### Launch only camera system:
```bash
ros2 launch llmy_camera camera.launch.py
```