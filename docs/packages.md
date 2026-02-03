# LLMy ROS Packages


| Package | Purpose | Language | Key Topics | Launch Command |
|---------|---------|----------|------------|----------------|
| **llmy_description** | Robot URDF model & transforms | XML/Python | `/robot_description`, `/tf`, `/joint_states` | `ros2 launch llmy_description view_robot.launch.py` |
| **llmy_servo_manager** | Direct motor control (C++) | C++ | `/motor_manager/*_cmd`, `/motor_manager/joint_states` | `ros2 launch llmy_servo_manager servo_manager.launch.py` |
| **llmy_servo_manager_py** | Motor control with advanced braking | Python | `/motor_manager/*_cmd`, `/motor_manager/joint_states` | `ros2 launch llmy_servo_manager_py servo_manager.launch.py` |
| **llmy_control_plugin** | ros2_control hardware bridge | C++ | `/cmd_vel`, `/arm_controller/joint_trajectory` | `ros2 launch llmy_control_plugin bringup.launch.py` |
| **llmy_teleop_xbox** | Xbox controller interface | Python | `/cmd_vel`, `/joy` | `ros2 launch llmy_teleop_xbox teleop_xbox.launch.py` |
| **llmy_control** | Controller configurations | YAML | N/A (config files) | Loaded by control_plugin |
| **llmy_camera** | RGB-D camera & vision | Python | `/head_camera/*`, `/scan`, `/wrist_camera/*` | `ros2 launch llmy_camera camera.launch.py` |
| **llmy_nav** | Navigation & SLAM modes | Python/YAML | `/cmd_vel_nav`, `/map`, `/plan` | `ros2 launch llmy_nav slam_nav.launch.py` |
| **llmy_moveit** | MoveIt 2 arm planning | YAML/Python | `/move_group/*`, `/planning_scene` | `ros2 launch llmy_moveit moveit.launch.py` |
| **llmy_mcp** | MCP bridge for LLMs | Python | `/cmd_vel_mcp`, dynamic topics | `ros2 run llmy_mcp mcp_server` |
| **llmy_imu** | IMU sensor & fusion | Python | `/imu/data`, `/imu/fused` | `ros2 launch llmy_imu imu.launch.py` |

### 📦 Detailed Package Information

#### **📐 llmy_description - Robot Model**

**What it does:** Provides the complete URDF/Xacro robot model with accurate physical properties, joint limits, collision meshes, and visual representations. This is the "digital twin" of your physical robot.

**Nodes launched:**
- `robot_state_publisher` - Publishes robot transforms and joint states
- `joint_state_publisher` - (Optional) For manual joint control in simulation

**How to run:**
```bash
# Standalone URDF visualization
ros2 launch llmy_description view_robot.launch.py

# Load robot model for other packages
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro $(ros2 pkg prefix llmy_description)/share/llmy_description/urdf/llmy.urdf.xacro)"
```

**Key topics:**
- `/robot_description` - URDF robot model
- `/tf` and `/tf_static` - Robot transforms
- `/joint_states` - Current joint positions


#### **🔧 llmy_servo_manager - Direct Motor Control (C++)**

**What it does:** Handles low-level communication with FEETECH STS servos via serial protocol. Converts ROS2 joint commands into servo-specific position/velocity commands and provides real-time telemetry feedback.

**Nodes launched:**
- `servo_manager_node` - Main servo communication node
- `ping_test` - (Optional) Motor connectivity testing utility

**How to run:**
```bash
# Main servo manager (requires hardware)
ros2 launch llmy_servo_manager servo_manager.launch.py

# Test motor connectivity
ros2 launch llmy_servo_manager ping_test.launch.py

# Custom serial port
ros2 launch llmy_servo_manager servo_manager.launch.py port:=/dev/ttyTHS1 baud:=1000000
```

**Key topics:**
- `/motor_manager/base_cmd` - Base motor velocity commands
- `/motor_manager/arm_cmd` - Arm motor position commands  
- `/motor_manager/head_cmd` - Head motor position commands
- `/motor_manager/joint_states` - Motor telemetry feedback

**Configuration:** Edit `config/servo_manager.yaml` to adjust motor IDs, serial settings, and control parameters.

---

#### **🐍 llmy_servo_manager_py - Python Motor Control**

**What it does:** Python-based alternative to the C++ servo manager with modular architecture, advanced braking system, and improved error handling. Provides the same functionality with enhanced safety features and easier customization.

**Features:**
- **Advanced Braking System**: Three configurable braking methods to prevent power spikes
- **Modular Architecture**: Clean separation of motor management, configuration, and command handling
- **Improved Safety**: Comprehensive connectivity testing and graceful error recovery
- **Enhanced Telemetry**: Detailed motor state monitoring and diagnostics

**Nodes launched:**
- `servo_manager_node.py` - Main Python servo manager node

**How to run:**
```bash
# Main Python servo manager (requires hardware)
ros2 launch llmy_servo_manager_py servo_manager.launch.py

# Test brake methods
ros2 launch llmy_servo_manager_py test_brake_methods.launch.py

# Custom configuration
ros2 launch llmy_servo_manager_py servo_manager.launch.py config_file:=custom_config.yaml
```

**Key topics:** (Same as C++ version)
- `/motor_manager/base_cmd` - Base motor velocity commands
- `/motor_manager/arm_cmd` - Arm motor position commands  
- `/motor_manager/head_cmd` - Head motor position commands
- `/motor_manager/joint_states` - Motor telemetry feedback

**Configuration:** Edit `config/servo_manager.yaml` to adjust motor IDs, serial settings, braking method, and control parameters.

> **💡 When to use:** Choose the Python version for enhanced safety features, easier customization, or when you need advanced braking control. Both versions are fully compatible with the rest of the system.

---

#### **🔌 llmy_control_plugin - Hardware Bridge**

**What it does:** Acts as the ros2_control hardware interface, bridging standard ROS2 controllers with the servo manager. Enables seamless integration with MoveIt2, navigation, and other ROS2 tools.

**Nodes launched:**
- `controller_manager` - ros2_control manager
- `diff_drive_controller` - Base movement controller (skid-steer)
- `arm_controller` - Arm trajectory controller
- `joint_state_broadcaster` - Joint state publisher

**How to run:**
```bash
# Launch with hardware interface
ros2 launch llmy_control_plugin bringup.launch.py

# Simulation mode (requires Gazebo)
ros2 launch llmy_control_plugin bringup.launch.py use_sim_time:=true
```

**Key topics:**
- `/cmd_vel` - Base velocity commands (input)
- `/arm_controller/joint_trajectory` - Arm trajectory commands (input)
- `/joint_states` - Combined joint states (output)
- `/controller_manager/*` - Controller status and management

---

#### **🎮 llmy_teleop_xbox - Manual Control**

**What it does:** Provides intuitive Xbox controller mapping for manual robot operation. Maps controller inputs to robot movements with safety limits and smooth control.

**Nodes launched:**
- `teleop_xbox_node` - Xbox controller interface
- `joy_node` - Joystick driver

**How to run:**
```bash
# Standard Xbox controller
ros2 launch llmy_teleop_xbox teleop_xbox.launch.py

# Custom controller device
ros2 launch llmy_teleop_xbox teleop_xbox.launch.py device:=/dev/input/js1
```

**Controller mapping:**
- **Right stick:** Base movement (forward/back + rotation)
- **RB/LB:** Arm joint 1 (+/-)
- **RT/LT:** Arm joint 2 (+/-)
- **Y/A:** Arm joint 3 (+/-)
- **B/X:** Arm joint 4 (+/-)
- **Start/Back:** Arm joint 5 (+/-)
- **Stick clicks:** Arm joint 6 (+/-)
- **D-pad up/down:** Camera pan
- **D-pad left/right:** Camera tilt

**Key topics:**
- `/cmd_vel` - Base velocity output
- `/arm_controller/joint_trajectory` - Arm movement output
- `/joy` - Raw joystick data

---

#### **⚙️ llmy_control - Controller Configuration**

**What it does:** Provides controller configurations and parameter files for the skid-steer differential drive base and 6-DOF arm using standard ros2_control patterns.

**Configuration files:**
- `config/controllers.yaml` - Controller parameters
- `config/ros2_control.yaml` - Hardware interface config
- `config/joint_limits.yaml` - Safety limits

**Loaded by:** llmy_control_plugin (no standalone launch)

**Key parameters:**
- Velocity limits for base wheels
- Position/velocity limits for arm joints
- Controller gains and dynamics






#### **🗺️ llmy_nav - Navigation & SLAM**

**What it does:** Provides multiple navigation modes for the robot — full map-based navigation with AMCL, map-free reactive navigation, SLAM-only mapping, and combined SLAM + Nav2 for simultaneous mapping and navigation. Includes a mode manager for switching between modes at runtime.

**Navigation modes:**
- **mapping** — Builds a map using slam_toolbox (no autonomous navigation)
- **navigation** — Full Nav2 with a pre-built map (map_server + AMCL)
- **mapfree** — Reactive local navigation without any map
- **slam_nav** — SLAM + Nav2 combined for simultaneous mapping and autonomous navigation

**How to run:**
```bash
# SLAM + Navigation (recommended for exploration)
ros2 launch llmy_nav slam_nav.launch.py

# Map-based navigation (requires a saved map)
ros2 launch llmy_nav navigation.launch.py map:=/path/to/map.yaml

# Build a map only
ros2 launch llmy_nav mapping.launch.py

# Reactive navigation without a map
ros2 launch llmy_nav mapfree.launch.py
```

**Key topics:**
- `/cmd_vel_nav` — Navigation velocity output (via velocity_smoother)
- `/map` — Occupancy grid map
- `/plan` — Global path plan
- `/local_costmap/costmap` — Local obstacle map
- `/global_costmap/costmap` — Global planning map

**Configuration:** Config files in `config/` include controller (MPPI), planner (SmacPlanner2D), costmaps, SLAM toolbox, AMCL, behavior trees, and velocity smoother parameters.

---

#### **🦾 llmy_moveit - Arm Motion Planning**

**What it does:** Provides MoveIt 2 configuration for the SO-ARM101 6-DOF arm + gripper. Enables collision-free trajectory planning, inverse kinematics, and integration with the `arm_trajectory_controller` for precise arm manipulation.

**Nodes launched:**
- `move_group` — MoveIt 2 planning and execution server
- `rviz2_moveit` — (Optional) RViz with MoveIt plugin for interactive planning

**How to run:**
```bash
# Launch MoveIt (hardware)
ros2 launch llmy_moveit moveit.launch.py

# Launch MoveIt (simulation)
ros2 launch llmy_moveit moveit.launch.py use_sim_time:=true

# Without RViz
ros2 launch llmy_moveit moveit.launch.py launch_rviz:=false
```

**Planning groups:**
- `arm` — Joints 1-5 (chain from base to gripper), KDL kinematics, OMPL planners (RRTConnect, RRT, PRM)
- `gripper` — Joint 6

**Named poses:** `home`, `open`, `closed`

**Controller switching:** MoveIt uses `arm_trajectory_controller`. Switch from direct position control:
```bash
ros2 control switch_controller --activate arm_trajectory_controller --deactivate arm_controller
```

**Configuration:** SRDF, kinematics (KDL), joint limits, OMPL planners, and MoveIt controller mappings in `config/`.

---

#### **🤖 llmy_mcp - MCP Bridge for LLMs**

**What it does:** Dynamic Model Context Protocol (MCP) server that bridges ROS 2 to Large Language Models. Automatically discovers all ROS 2 topics and services at runtime, enabling LLMs to read sensor data, publish commands, and call services through natural language.

**Nodes launched:**
- `llmy_mcp_bridge` — ROS 2 node running inside the MCP server process

**How to run:**
```bash
# Default (stdio transport, for local MCP clients like Claude Desktop)
ros2 run llmy_mcp mcp_server

# HTTP server (SSE transport, for remote access)
ros2 run llmy_mcp mcp_server --transport sse --host 0.0.0.0 --port 8765

# With custom config
ros2 run llmy_mcp mcp_server --config config/mcp_server.yaml
```

**MCP tools exposed:**
- `list_topics` / `list_services` — Discover available ROS 2 interfaces
- `subscribe_topic` / `read_topic` — Read sensor data from any topic
- `publish_topic` / `publish_twist_stamped` — Send commands to the robot
- `call_service` — Call any ROS 2 service

**Key topics:**
- `/cmd_vel_mcp` — Default velocity command output (TwistStamped)
- Reads from any topic dynamically (auto-subscribes on demand)

**Configuration:** `config/mcp_server.yaml` controls topic/service filtering, QoS settings, auto-subscriptions, and message cache size.

---

#### **📷 llmy_camera - Vision System**

**What it does:** Integrates RGB-D cameras with compressed image transport and depth-to-laser conversion. Provides both manipulation-ready RGB-D data and navigation-ready 2D laser scans.

**Nodes launched:**
- `realsense2_camera_node` - RealSense D435 driver
- `depthimage_to_laserscan` - Depth to 2D scan converter
- `image_transport` - Compressed image streaming
- `usb_cam_node` - (Optional) Wrist camera driver

**How to run:**
```bash
# Full camera system
ros2 launch llmy_camera camera.launch.py

# RealSense only
ros2 launch llmy_camera realsense.launch.py

# With custom resolution
ros2 launch llmy_camera camera.launch.py width:=1280 height:=720
```

**Key topics:**
- `/head_camera/color/image_raw` - RGB images
- `/head_camera/depth/image_rect_raw` - Depth images
- `/head_camera/rgbd` - Combined RGB-D data
- `/scan` - 2D laser scan from depth
- `/wrist_camera/image_raw` - Wrist camera feed

---

#### **📐 llmy_imu - Orientation Sensing**

**What it does:** Handles ICM20948 9-DOF sensor integration with Madgwick sensor fusion, providing calibrated orientation data crucial for navigation and balance.

**Nodes launched:**
- `imu_filter_madgwick` - Sensor fusion node
- `icm20948_driver` - Raw IMU data driver

**How to run:**
```bash
# IMU with sensor fusion
ros2 launch llmy_imu imu.launch.py

# Raw IMU data only
ros2 run llmy_imu icm20948_driver
```

**Key topics:**
- `/imu/data` - Raw IMU measurements
- `/imu/fused` - Fused orientation estimate
- `/imu/mag` - Magnetometer data

**Calibration:** Run calibration sequence on first setup:
```bash
ros2 run llmy_imu calibrate_imu
```