# llmy_control_plugin

A `ros2_control` **SystemInterface** hardware plugin that bridges ros2_control to the LLMy servo manager via ROS2 topics.

## Purpose

This plugin implements the hardware abstraction layer for ros2_control, allowing standard controllers to work with LLMy hardware. It uses a topic-based interface rather than direct serial communication, which decouples the control stack from hardware details.

## Topic Interface

### Published (Commands to Hardware)

- **`/motor_manager/base_cmd`** - `std_msgs/Float64MultiArray`
  - `[back_motor_rotation, left_motor_rotation, right_motor_rotation]` (rad/s)
  - QoS: Best-effort, depth 1

- **`/motor_manager/arm_cmd`** - `std_msgs/Float64MultiArray`
  - `[joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]` (radians)
  - QoS: Best-effort, depth 1

- **`/motor_manager/head_cmd`** - `std_msgs/Float64MultiArray`
  - `[camera_pan, camera_tilt]` (radians)
  - QoS: Best-effort, depth 1

### Subscribed (Feedback from Hardware)

- **`/motor_manager/joint_states`** - `sensor_msgs/JointState`
  - All joint positions and velocities
  - QoS: Reliable, depth 5

## Hardware Interfaces Exported

The plugin exports these command and state interfaces to ros2_control:

**Base (velocity interfaces):**
- `back_motor_rotation/velocity`
- `left_motor_rotation/velocity`
- `right_motor_rotation/velocity`

**Arm (position interfaces):**
- `1/position`, `2/position`, `3/position`, `4/position`, `5/position`, `6/position`

**Head (position interfaces):**
- `camera_pan/position`, `camera_tilt/position`

## Configuration

### Hardware Interface Config

Configuration is in `llmy_control/config/ros2_control_bridge.yaml`:

```yaml
hardware_interface:
  plugin: "llmy_control_plugin/ROS2ControlBridge"
  base_joints: [back_motor_rotation, left_motor_rotation, right_motor_rotation]
  arm_joints: ["1", "2", "3", "4", "5", "6"]
  head_joints: [camera_pan, camera_tilt]
```

## Usage

This plugin is loaded automatically by `llmy_control/control_stack.launch.py`. You don't typically interact with it directly.

### Normal Usage

```bash
# Launch complete system (includes this plugin)
ros2 launch llmy_control control_stack.launch.py
```

### Standalone Testing

```bash
# 1. Start servo manager
ros2 launch llmy_servo_manager servo_manager.launch.py

# 2. Launch plugin (requires robot_description)
ros2 launch llmy_control_plugin bringup.launch.py
```

## Implementation Notes

### Lifecycle

1. **`on_init()`** - Read configuration, register joint interfaces
2. **`on_configure()`** - Create publishers and subscribers
3. **`on_activate()`** - Ready for control
4. **`read()`** - Update state interfaces from `/motor_manager/joint_states` (called at update rate)
5. **`write()`** - Publish commands from command interfaces (called at update rate)