# llmy_control

Controller configurations and launch files for the LLMy robot's motion subsystem. Manages differential drive, 6-DOF arm, and pan-tilt head through ros2_control.

## Launch Files

| Launch File | Description |
|-------------|-------------|
| `motion.launch.py` | Main entry point — launches servo manager + control stack |
| `control_stack.launch.py` | Robot state publisher, controller manager, controller spawners, twist_mux |
| `base_systems.launch.py` | Servo manager for low-level motor communication |

```bash
# Full motion subsystem
ros2 launch llmy_control motion.launch.py

# Control stack only (servo manager already running)
ros2 launch llmy_control control_stack.launch.py
```

## Config Files

| File | Description |
|------|-------------|
| `controllers.hw.yaml` | Hardware controller definitions (50Hz, real hardware) |
| `controllers.sim.yaml` | Simulation controller definitions (100Hz, sim time) |
| `twist_mux.yaml` | Velocity multiplexer — xbox (pri 50), mcp (pri 30), nav (pri 10) |
| `ros2_control_bridge.yaml` | Hardware interface plugin config for servo_manager topics |

## Controllers

| Controller | Type | State |
|-----------|------|-------|
| `joint_state_broadcaster` | JointStateBroadcaster | Active |
| `diff_drive_controller` | DiffDriveController | Active |
| `arm_controller` | JointGroupPositionController | Active |
| `arm_trajectory_controller` | JointTrajectoryController | Inactive (for MoveIt) |
| `head_controller` | JointGroupPositionController | Active |

Switch to trajectory control for MoveIt:
```bash
ros2 control switch_controller --activate arm_trajectory_controller --deactivate arm_controller
```

## URDF

`urdf/LLMy.hardware.xacro` defines the ros2_control hardware interface using the `llmy_control_plugin/ROS2ControlBridge` plugin, which communicates with the servo manager via ROS topics.
