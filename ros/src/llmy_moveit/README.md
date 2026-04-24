# llmy_moveit

MoveIt 2 configuration for the LLMy robot's 6-DOF arm (SO-ARM101) and gripper. Provides collision-free trajectory planning, inverse kinematics, and integration with the ros2_control trajectory controller.

## Launch

```bash
# Hardware
ros2 launch llmy_moveit moveit.launch.py

# Simulation
ros2 launch llmy_moveit moveit.launch.py use_sim_time:=true

# Without RViz
ros2 launch llmy_moveit moveit.launch.py launch_rviz:=false
```

## Prerequisites

The control stack must already be running. MoveIt uses `arm_trajectory_controller`, which is loaded inactive by default. Activate it:

```bash
ros2 control switch_controller --activate arm_trajectory_controller --deactivate arm_controller
```

## Planning Groups

| Group | Joints | Description |
|-------|--------|-------------|
| `arm` | 1, 2, 3, 4, 5 | Kinematic chain from base to gripper |
| `gripper` | 6 | Gripper open/close |

## Named Poses

| Pose | Description |
|------|-------------|
| `home` | All joints at zero |
| `open` | Gripper fully open |
| `closed` | Gripper closed |

## Config Files

| File | Description |
|------|-------------|
| `llmy.srdf` | Planning groups, named poses, collision disable pairs |
| `kinematics.yaml` | KDL solver (0.005 resolution, 0.05s timeout) |
| `joint_limits.yaml` | Velocity/acceleration limits and scaling factors |
| `ompl_planning.yaml` | OMPL planners: RRTConnect, RRT, PRM |
| `moveit_controllers.yaml` | Maps MoveIt to ros2_control controllers |
