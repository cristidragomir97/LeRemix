# llmy_description

Complete URDF/Xacro robot model for the LLMy mobile manipulator. Provides the "digital twin" used by all other packages for visualization, simulation, and control.

## Robot Structure

- **Base**: Differential drive with 2 powered wheels + 2 passive casters
- **Arm**: 6-DOF (joints 1-6) — base rotation, shoulder, elbow, two wrist joints, gripper
- **Head**: Pan-tilt camera mount with 2 servos
- **Sensors**: RealSense RGB-D (head), USB camera (wrist), RPLidar, IMU

## Launch Files

```bash
# Visualize the robot model in RViz (standalone)
ros2 launch llmy_description display.launch.py

# Spawn in Gazebo (usually called by llmy_gazebo instead)
ros2 launch llmy_description gazebo.launch.py
```

## Key Files

| File | Description |
|------|-------------|
| `urdf/LLMy.xacro` | Main robot model — all links, joints, and sensor frames |
| `urdf/materials.xacro` | Material definitions |
| `urdf/LLMy.gazebo` | Gazebo-specific physics and sensor plugins |
| `urdf/LLMy.ros2control` | ros2_control interfaces for simulation |
| `meshes/*.stl` | 48 STL mesh files for visualization and collision |
| `config/display.rviz` | RViz visualization preset |

## Xacro Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim` | `false` | Load Gazebo ros2_control plugin when `true`, hardware plugin when `false` |
