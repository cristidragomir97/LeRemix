# llmy_gazebo

Gazebo Harmonic simulation environment for the LLMy robot. Spawns the robot with all controllers, sensor bridges, and a furnished apartment world.

## Launch

```bash
# Launch simulation (default: sticky_floor apartment world)
ros2 launch llmy_gazebo gazebo.launch.py

# Empty world
ros2 launch llmy_gazebo gazebo.launch.py world:=empty.sdf
```

## What Gets Launched

1. Robot URDF loaded via xacro (`use_sim:=true`)
2. Gazebo simulation with world file
3. ROS-Gazebo bridges for sensors (lidar, IMU, cameras, clock)
4. Controller spawners: `joint_state_broadcaster`, `diff_drive_controller`, `arm_controller`, `head_controller`
5. `arm_trajectory_controller` loaded inactive (for MoveIt)
6. `twist_mux` for velocity command arbitration

## World Files

| World | Description |
|-------|-------------|
| `sticky_floor.world` | Furnished apartment (~15x12m) with walls, furniture, and obstacles |
| `empty.sdf` | Minimal ground plane with lighting |

## Bridged Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/scan` | LaserScan | Gazebo -> ROS |
| `/sensors/imu` | Imu | Gazebo -> ROS |
| `/head_camera/rgb/*` | Image + CameraInfo | Gazebo -> ROS |
| `/head_camera/depth/*` | Image + CameraInfo | Gazebo -> ROS |
| `/wrist_camera/rgb/*` | Image + CameraInfo | Gazebo -> ROS |
| `/clock` | Clock | Gazebo -> ROS |
