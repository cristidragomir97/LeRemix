# leremix_description

Robot description (URDF/Xacro, meshes) for the LeRemix mobile manipulation robot. Contains the complete 3D model, kinematics, and visual representation.

## Features

- Complete URDF description of LeRemix robot
- Omnidirectional mobile base with 3 mecanum wheels
- 6-DOF robotic arm with gripper
- Camera with pan/tilt mechanism  
- High-quality STL meshes for all components
- Materials and visual properties
- Collision geometries for simulation

## Components

### Mobile Base
- Omnidirectional drive system
- 3 mecanum wheels (back, left, right)
- Battery pack mounting
- Main chassis and support structure

### Robotic Arm
- 6 degrees of freedom
- Servo-based joints (1-6)
- Gripper end-effector with dual jaws
- Joint limits and dynamics defined

### Camera System
- Pan servo for horizontal rotation
- Tilt servo for vertical adjustment
- Camera mount and housing

### Electronics Housing
- NVIDIA Orin mount
- Battery pack mounts (left/right)
- IMU mounting location
- Cable management

## File Structure

```
leremix_description/
├── urdf/
│   ├── LeRemix.xacro          # Main robot description
│   ├── LeRemix.trans          # Joint transformations
│   └── materials.xacro        # Material definitions
└── meshes/                    # STL mesh files
    ├── base_link.stl          # Main chassis
    ├── *_wheel_*.stl          # Wheel assemblies
    ├── arm_*.stl              # Arm components
    ├── camera_*.stl           # Camera parts
    ├── orin_*.stl            # Computing hardware
    └── ...                    # Additional components
```

## Usage

### View Robot Model

```bash
# Launch robot state publisher with URDF
ros2 launch robot_state_publisher view_robot.launch.py \
  model:=$(ros2 pkg prefix leremix_description)/share/leremix_description/urdf/LeRemix.xacro

# View in RViz
ros2 run rviz2 rviz2
# Add RobotModel display, set Fixed Frame to "base_link"
```

### Use in Launch Files

```python
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable

# Load robot description
xacro_file = PathJoinSubstitution([
    FindPackageShare("leremix_description"),
    "urdf", 
    "LeRemix.xacro"
])

robot_description = {
    "robot_description": Command([
        FindExecutable(name="xacro"), " ", xacro_file
    ])
}

# Use with robot_state_publisher
rsp_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[robot_description],
    output="screen"
)
```

### Generate Static URDF

```bash
# Convert xacro to URDF
xacro $(ros2 pkg prefix leremix_description)/share/leremix_description/urdf/LeRemix.xacro > leremix.urdf

# Check URDF validity
check_urdf leremix.urdf

# Visualize joint tree
urdf_to_graphiz leremix.urdf
```

## Joint Information

### Base Joints
- `back_motor_rotation` - Back wheel rotation
- `left_motor_rotation` - Left wheel rotation  
- `right_motor_rotation` - Right wheel rotation

### Arm Joints
- `1` through `6` - Arm joint servos (base to wrist)
- `camera_tilt` - Camera tilt servo

## Servo Calibration Data

The robot arm servos have been physically calibrated with the arm in a fully erect position. The middle point (2048) was set with all joints at their mechanical neutral position, then joint limits were measured based on encoder counts.

### Joint Calibration Values
| Joint | Physical Servo | Range (Encoder Counts) | Relaxed Position | Description |
|-------|---------------|------------------------|------------------|-------------|
| Joint 1 (pan) | 4 | 1024 - 3072 | 2048 | Full left to full right |
| Joint 2 (shoulder) | 5 | 880 - 3072 | 1024 | Full up to full down |
| Joint 3 (elbow) | 6 | 2024 - 3880 | 3880 | Full up to full down |
| Joint 4 (wrist tilt) | 7 | 922 - 3072 | 909 | Full up to full down |
| Joint 5 (wrist rotation) | 8 | 1372 - 4096 | 2048 | Full left to full right |
| Joint 6 (gripper) | 9 | 2048 - 3446 | 2048 | Closed to open |
| Camera tilt | 11 | 1024 - 2611 | 2048 | Full down to full up |

**Notes:**
- Encoder counts range from 0-4095 (12-bit servos)
- 2048 represents the mechanical center position for most servos
- Relaxed positions represent the robot's natural pose when powered on
- Joint limits in Gazebo simulation are set based on these measured values
- The firmware automatically moves to relaxed positions on startup

## Coordinate Frames

- **base_link** - Robot base reference frame
- **odom** - Odometry frame (typical parent)
- **imu_link** - IMU sensor frame
- **camera_link** - Camera optical frame
- **end_effector** - Arm end-effector frame

## Dependencies

- xacro (URDF processing)
- robot_state_publisher
- Standard ROS2 URDF tools

## Integration

This package is used by:
- **leremix_gazebo** - Simulation environment
- **leremix_control** - ros2_control integration
- **RViz** - Visualization and debugging
- **MoveIt** - Motion planning (if configured)

## Customization

To modify the robot:
1. Edit `LeRemix.xacro` for structural changes
2. Update `materials.xacro` for appearance
3. Replace STL files in `meshes/` for new components
4. Rebuild and test: `colcon build --packages-select leremix_description`