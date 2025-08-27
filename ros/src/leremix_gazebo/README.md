# leremix_gazebo

Gazebo simulation package for the LeRemix robot, providing a complete simulation environment with physics, sensors, and ros2_control integration.

## Features

- Complete Gazebo simulation of LeRemix robot
- Physics-based omnidirectional drive simulation
- Simulated arm dynamics and control
- Custom world environments
- ros2_control hardware interface for simulation
- Sensor simulation (IMU, cameras, etc.)

## Components

### Simulation Launch
- **sim.launch.py** - Main simulation launcher
- Loads robot URDF with Gazebo plugins
- Spawns controllers automatically
- Configurable GUI and world options

### Gazebo Configuration
- **gazebo_configs.xacro** - Basic Gazebo settings
- **gazebo_interfaces.xacro** - Hardware interfaces for simulation  
- **gazebo_plugins.xacro** - Sensor and actuator plugins
- **leremix_gazebo_overlay.xacro** - Complete simulation overlay

### World Files
- **worlds/sticky_floor.world** - Testing environment with high friction
- Custom world for robot testing and validation

## Usage

### Launch Simulation

```bash
# Launch full simulation (includes GUI)
ros2 launch leremix_gazebo sim.launch.py

# Launch without GUI
ros2 launch leremix_gazebo sim.launch.py gui:=false

# Launch with custom spawn position
ros2 launch leremix_gazebo sim.launch.py spawn_z:=0.1

# Launch paused (for setup)
ros2 launch leremix_gazebo sim.launch.py pause:=true
```

### Launch Arguments

- `gui` - Show Gazebo GUI (default: true)
- `pause` - Start simulation paused (default: false)  
- `spawn_z` - Robot spawn height (default: 0.05)
- `cm_timeout` - Controller manager timeout (default: 60)

### Control the Robot

Once simulation is running, controllers are automatically spawned:

```bash
# Check active controllers
ros2 control list_controllers

# Send velocity commands to base
ros2 topic pub /omnidirectional_controller/cmd_vel_unstamped geometry_msgs/Twist \
  "{linear: {x: 0.5, y: 0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"

# Send arm position commands  
ros2 topic pub /arm_controller/commands std_msgs/Float64MultiArray \
  "{data: [0.0, 0.5, -0.5, 0.0, 0.0, 0.0, 0.0]}"

# Use Xbox controller for teleoperation
ros2 launch leremix_teleop_xbox teleop_xbox.launch.py
```

### Monitor Robot State

```bash
# View joint states
ros2 topic echo /joint_states

# Monitor base odometry (if enabled)
ros2 topic echo /odom

# View robot in RViz
ros2 run rviz2 rviz2
```

## Simulation Features

### Physics Integration
- Accurate omnidirectional drive dynamics
- Realistic arm joint physics and limits
- Collision detection and response
- Gravity and inertia simulation

### Hardware Simulation
- ros2_control `gazebo_ros2_control` plugin
- Simulated position and velocity interfaces
- Joint state feedback
- Effort/torque simulation

### Sensor Simulation
- IMU sensor simulation
- Camera simulation capabilities
- Joint encoders and feedback

## World Environments

### Default World
Uses TurtleBot3 house world for testing navigation and manipulation tasks.

### Sticky Floor World
- High friction surface for testing drive capabilities
- Prevents wheel slippage during testing
- Located: `worlds/sticky_floor.world`

## Troubleshooting

### Controllers Not Starting
```bash
# Check controller manager status
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers

# Manually spawn controllers if needed
ros2 run controller_manager spawner omnidirectional_controller
ros2 run controller_manager spawner arm_controller
```

### Robot Not Responding
```bash
# Verify topics are active
ros2 topic list | grep -E "(cmd_vel|commands)"

# Check joint states
ros2 topic echo /joint_states --once

# Restart simulation
ros2 launch leremix_gazebo sim.launch.py
```

### Performance Issues
- Reduce physics update rate in world file
- Disable GUI: `gui:=false`
- Use simpler collision meshes
- Reduce controller update rates

## Dependencies

- gazebo_ros
- gazebo_ros2_control  
- robot_state_publisher
- xacro
- leremix_description
- leremix_control
- turtlebot3_gazebo (for default world)

## Integration

This simulation works with:
- **leremix_teleop_xbox** - Xbox controller teleoperation
- **leremix_control** - Controller configurations
- **Navigation stack** - SLAM and autonomous navigation
- **MoveIt** - Arm motion planning
- **RViz** - Visualization and monitoring

## Customization

### Adding Sensors
1. Edit `gazebo_plugins.xacro`
2. Add sensor plugin configuration
3. Update robot URDF if needed

### Custom Worlds  
1. Create `.world` file in `worlds/`
2. Update launch file world path
3. Test with `gazebo worlds/your_world.world`