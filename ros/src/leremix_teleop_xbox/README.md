# leremix_teleop_xbox

Xbox controller teleoperation for LeRemix robot, providing intuitive control of both the omnidirectional base and 6-DOF robotic arm with camera tilt.

## Features

- Xbox controller support for mobile base control
- Intuitive arm joint control using analog sticks and triggers
- Camera tilt control via START/BACK buttons
- Configurable movement scales and deadbands
- Real-time joint position control with visual feedback
- Safety features and joint limit handling

## Control Mapping

### Base Movement (Face Buttons)
- **Y Button** - Forward movement
- **A Button** - Backward movement  
- **X Button** - Strafe left
- **B Button** - Strafe right
- **No button pressed** - Stop (explicit stop command)

### Arm Control

#### Joints 1 & 2 (Base Rotation & Shoulder)
- **Left Stick Button (LSB) + Left Stick** - Control joints 1 & 2
- LSB must be pressed to activate left stick for arm control
- X-axis controls joint 1, Y-axis controls joint 2

#### Joint 3 (Elbow)
- **Right Trigger (RT)** - Analog control of joint 3
- **Right Bumper (RB)** - Step increment for joint 3

#### Joint 4 (Wrist 1)
- **Left Trigger (LT)** - Analog control of joint 4  
- **Left Bumper (LB)** - Step increment for joint 4

#### Joints 5 & 6 (Wrist 2 & 3)
- **Right Stick Button (RSB) + Right Stick** - Control joints 5 & 6
- RSB must be pressed to activate right stick for arm control
- X-axis controls joint 5, Y-axis controls joint 6

#### Camera Tilt
- **START Button** - Tilt camera up (+)
- **BACK Button** - Tilt camera down (-)

### Test Mode
- **START + BACK (both pressed)** - Applies small increment to all joints for testing

## Configuration

### Parameters
- `cmd_vel_topic` - Base velocity command topic (default: `/omnidirectional_controller/cmd_vel_unstamped`)
- `arm_cmd_topic` - Arm command topic (default: `/arm_group_position_controller/commands`)
- `linear_scale` - Forward/backward speed scaling (default: 0.6)
- `lateral_scale` - Strafe speed scaling (default: 0.6)
- `arm_increment` - Analog stick increment per update (default: 0.03)
- `arm_button_step` - Button press increment (default: 0.1)
- `arm_deadband` - Analog stick deadband threshold (default: 0.25)
- `arm_rate` - Arm command publishing rate in Hz (default: 50.0)

### Joint Order
The arm commands are published as Float64MultiArray in this order:
`['1', '2', '3', '4', '5', '6', 'camera_tilt']`

## Usage

### Hardware Setup
1. Connect Xbox controller to computer via USB or wireless adapter
2. Verify controller is detected: `ls /dev/input/js*`

### Launch Teleoperation

```bash
# Start joy node for Xbox controller
ros2 run joy joy_node

# Launch teleop node
ros2 launch leremix_teleop_xbox teleop_xbox.launch.py

# Or run node directly with custom parameters
ros2 run leremix_teleop_xbox teleop_xbox.py --ros-args \
  -p linear_scale:=0.8 \
  -p arm_increment:=0.05
```

### Launch File Usage

```python
# In your launch file
teleop_node = Node(
    package='leremix_teleop_xbox',
    executable='teleop_xbox.py',
    parameters=[{
        'linear_scale': 0.5,
        'arm_increment': 0.02,
        'arm_deadband': 0.3,
    }]
)
```

### Verify Operation

```bash
# Monitor base commands
ros2 topic echo /omnidirectional_controller/cmd_vel_unstamped

# Monitor arm commands
ros2 topic echo /arm_group_position_controller/commands

# Check joy messages
ros2 topic echo /joy

# View current joint positions
ros2 topic echo /joint_states
```

## Safety Features

- **Explicit Stop**: Publishes stop command when no movement buttons pressed
- **Deadband**: Prevents accidental movement from controller drift
- **Incremental Control**: Smooth, controlled joint movements
- **Button Guards**: Stick buttons must be pressed for arm control
- **Rate Limiting**: Controlled update frequency prevents overwhelming system

## Troubleshooting

### Controller Not Detected
```bash
# Check if controller is connected
ls /dev/input/js*

# Test controller input
ros2 run joy joy_enumerate_devices
ros2 topic echo /joy
```

### Robot Not Responding
```bash
# Verify topics match controller configuration
ros2 topic list | grep -E "(cmd_vel|commands)"

# Check if controllers are active
ros2 control list_controllers

# Monitor teleop node output
ros2 run leremix_teleop_xbox teleop_xbox.py --ros-args --log-level debug
```

### Arm Moving Unexpectedly
- Increase `arm_deadband` parameter to reduce sensitivity
- Check controller calibration
- Verify joint limits in robot configuration

### Base Not Moving Smoothly
- Adjust `linear_scale` and `lateral_scale` parameters
- Check omnidirectional controller configuration
- Verify wheel directions and scaling

## Dependencies

- rclpy
- sensor_msgs (Joy messages)
- geometry_msgs (Twist messages)  
- std_msgs (Float64MultiArray)
- joy (ROS2 joystick driver)

## Integration

This package works with:
- **leremix_control** - Robot controller configuration
- **leremix_gazebo** - Simulation environment
- **Hardware controllers** - Real robot control via leremix_control_plugin
- **Joy node** - Xbox controller driver

## Customization

### Adding New Buttons
1. Define button index in `__init__`
2. Add button check in `on_joy` callback
3. Implement desired action
4. Update this README with new mapping

### Changing Control Scheme
1. Modify button/axis mappings in `__init__`
2. Update `on_joy` callback logic
3. Adjust parameters as needed
4. Test thoroughly before deployment