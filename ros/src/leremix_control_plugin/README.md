# leremix_control_plugin

A `ros2_control` **SystemInterface** that bridges controller commands to topics and reads joint states from a topic.  
Designed for an ESP32 running **micro-ROS**:

- Publishes base wheel velocities (rad/s) to `/esp32/base_cmd` as `std_msgs/Float64MultiArray`
- Publishes arm joint targets (rad) to `/esp32/arm_cmd` as `std_msgs/Float64MultiArray`
- Subscribes to `/esp32/joint_states` (`sensor_msgs/JointState`) for positions (and optional velocities)

## Build

```bash
cd ~/ros2_ws/src
unzip /path/to/leremix_control_plugin.zip -d .
colcon build --packages-select leremix_control_plugin
source ~/ros2_ws/install/setup.bash
```

## Run

1) Start your micro-ROS Agent (on host):
```bash
# Example: rmw_microxrcedds agent
micro-ros-agent udp4 --port 8888
```

2) Launch controller manager and spawn controllers:
```bash
ros2 launch leremix_control_plugin bringup.launch.py
```

This loads:
- `config/controllers.yaml` (your controller settings)
- `config/ros2_control_esp32_bridge.yaml` (hardware plugin config)

## Topic contracts

- `/esp32/base_cmd` — `std_msgs/Float64MultiArray`
  - index 0: `back_motor_rotation` (rad/s)
  - index 1: `left_motor_rotation` (rad/s)
  - index 2: `right_motor_rotation` (rad/s)

- `/esp32/arm_cmd` — `std_msgs/Float64MultiArray`
  - indices follow: `["1","2","3","4","5","6","camera_tilt"]` in that order (radians)

- `/esp32/joint_states` — `sensor_msgs/JointState`
  - `name[]` must contain any/all of the joints above
  - fill `position[]` (radians). For base wheels, fill `velocity[]` if available.

## Notes

- QoS: publishers use **best-effort** depth 1 (low latency). Subscriber uses **reliable** depth 5; adjust to best-effort if your link drops.
- The plugin runs inside `controller_manager` and spins a tiny executor in `read()` to keep state responsive without blocking.

