# llmy_slam

SLAM (Simultaneous Localization and Mapping) integration for the LLMy robot using [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox).

This package provides custom launch files and configurations specifically tailored for the LLMy robot's hardware setup.

## Overview

This package wraps slam_toolbox with LLMy-specific configurations for:
- **Sensor input**: RPLidar on `/scan` topic
- **Odometry**: Omnidirectional controller publishing `/omnidirectional_controller/odom` and TF `odom` → `base_link`
- **Base frame**: `base_link`
- **Map frame**: `map`

## Prerequisites

1. **Install slam_toolbox**:
   ```bash
   sudo apt install ros-${ROS_DISTRO}-slam-toolbox
   ```

2. **Ensure RPLidar is publishing** on `/scan` topic:
   - Enable RPLidar in your bringup configuration
   - Or use depth-to-laserscan from the RealSense camera

3. **Verify odometry is being published**:
   ```bash
   ros2 topic echo /omnidirectional_controller/odom
   ```

   Note: slam_toolbox uses TF transforms (`odom` → `base_link`) rather than subscribing to the odometry topic directly.

## Usage

### 1. Mapping Mode (Creating a Map)

Launch slam_toolbox in online asynchronous mapping mode:

```bash
ros2 launch llmy_slam online_async_launch.py
```

**Options:**
- `use_sim_time:=true` - Use simulation time (for Gazebo)
- `params_file:=/path/to/custom/params.yaml` - Use custom parameters

**While mapping:**
- Drive the robot around using teleop or xbox controller
- The map will be built in real-time
- You can view the map in RViz by adding the `/map` topic

**To save the map:**

Use the slam_toolbox service:
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/path/to/your/map_name'}}"
```

This will create two files:
- `map_name.posegraph` - SLAM graph data
- `map_name.data` - Serialized map data

### 2. Localization Mode (Using a Pre-built Map)

Once you have a saved map, you can use it for localization only:

1. **Update the map path** in `config/mapper_params_localization.yaml`:
   ```yaml
   map_file_name: /path/to/your/saved/map_name
   ```

2. **Launch localization**:
   ```bash
   ros2 launch llmy_slam localization_launch.py
   ```

In this mode, the robot will localize itself within the existing map without modifying it.

### 3. Lifelong Mode (Dynamic Mode Switching) ⭐ **Recommended**

Lifelong mode is the most flexible approach, allowing you to **dynamically switch between mapping and localization** using ROS services without restarting the node.

**Launch lifelong mode:**
```bash
ros2 launch llmy_slam lifelong_launch.py
```

**Options:**
- `use_sim_time:=true` - Use simulation time (for Gazebo)
- `params_file:=/path/to/custom/params.yaml` - Use custom parameters

#### Dynamic Mode Switching with Services

Once running, you can control the SLAM behavior using these services:

**1. Pause Mapping (Switch to Localization Mode):**
```bash
ros2 service call /slam_toolbox/pause_new_measurements std_srvs/srv/SetBool "{data: true}"
```

**2. Resume Mapping:**
```bash
ros2 service call /slam_toolbox/pause_new_measurements std_srvs/srv/SetBool "{data: false}"
```

**3. Save the Current Map (while running):**
```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/path/to/map_name'}"
```

**4. Load an Existing Map (while running):**
```bash
ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph "{filename: '/path/to/map_name', match_type: 2}"
```
- `match_type: 1` - Start at the origin of the loaded map
- `match_type: 2` - Start at the current position (relocalize in the map)

**5. Clear the Current Queue:**
```bash
ros2 service call /slam_toolbox/clear_queue std_srvs/srv/Empty
```

#### Example Workflow with Lifelong Mode

```bash
# 1. Launch lifelong SLAM
ros2 launch llmy_slam lifelong_launch.py

# 2. Drive around and build a map
# ... (use teleop/xbox controller)

# 3. Save the map
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/user/maps/office_map'}"

# 4. Switch to localization mode (pause mapping)
ros2 service call /slam_toolbox/pause_new_measurements std_srvs/srv/SetBool "{data: true}"

# 5. Later, resume mapping to update the map
ros2 service call /slam_toolbox/pause_new_measurements std_srvs/srv/SetBool "{data: false}"

# 6. Or load a different map entirely
ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph \
  "{filename: '/home/user/maps/warehouse_map', match_type: 2}"
```

#### Advantages of Lifelong Mode

- **No restarts needed** - Switch modes on the fly
- **Flexible workflows** - Map, localize, then continue mapping
- **Multi-session mapping** - Save and continue mapping across multiple sessions
- **Hot-swapping maps** - Load different maps without stopping the node

## Configuration Files

### `config/mapper_params_lifelong.yaml`
Configuration for lifelong mode with dynamic behavior:
- Supports both mapping and localization
- Can load existing maps and continue mapping
- Full service interface for dynamic control
- Recommended for production deployments

### `config/mapper_params_online_async.yaml`
Configuration for online mapping mode. Key parameters:
- `scan_topic: /scan` - Laser scan input
- `odom_frame: odom` - Odometry frame
- `base_frame: base_link` - Robot base frame
- `map_frame: map` - Map frame
- `max_laser_range: 12.0` - RPLidar typical max range
- `resolution: 0.05` - Map resolution (5cm per pixel)
- `minimum_travel_distance: 0.2` - Minimum movement before updating (meters)

### `config/mapper_params_localization.yaml`
Configuration for localization mode. Similar to mapping mode but with:
- `mode: localization` - Only localizes, doesn't update map
- `map_file_name: /path/to/map` - Path to pre-built map

## RViz Visualization

To visualize SLAM in RViz:

1. Launch RViz:
   ```bash
   rviz2
   ```

2. Add the following displays:
   - **TF** - To see coordinate frames
   - **Map** - Topic: `/map`
   - **LaserScan** - Topic: `/scan`
   - **RobotModel** - Uses the URDF description

3. Set **Fixed Frame** to `map`

## Integration with LLMy Bringup

To automatically start SLAM with your robot, you can include this launch file in `llmy_bringup`:

```python
from launch.actions import IncludeLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Option 1: Lifelong mode (recommended for flexibility)
slam_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare('llmy_slam'),
            'launch',
            'lifelong_launch.py'
        ])
    )
)

# Option 2: Online mapping mode (simpler, mapping only)
# slam_launch = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource(
#         PathJoinSubstitution([
#             FindPackageShare('llmy_slam'),
#             'launch',
#             'online_async_launch.py'
#         ])
#     )
# )
```

## Topics

### Subscribed:
- `/scan` (sensor_msgs/LaserScan) - Laser scan from RPLidar
- `/tf` - Transform tree (requires `odom` → `base_link` transform)
- `/tf_static` - Static transforms

**Note**: slam_toolbox does not subscribe to `/omnidirectional_controller/odom` directly. It uses the TF transform `odom` → `base_link` published by the omnidirectional controller.

### Published:
- `/map` (nav_msgs/OccupancyGrid) - Generated map
- `/tf` - Publishes `map` → `odom` transform

## Services

slam_toolbox provides many useful services:

### Core Services

- `/slam_toolbox/pause_new_measurements` - **Pause/resume mapping** (switch to localization mode)
- `/slam_toolbox/serialize_map` - **Save map to file** (posegraph format)
- `/slam_toolbox/deserialize_map` - **Load map from file** and optionally relocalize
- `/slam_toolbox/save_map` - Save map as standard ROS map format (.pgm + .yaml)
- `/slam_toolbox/clear_queue` - Clear the scan queue

### Additional Services

- `/slam_toolbox/toggle_interactive_mode` - Enable/disable interactive markers in RViz
- `/slam_toolbox/clear` - Clear the current graph (start fresh)
- `/slam_toolbox/dynamic_map` - Get the current map
- `/slam_toolbox/manual_loop_closure` - Manually trigger loop closure

List all available services:
```bash
ros2 service list | grep slam_toolbox
```

Get service details:
```bash
ros2 service type /slam_toolbox/pause_new_measurements
ros2 interface show std_srvs/srv/SetBool
```

### Quick Service Reference

**Save map (posegraph format, can be loaded back):**
```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/user/maps/my_map'}"
```

**Save map (standard ROS format, for navigation stack):**
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/home/user/maps/my_map'}}"
```

**Load map and relocalize:**
```bash
ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph \
  "{filename: '/home/user/maps/my_map', match_type: 2}"
```

**Pause mapping (localization only):**
```bash
ros2 service call /slam_toolbox/pause_new_measurements std_srvs/srv/SetBool "{data: true}"
```

**Resume mapping:**
```bash
ros2 service call /slam_toolbox/pause_new_measurements std_srvs/srv/SetBool "{data: false}"
```

### Helper Script

For convenience, a helper script is provided to simplify common operations:

```bash
# After building the package, the script will be available at:
ros2 pkg prefix llmy_slam
# Or use it directly from source:
./ros/src/llmy_slam/scripts/slam_control.sh

# Usage examples:
./scripts/slam_control.sh pause                         # Pause mapping
./scripts/slam_control.sh resume                        # Resume mapping
./scripts/slam_control.sh save /path/to/map            # Save map
./scripts/slam_control.sh save_ros /path/to/map        # Save as ROS format
./scripts/slam_control.sh load /path/to/map            # Load map
./scripts/slam_control.sh list                         # List all services
```

## Troubleshooting

### No map appearing
- Check that `/scan` topic is publishing: `ros2 topic echo /scan`
- Verify TF tree is complete: `ros2 run tf2_tools view_frames`
- Check that odometry is being published: `ros2 topic echo /omnidirectional_controller/odom`
- Verify `odom` → `base_link` TF is published: `ros2 run tf2_ros tf2_echo odom base_link`

### Poor map quality
- Adjust `minimum_travel_distance` and `minimum_travel_heading` parameters
- Increase `loop_closure` parameters for better loop detection
- Ensure your laser scan data is clean and not noisy

### Transform errors
- Verify that `odom` → `base_link` transform is being published by omnidirectional_controller:
  ```bash
  ros2 run tf2_ros tf2_echo odom base_link
  ```
- Check that all sensor frames are properly defined in the URDF
- Use `ros2 run tf2_tools view_frames` to debug TF tree
- Ensure `/omnidirectional_controller/odom` topic is publishing

## Parameters Tuning

Key parameters you might want to adjust based on your environment:

- **For small environments** (< 10m x 10m):
  - Decrease `loop_search_maximum_distance`
  - Increase `minimum_travel_distance`

- **For large environments** (> 50m x 50m):
  - Increase `loop_search_maximum_distance`
  - Decrease `minimum_travel_distance`
  - Increase `stack_size_to_use`

- **For fast-moving robots**:
  - Decrease `minimum_time_interval`
  - Increase `transform_publish_period`

## License

MIT

## References

- [slam_toolbox documentation](https://github.com/SteveMacenski/slam_toolbox)
- [slam_toolbox ROS2 tutorial](https://navigation.ros.org/tutorials/docs/using_slam_toolbox.html)
