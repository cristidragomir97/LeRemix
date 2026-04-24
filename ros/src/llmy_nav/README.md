# LLMy Navigation Package

Nav2 configuration and mode management for the LLMy robot.

## Quick Start

```bash
# Build the package
colcon build --packages-select llmy_nav
source install/setup.bash

# Launch navigation with a map
ros2 launch llmy_nav navigation.launch.py map:=/path/to/map.yaml

# Launch SLAM for mapping
ros2 launch llmy_nav mapping.launch.py

# Launch mapfree navigation (no localization)
ros2 launch llmy_nav mapfree.launch.py

# Launch the mode manager (orchestrates mode switching)
ros2 launch llmy_nav mode_manager.launch.py
```

## Navigation Modes

### Navigation Mode
Full autonomous navigation using a pre-built map:
- Map server provides static occupancy grid
- AMCL localizes the robot in the map
- Nav2 plans and executes paths
- Recovery behaviors handle stuck situations

### Mapping Mode
SLAM for building new maps:
- SLAM Toolbox builds map in real-time
- Teleoperate the robot to explore
- Save maps via service call

### Mapfree Mode
Local navigation without a map:
- Identity transform from map to odom
- Rolling window costmaps
- Good for teleoperation with obstacle avoidance

## Configuration Files

| File | Purpose |
|------|---------|
| `planner.yaml` | SmacPlanner2D global path planning |
| `controller.yaml` | MPPI controller for trajectory following |
| `costmap.yaml` | Global/local costmaps for map-based navigation |
| `costmap_mapfree.yaml` | Costmaps for mapfree mode |
| `amcl.yaml` | AMCL localization parameters |
| `bt_navigator.yaml` | Behavior tree navigator config |
| `behavior.yaml` | Recovery behaviors (spin, backup, wait) |
| `velocity_smoother.yaml` | Velocity command smoothing |
| `slam_toolbox.yaml` | SLAM Toolbox configuration |

## Robot Parameters

The configuration is tuned for the LLMy robot:
- Wheel separation: 0.176m
- Wheel radius: 0.025m
- Max linear velocity: 0.5 m/s (conservative)
- Max angular velocity: 1.0 rad/s
- Footprint: 30cm x 20cm rectangle

## Map Storage

Maps are stored in `~/.llmy/maps/` with the format:
- `mapname.yaml` - Map metadata
- `mapname.pgm` - Occupancy grid image

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/nav/change_mode` | Trigger | Cycle through modes (Phase 1) |
| `/nav/save_map` | Trigger | Save current SLAM map |
| `/nav/stop` | Trigger | Stop current mode |

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/nav/mode` | String | Current mode |
| `/nav/maps` | String (JSON) | Available maps |
| `/nav/current_map` | String | Active map name |

## Phase 2 Roadmap

Phase 2 will migrate from subprocess spawning to lifecycle management:
- Faster mode switching (< 1 second vs 5+ seconds)
- Built-in health monitoring
- Graceful state transitions
- Custom service types for proper API

See `SPEC.md` for the full specification.
