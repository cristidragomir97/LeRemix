# LLMy Navigation Package Specification

Based on analysis of the Maurice Nav package (Report 17), this specification outlines a navigation package for the LLMy robot that adopts the **good patterns** while avoiding the **known issues**.

---

## Executive Summary

Create `llmy_nav` - a Nav2 configuration and orchestration package that provides:
- Three navigation modes: **mapping**, **navigation**, **mapfree**
- **Lifecycle-based** mode switching (not subprocess spawning)
- Comprehensive Nav2 configuration tuned for the LLMy robot
- Map management with persistence
- MCP integration for LLM control

---

## Package Structure

```
llmy_nav/
├── llmy_nav/
│   └── mode_manager.py           # Lifecycle-based mode orchestration
├── launch/
│   ├── nav_bringup.launch.py     # Launch all nav nodes (inactive)
│   ├── mode_manager.launch.py    # Mode manager node
│   ├── navigation.launch.py      # (Optional) Standalone navigation
│   ├── mapping.launch.py         # (Optional) Standalone mapping
│   └── mapfree.launch.py         # (Optional) Standalone mapfree
├── config/
│   ├── planner.yaml              # SmacPlanner2D configuration
│   ├── controller.yaml           # MPPI controller configuration
│   ├── costmap.yaml              # Global/local costmaps (map-based)
│   ├── costmap_mapfree.yaml      # Costmaps without static map
│   ├── amcl.yaml                 # Localization parameters
│   ├── bt_navigator.yaml         # Behavior tree navigator
│   ├── behavior.yaml             # Recovery behaviors
│   ├── velocity_smoother.yaml    # Velocity smoothing
│   ├── slam_toolbox.yaml         # SLAM configuration
│   └── modes.yaml                # Mode definitions and profiles
├── behavior_trees/
│   ├── nav_to_pose.xml           # Navigate to single pose
│   ├── nav_through_poses.xml     # Waypoint following
│   └── compute_path.xml          # Path computation only
├── maps/                         # Default maps directory
│   └── .gitkeep
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## Navigation Modes

### Mode 1: Mapping (SLAM)

**Purpose**: Build a map of the environment

**Components**:
- `slam_toolbox` (async mode)
- **No AMCL** - SLAM provides map→odom transform
- **No map_server** - Map being built
- Optional: planner/controller for assisted teleop

**Key Outputs**:
- `/map` - Occupancy grid being built
- `/mapping_pose` - Robot pose in map frame
- Saved maps to `~/llmy/maps/`

### Mode 2: Navigation (Map-Based)

**Purpose**: Autonomous navigation using pre-built map

**Components**:
- `map_server` - Serves static map
- `amcl` - Particle filter localization
- `planner_server` - Global path planning (SmacPlanner2D)
- `controller_server` - Local control (MPPI)
- `bt_navigator` - Behavior tree execution
- `behavior_server` - Recovery behaviors (spin, backup, wait)
- `velocity_smoother` - Command smoothing

**Data Flow**:
```
Goal → BT Navigator → Planner → Controller → Velocity Smoother → /cmd_vel
                        ↑           ↑
                     Costmaps ← /scan (LiDAR/depth)
                        ↑
                   AMCL ← /map (map_server)
```

### Mode 3: Mapfree (Local Navigation)

**Purpose**: Reactive navigation without a map

**Components**:
- `static_transform_publisher` - Identity map→odom TF
- `planner_server` - Local planning only
- `controller_server` - MPPI controller
- `bt_navigator` - Behavior tree
- `behavior_server` - Recovery behaviors
- **No AMCL** - Uses odometry directly
- **No map_server** - Rolling window costmaps

**Use Case**: Teleoperation with obstacle avoidance, unknown environments

---

## Configuration Details

### Robot Parameters (LLMy-Specific)

```yaml
# From llmy_control/controllers.hw.yaml
robot:
  wheel_separation: 0.176      # meters (center to center)
  wheel_radius: 0.025          # meters
  max_linear_velocity: 1.0     # m/s
  max_angular_velocity: 2.0    # rad/s
  base_frame: base_link
  odom_frame: odom
```

### Robot Footprint

```yaml
# Approximate rectangular footprint
# TODO: Measure actual LLMy dimensions
footprint: "[[0.15, 0.10], [0.15, -0.10], [-0.15, -0.10], [-0.15, 0.10]]"
# 30cm x 20cm rectangle
```

### Planner (SmacPlanner2D)

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner/SmacPlanner2D"
      tolerance: 0.5                    # Goal tolerance (meters)
      max_planning_time: 2.0            # Timeout (seconds)
      allow_unknown: false              # Don't plan through unknown
      cost_travel_multiplier: 2.0       # Prefer low-cost paths
      smoother:
        max_iterations: 1000
        w_smooth: 0.3
        w_data: 0.2
```

### Controller (MPPI)

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"

      # Prediction horizon
      time_steps: 50
      model_dt: 0.05                    # 2.5 second lookahead

      # Motion model
      motion_model: "DiffDrive"

      # Velocity limits (conservative for safety)
      vx_min: -0.2
      vx_max: 0.5                       # Half of max for indoor
      vy_max: 0.0                       # Differential drive
      wz_max: 1.0

      # Acceleration limits
      ax_max: 0.3
      ax_min: -0.3
      az_max: 0.5

      # Sampling
      batch_size: 1000
      vx_std: 0.2
      wz_std: 0.4

      # Goal tolerance
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.1
```

### Costmaps

```yaml
# Global costmap (for planning)
global_costmap:
  ros__parameters:
    update_frequency: 5.0
    publish_frequency: 2.0
    global_frame: map
    robot_base_frame: base_link
    resolution: 0.05                    # 5cm cells
    rolling_window: false
    footprint: "[[0.15, 0.10], [0.15, -0.10], [-0.15, -0.10], [-0.15, 0.10]]"
    plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      observation_sources: scan
      scan:
        topic: /scan
        max_obstacle_height: 2.0
        raytrace_max_range: 3.0
        obstacle_max_range: 2.5

    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 10.0
      inflation_radius: 0.25            # 25cm safety margin

# Local costmap (for control)
local_costmap:
  ros__parameters:
    update_frequency: 10.0
    publish_frequency: 5.0
    global_frame: odom
    robot_base_frame: base_link
    rolling_window: true
    width: 3                            # 3m x 3m window
    height: 3
    resolution: 0.05
    plugins: ["obstacle_layer", "inflation_layer"]
```

### AMCL (Localization)

```yaml
amcl:
  ros__parameters:
    min_particles: 500
    max_particles: 2000
    update_min_d: 0.2                   # Update every 20cm
    update_min_a: 0.5                   # Or 0.5 rad rotation
    resample_interval: 1
    transform_tolerance: 0.5

    # Differential drive motion model
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    alpha1: 0.2                         # Rotation from rotation
    alpha2: 0.2                         # Rotation from translation
    alpha3: 0.2                         # Translation from translation
    alpha4: 0.2                         # Translation from rotation

    # Laser model
    laser_model_type: likelihood_field
    z_hit: 0.5
    z_rand: 0.5
    sigma_hit: 0.2
```

### SLAM Toolbox

```yaml
slam_toolbox:
  ros__parameters:
    # CRITICAL: Must be false for real robot!
    use_sim_time: false

    mode: mapping
    resolution: 0.05
    minimum_travel_distance: 0.2
    transform_publish_period: 0.02      # 50Hz TF
    map_update_interval: 1.0            # 1Hz map publish

    # Scan matching
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    use_scan_matching: true
    use_scan_barycenter: true

    # Loop closure
    do_loop_closing: true
    loop_search_maximum_distance: 3.0
    loop_match_minimum_chain_size: 10
```

### Velocity Smoother

```yaml
velocity_smoother:
  ros__parameters:
    smoothing_frequency: 20.0
    feedback_type: "OPEN_LOOP"
    max_velocity: [0.5, 0.0, 1.0]       # [vx, vy, wz]
    min_velocity: [-0.2, 0.0, -1.0]
    max_accel: [0.3, 0.0, 0.5]
    max_decel: [-0.3, 0.0, -0.5]
    deadband_velocity: [0.0, 0.0, 0.0]
```

### Recovery Behaviors

```yaml
behavior_server:
  ros__parameters:
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
      simulate_ahead_time: 2.0
    backup:
      plugin: "nav2_behaviors/BackUp"
      simulate_ahead_time: 2.0
    wait:
      plugin: "nav2_behaviors/Wait"
```

---

## Mode Manager Design

### Architecture: Lifecycle-Based (NOT Subprocess)

**Why**: The Maurice analysis identified subprocess spawning as the primary architectural weakness:
- Race conditions with hardcoded delays
- No health checking
- Zombie process risk
- SIGKILL may not clean up ROS2 resources

**Solution**: Use ROS2 lifecycle management

```
┌─────────────────┐     ┌───────────────────────┐
│  mode_manager   │────▶│ Navigation Lifecycle  │
│                 │     │      Manager          │
│                 │     └───────────┬───────────┘
│                 │                 │
│                 │     ┌───────────┼───────────┐
│                 │     ▼           ▼           ▼
│                 │  map_server   amcl    planner...
│                 │
│                 │     ┌───────────────────────┐
│                 │────▶│  Mapping Lifecycle    │
│                 │     │       Manager         │
│                 │     └───────────┬───────────┘
│                 │                 │
│                 │                 ▼
│                 │            slam_toolbox
└─────────────────┘
```

### Services

| Service | Type | Purpose |
|---------|------|---------|
| `/nav/change_mode` | `ChangeMode.srv` | Switch navigation mode |
| `/nav/change_map` | `ChangeMap.srv` | Change active map |
| `/nav/save_map` | `SaveMap.srv` | Save SLAM map |
| `/nav/delete_map` | `DeleteMap.srv` | Delete a map |
| `/nav/get_status` | `GetNavStatus.srv` | Get detailed status |

### Topics Published

| Topic | Type | Rate | Purpose |
|-------|------|------|---------|
| `/nav/mode` | `String` | 1 Hz | Current mode |
| `/nav/maps` | `StringArray` | 1 Hz | Available maps |
| `/nav/current_map` | `String` | 1 Hz | Active map name |
| `/nav/status` | `NavStatus` | 1 Hz | Detailed status |

### Custom Messages

```
# msg/NavStatus.msg
string mode                    # "navigation", "mapping", "mapfree", "none"
string current_map
string[] available_maps
bool is_active                 # All nodes healthy
bool is_localized              # AMCL converged (nav mode only)
float32 localization_score     # AMCL score if available
string[] active_nodes          # List of active lifecycle nodes
string error                   # Last error if any
```

```
# srv/ChangeMode.srv
string mode
---
bool success
string message
float32 transition_time_sec
```

### Startup Sequence

```python
# 1. Launch all nav nodes (inactive)
ros2 launch llmy_nav nav_bringup.launch.py

# 2. Launch mode manager
ros2 launch llmy_nav mode_manager.launch.py

# 3. Mode manager:
#    a. Loads last mode from ~/.llmy/.last_mode
#    b. Loads last map from ~/.llmy/.last_map
#    c. Waits for lifecycle managers to be available
#    d. Activates appropriate mode via lifecycle transitions
```

### Mode Switching (Lifecycle)

```python
async def change_mode(self, target_mode: str) -> bool:
    # 1. Deactivate current mode
    if self.current_mode != "none":
        await self.lifecycle_deactivate(self.current_mode)
        await self.lifecycle_cleanup(self.current_mode)

    # 2. Configure and activate target mode
    await self.lifecycle_configure(target_mode)

    # 3. Update map parameter if navigation mode
    if target_mode == "navigation":
        await self.set_map_parameter(self.current_map)

    await self.lifecycle_activate(target_mode)

    # 4. Verify activation
    is_active = await self.verify_mode_active(target_mode)

    if is_active:
        self.current_mode = target_mode
        self.save_last_mode(target_mode)
        return True
    else:
        self.current_mode = "none"
        return False
```

### Benefits Over Subprocess Approach

| Aspect | Subprocess | Lifecycle |
|--------|-----------|-----------|
| Switch time | 5-7 seconds | < 1 second |
| Health check | None | Built-in |
| Error detection | Process exit only | State feedback |
| Cleanup | SIGKILL (risky) | Graceful |
| Memory | 2-3 processes | 0 overhead |

---

## Map Management

### Directory Structure

```
~/.llmy/
├── maps/
│   ├── home.yaml           # Default map
│   ├── home.pgm
│   ├── office.yaml
│   ├── office.pgm
│   └── ...
├── .last_mode              # "navigation", "mapping", or "mapfree"
└── .last_map               # "home.yaml"
```

### Map Operations

**Save Map** (mapping mode only):
```python
async def save_map(self, name: str, overwrite: bool = False) -> bool:
    if self.current_mode != "mapping":
        return False, "Must be in mapping mode"

    # Validate name
    if not name.replace('_', '').replace('-', '').isalnum():
        return False, "Invalid map name"

    path = self.maps_dir / f"{name}.yaml"
    if path.exists() and not overwrite:
        return False, "Map exists, set overwrite=True"

    # Use map_saver_cli
    result = await self.call_map_saver(path)

    self.refresh_maps()
    return result.success, result.message
```

**Change Map** (navigation mode):
```python
async def change_map(self, name: str) -> bool:
    if name not in self.available_maps:
        return False, "Map not found"

    self.current_map = name
    self.save_last_map(name)

    if self.current_mode == "navigation":
        # Update map_server parameter and reconfigure
        await self.set_map_parameter(name)
        await self.lifecycle_deactivate("navigation")
        await self.lifecycle_activate("navigation")

    return True, f"Map changed to {name}"
```

---

## MCP Integration

The `llmy_mcp` package exposes navigation functionality:

### Navigation Tools

```python
@mcp.tool
def set_navigation_mode(mode: str) -> dict:
    """Switch navigation mode: 'mapping', 'navigation', or 'mapfree'"""

@mcp.tool
def get_navigation_status() -> dict:
    """Get current navigation status including mode, map, and health"""

@mcp.tool
def navigate_to_pose(x: float, y: float, theta: float = 0.0) -> dict:
    """Send navigation goal (navigation mode only)"""

@mcp.tool
def cancel_navigation() -> dict:
    """Cancel current navigation goal"""

@mcp.tool
def save_map(name: str) -> dict:
    """Save current SLAM map (mapping mode only)"""

@mcp.tool
def list_maps() -> list[str]:
    """List available maps"""

@mcp.tool
def set_map(name: str) -> dict:
    """Change the active map"""
```

---

## Testing Requirements

### Unit Tests

- [ ] Mode switching state machine
- [ ] Map name validation
- [ ] Persistence load/save
- [ ] Configuration loading

### Integration Tests

- [ ] Mode switch: none → navigation
- [ ] Mode switch: navigation → mapping
- [ ] Mode switch: mapping → mapfree
- [ ] Mode switch: mapfree → navigation
- [ ] Rapid mode switching (stress test)
- [ ] Map save during mapping
- [ ] Map change during navigation
- [ ] Recovery after node crash

### System Tests

- [ ] Full navigation to goal
- [ ] Obstacle avoidance
- [ ] Recovery behavior execution
- [ ] SLAM map building
- [ ] Localization convergence

---

## Dependencies

```xml
<depend>rclpy</depend>
<depend>nav2_map_server</depend>
<depend>nav2_amcl</depend>
<depend>nav2_planner</depend>
<depend>nav2_controller</depend>
<depend>nav2_bt_navigator</depend>
<depend>nav2_behaviors</depend>
<depend>nav2_velocity_smoother</depend>
<depend>nav2_lifecycle_manager</depend>
<depend>slam_toolbox</depend>
<depend>tf2_ros</depend>
<depend>lifecycle_msgs</depend>
```

---

## Implementation Phases

### Phase 1: Core Package Structure
- [ ] Create package skeleton
- [ ] Write configuration files
- [ ] Create launch files
- [ ] Basic mode_manager without lifecycle (for testing)

### Phase 2: Lifecycle Integration
- [ ] Implement lifecycle-aware mode_manager
- [ ] Add health checking
- [ ] Add error recovery
- [ ] Integration with nav2_lifecycle_manager

### Phase 3: Map Management
- [ ] Map discovery
- [ ] Map save/load
- [ ] Persistence
- [ ] Map change during navigation

### Phase 4: MCP Integration
- [ ] Add navigation tools to llmy_mcp
- [ ] Add navigation resources
- [ ] Test LLM control of navigation

### Phase 5: Testing & Polish
- [ ] Write integration tests
- [ ] Documentation
- [ ] Performance tuning
- [ ] Error handling edge cases

---

## Key Differences from Maurice Nav

| Aspect | Maurice | LLMy |
|--------|---------|------|
| Mode switching | Subprocess spawning | Lifecycle management |
| Health checking | None | Built-in lifecycle states |
| Mode switch time | 5-7 seconds | < 1 second |
| use_sim_time | Buggy (defaults true) | Correct (defaults false) |
| Error recovery | None (sets mode to "none") | Retry + fallback |
| Config location | Environment variable | ROS parameter |
| Testing | None | Required |
| MCP integration | None | Built-in |

---

## References

- [Nav2 Documentation](https://navigation.ros.org/)
- [ROS2 Lifecycle](https://design.ros2.org/articles/node_lifecycle.html)
- [MPPI Controller](https://navigation.ros.org/configuration/packages/configuring-mppic.html)
- [SmacPlanner2D](https://navigation.ros.org/configuration/packages/configuring-smac-planner.html)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
