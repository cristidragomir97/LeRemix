# LLMy MCP - ROS2 Bridge for LLMs

A dynamic [Model Context Protocol (MCP)](https://modelcontextprotocol.io/) server that bridges ROS2 to Large Language Models. This allows LLMs like Claude to interact with robots, read sensor data, and control actuators through natural language.

## Features

- **Dynamic Discovery**: Automatically detects all ROS2 topics and services
- **Type Resolution**: Resolves message types at runtime - no hardcoding needed
- **Bridgeability Detection**: Identifies which types can/cannot be bridged
- **Generic Interface**: Subscribe, publish, and call services on any ROS2 interface
- **Message Conversion**: Automatic JSON ↔ ROS message conversion
- **Configurable**: Filter topics/services, set QoS, auto-subscribe on startup

## Installation

### Prerequisites

- ROS2 Jazzy (or compatible)
- Python 3.10+
- FastMCP 2.x

### Using Pixi (recommended)

FastMCP is already in `pixi.toml`:

```bash
pixi install
```

### Manual Installation

```bash
pip install "fastmcp>=2.0,<3"
```

### Building the Package

```bash
# From workspace root
pixi run colcon build --packages-select llmy_mcp
source install/setup.bash
```

## Usage

### Running the Server

```bash
# Basic usage
ros2 run llmy_mcp mcp_server

# With config file
ros2 run llmy_mcp mcp_server --config /path/to/config.yaml

# Direct Python
python -m llmy_mcp.server
```

### Connecting an MCP Client

For Claude Desktop, add to your MCP configuration:

```json
{
  "mcpServers": {
    "ros2": {
      "command": "ros2",
      "args": ["run", "llmy_mcp", "mcp_server"],
      "env": {
        "ROS_DOMAIN_ID": "0"
      }
    }
  }
}
```

## Available Tools

### Discovery

| Tool | Description |
|------|-------------|
| `list_topics()` | List all ROS2 topics with types and bridgeability |
| `list_services()` | List all ROS2 services |
| `get_topic_info(topic)` | Get details about a specific topic |
| `get_bridge_status()` | Summary of bridgeable vs unbridgeable interfaces |

### Subscriptions

| Tool | Description |
|------|-------------|
| `subscribe_topic(topic)` | Subscribe to start receiving messages |
| `unsubscribe_topic(topic)` | Stop receiving messages |
| `get_subscriptions()` | List active subscriptions |
| `read_topic(topic, count)` | Read cached messages (auto-subscribes if needed) |

### Publishing

| Tool | Description |
|------|-------------|
| `publish_topic(topic, message, msg_type)` | Publish any message type |
| `publish_twist(topic, linear_x, angular_z, ...)` | Convenience for Twist messages |
| `publish_twist_stamped(topic, ...)` | Convenience for TwistStamped |
| `publish_float64_array(topic, data)` | Convenience for Float64MultiArray |

### Services

| Tool | Description |
|------|-------------|
| `call_service(service, request, timeout)` | Call any ROS2 service |

## Resources

MCP resources provide read-only data:

| URI | Description |
|-----|-------------|
| `ros://topics` | List of all topics |
| `ros://services` | List of all services |
| `ros://status` | Bridge status summary |
| `ros://subscriptions` | Active subscriptions |

## Configuration

Create a YAML config file to customize behavior:

```yaml
# Exclude topics (regex patterns)
excluded_topics:
  - "^/rosout.*"
  - "^/parameter_events$"

# Exclude services (regex patterns)
excluded_services:
  - ".*/get_parameters$"
  - ".*/set_parameters$"

# Auto-subscribe on startup
auto_subscribe:
  - /joint_states
  - /scan

# Message cache size per topic
message_cache_size: 10

# QoS settings
default_reliability: best_effort  # or "reliable"
default_history_depth: 10
```

## Extending the Server

### Adding Custom Tools

Edit `server.py` to add domain-specific tools:

```python
@mcp.tool
def my_custom_tool(
    param: Annotated[float, Field(description="Description")]
) -> dict:
    """Tool description shown to the LLM."""
    bridge = get_bridge()
    # Your logic here
    return {"result": "success"}
```

### Adding Convenience Publishers

For common message types, add convenience wrappers:

```python
@mcp.tool
def publish_pose(
    topic: Annotated[str, Field(description="Topic name")] = "/goal_pose",
    x: Annotated[float, Field(description="X position")] = 0.0,
    y: Annotated[float, Field(description="Y position")] = 0.0,
    theta: Annotated[float, Field(description="Orientation")] = 0.0
) -> dict:
    """Publish a PoseStamped message."""
    import math
    bridge = get_bridge()

    now = bridge.node.get_clock().now()
    message = {
        "header": {
            "stamp": {"sec": now.nanoseconds // 1_000_000_000,
                     "nanosec": now.nanoseconds % 1_000_000_000},
            "frame_id": "map"
        },
        "pose": {
            "position": {"x": x, "y": y, "z": 0.0},
            "orientation": {
                "x": 0.0, "y": 0.0,
                "z": math.sin(theta/2), "w": math.cos(theta/2)
            }
        }
    }
    return publish_topic(topic, message, "geometry_msgs/msg/PoseStamped")
```

### Supporting Custom Message Types

The type resolver automatically handles any message type that:
1. Is installed in the ROS2 environment
2. Has Python bindings generated

For custom packages, ensure they're built and sourced:

```bash
colcon build --packages-select my_custom_msgs
source install/setup.bash
```

The bridge will then automatically detect and bridge topics using those types.

### Adding Resources

Add custom resources for commonly-needed data:

```python
@mcp.resource("ros://robot/state")
def resource_robot_state() -> dict:
    """Aggregated robot state."""
    bridge = get_bridge()

    # Ensure subscribed
    bridge.subscribe_topic("/joint_states")
    bridge.subscribe_topic("/imu/data")

    return {
        "joints": bridge.get_latest_message("/joint_states"),
        "imu": bridge.get_latest_message("/imu/data"),
        "timestamp": time.time()
    }
```

### Custom Type Converters

For special message handling, extend `type_resolver.py`:

```python
def msg_to_dict(msg: Any) -> Any:
    # Add special handling for specific types
    if isinstance(msg, sensor_msgs.msg.Image):
        # Custom image handling
        return {
            "width": msg.width,
            "height": msg.height,
            "encoding": msg.encoding,
            # Maybe base64 encode the data
        }
    # ... default handling
```

## Architecture

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   MCP Client    │────▶│   FastMCP Server │────▶│   ROS2 Bridge   │
│  (Claude, etc)  │◀────│   (server.py)    │◀────│ (ros_bridge.py) │
└─────────────────┘     └──────────────────┘     └────────┬────────┘
                                                          │
                               ┌──────────────────────────┼──────────────────────────┐
                               │                          │                          │
                               ▼                          ▼                          ▼
                        ┌─────────────┐           ┌─────────────┐           ┌─────────────┐
                        │   Topics    │           │  Services   │           │   Actions   │
                        │ /cmd_vel    │           │ /spawn      │           │  (planned)  │
                        │ /scan       │           │ /kill       │           │             │
                        └─────────────┘           └─────────────┘           └─────────────┘
```

### Key Components

- **`server.py`**: FastMCP server with tool/resource definitions
- **`ros_bridge.py`**: ROS2 node handling subscriptions, publishers, service clients
- **`type_resolver.py`**: Dynamic message type resolution and JSON conversion

## Bridgeability

Not all ROS2 types can be bridged. A type is bridgeable if:
- The message package is installed
- Python bindings exist
- The module can be imported

Common unbridgeable scenarios:
- Custom messages not built/sourced
- C++-only packages without Python bindings
- Third-party packages not in the environment

Use `get_bridge_status()` to see what's bridgeable.

## Example Session

```
LLM: What topics are available?
→ list_topics()
← [{"name": "/joint_states", "type": "sensor_msgs/msg/JointState", ...}, ...]

LLM: Read the current joint positions
→ read_topic("/joint_states")
← {"messages": [{"name": ["joint1", "joint2", ...], "position": [0.1, 0.2, ...]}]}

LLM: Move the robot forward slowly
→ publish_twist_stamped("/diff_drive_controller/cmd_vel", linear_x=0.1)
← {"success": true, "message": "Published to /diff_drive_controller/cmd_vel"}
```

## License

MIT
