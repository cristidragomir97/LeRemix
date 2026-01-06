"""
LLMy MCP Server - Generic ROS2 bridge via Model Context Protocol.

This server dynamically exposes all ROS2 topics and services that can be
bridged to Python. It provides:
- Discovery of all topics and services
- Dynamic subscription to any topic
- Publishing to any topic
- Calling any service
- Automatic type resolution and conversion
"""

import time
import json
import logging
from pathlib import Path
from typing import Annotated, Optional, Any

from fastmcp import FastMCP, Context
from pydantic import Field

from .ros_bridge import get_bridge, shutdown_bridge, BridgeConfig

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


# Create the MCP server
mcp = FastMCP(
    name="ROS2 Bridge",
    instructions="""
You are connected to a ROS2 system via a dynamic bridge. This bridge can:

1. **Discover** all available topics and services
2. **Subscribe** to any topic to receive messages
3. **Publish** to any topic by providing message data as JSON
4. **Call** any service with a request

## Getting Started

1. Use `list_topics()` to see all available topics
2. Use `list_services()` to see all available services
3. Use `get_bridge_status()` to see what's bridgeable vs not

## Reading Data

1. First subscribe: `subscribe_topic("/joint_states")`
2. Then read: `read_topic("/joint_states")`

## Publishing Data

Use `publish_topic()` with the topic name and message as a dictionary.
The message structure must match the topic's message type.

Example for geometry_msgs/msg/Twist:
```json
{
    "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.1}
}
```

## Calling Services

Use `call_service()` with the service name and request as a dictionary.

## Tips

- Check `is_bridgeable` field - some types may not be available
- Messages are automatically converted to/from JSON
- Use `get_topic_info()` to see a topic's message type
- Subscribe before reading - unsubscribed topics have no cached data
"""
)


# ============================================================================
# DISCOVERY TOOLS
# ============================================================================

@mcp.tool
def list_topics(
    refresh: Annotated[bool, Field(description="Force refresh the topic list")] = False,
    bridgeable_only: Annotated[bool, Field(description="Only show topics that can be bridged")] = False
) -> list[dict]:
    """
    List all available ROS2 topics.

    Returns topic names, message types, publisher/subscriber counts,
    and whether each topic can be bridged (type is resolvable).
    """
    bridge = get_bridge()
    topics = bridge.list_topics(refresh=refresh)

    if bridgeable_only:
        topics = [t for t in topics if t.is_bridgeable]

    return [
        {
            "name": t.name,
            "type": t.msg_type,
            "publishers": t.publishers,
            "subscribers": t.subscribers,
            "is_bridgeable": t.is_bridgeable,
            "error": t.error
        }
        for t in topics
    ]


@mcp.tool
def list_services(
    refresh: Annotated[bool, Field(description="Force refresh the service list")] = False,
    bridgeable_only: Annotated[bool, Field(description="Only show services that can be bridged")] = False
) -> list[dict]:
    """
    List all available ROS2 services.

    Returns service names, types, and whether each can be bridged.
    """
    bridge = get_bridge()
    services = bridge.list_services(refresh=refresh)

    if bridgeable_only:
        services = [s for s in services if s.is_bridgeable]

    return [
        {
            "name": s.name,
            "type": s.srv_type,
            "is_bridgeable": s.is_bridgeable,
            "error": s.error
        }
        for s in services
    ]


@mcp.tool
def get_topic_info(
    topic: Annotated[str, Field(description="Topic name (e.g., /joint_states)")]
) -> dict:
    """
    Get detailed information about a specific topic.

    Returns the topic's message type, connection info, and bridgeability.
    """
    bridge = get_bridge()
    info = bridge.get_topic_info(topic)

    if not info:
        return {"error": f"Topic {topic} not found"}

    return {
        "name": info.name,
        "type": info.msg_type,
        "publishers": info.publishers,
        "subscribers": info.subscribers,
        "is_bridgeable": info.is_bridgeable,
        "error": info.error
    }


@mcp.tool
def get_bridge_status() -> dict:
    """
    Get the overall status of the ROS2 bridge.

    Shows counts of bridgeable vs unbridgeable topics/services,
    active subscriptions, and any errors.
    """
    bridge = get_bridge()
    return bridge.get_discovery_summary()


# ============================================================================
# SUBSCRIPTION TOOLS
# ============================================================================

@mcp.tool
def subscribe_topic(
    topic: Annotated[str, Field(description="Topic name to subscribe to")]
) -> dict:
    """
    Subscribe to a ROS2 topic to start receiving messages.

    The topic's message type is auto-detected. Messages are cached
    and can be read with read_topic().
    """
    bridge = get_bridge()
    success, message = bridge.subscribe_topic(topic)

    return {
        "success": success,
        "message": message,
        "topic": topic
    }


@mcp.tool
def unsubscribe_topic(
    topic: Annotated[str, Field(description="Topic name to unsubscribe from")]
) -> dict:
    """
    Unsubscribe from a ROS2 topic.

    Stops receiving messages and clears the cache for this topic.
    """
    bridge = get_bridge()
    success, message = bridge.unsubscribe_topic(topic)

    return {
        "success": success,
        "message": message
    }


@mcp.tool
def get_subscriptions() -> list[str]:
    """
    Get list of currently subscribed topics.
    """
    bridge = get_bridge()
    return bridge.get_subscribed_topics()


@mcp.tool
def read_topic(
    topic: Annotated[str, Field(description="Topic name to read from")],
    count: Annotated[int, Field(description="Number of recent messages to return")] = 1,
    wait: Annotated[bool, Field(description="Wait briefly for a message if none cached")] = True
) -> dict:
    """
    Read messages from a subscribed topic.

    Returns the most recent cached messages as JSON-serializable dictionaries.
    If not subscribed, will auto-subscribe first.
    """
    bridge = get_bridge()

    # Auto-subscribe if needed
    if topic not in bridge.get_subscribed_topics():
        success, msg = bridge.subscribe_topic(topic)
        if not success:
            return {"error": msg}
        if wait:
            time.sleep(0.2)  # Wait for first message

    messages = bridge.get_cached_messages(topic, count)

    if not messages and wait:
        time.sleep(0.1)
        messages = bridge.get_cached_messages(topic, count)

    cache = bridge._message_caches.get(topic)
    return {
        "topic": topic,
        "message_type": cache.msg_type if cache else None,
        "count": len(messages),
        "messages": messages,
        "last_update": cache.last_update if cache else None
    }


# ============================================================================
# PUBLISHING TOOLS
# ============================================================================

@mcp.tool
def publish_topic(
    topic: Annotated[str, Field(description="Topic name to publish to")],
    message: Annotated[dict, Field(description="Message data as a dictionary matching the topic's message type")],
    msg_type: Annotated[Optional[str], Field(description="Optional message type (auto-detected if not provided)")] = None
) -> dict:
    """
    Publish a message to a ROS2 topic.

    The message must be a dictionary with fields matching the topic's message type.
    The type is auto-detected from existing publishers on the topic.

    Example for geometry_msgs/msg/Twist:
    {
        "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.1}
    }
    """
    bridge = get_bridge()
    success, result = bridge.publish(topic, message, msg_type)

    return {
        "success": success,
        "message": result,
        "topic": topic
    }


@mcp.tool
def publish_twist(
    topic: Annotated[str, Field(description="Topic name for velocity command")] = "/cmd_vel",
    linear_x: Annotated[float, Field(description="Forward/backward velocity (m/s)")] = 0.0,
    linear_y: Annotated[float, Field(description="Left/right velocity (m/s)")] = 0.0,
    linear_z: Annotated[float, Field(description="Up/down velocity (m/s)")] = 0.0,
    angular_x: Annotated[float, Field(description="Roll velocity (rad/s)")] = 0.0,
    angular_y: Annotated[float, Field(description="Pitch velocity (rad/s)")] = 0.0,
    angular_z: Annotated[float, Field(description="Yaw velocity (rad/s)")] = 0.0
) -> dict:
    """
    Convenience tool to publish a Twist message (velocity command).

    Common use: controlling robot base motion.
    """
    message = {
        "linear": {"x": linear_x, "y": linear_y, "z": linear_z},
        "angular": {"x": angular_x, "y": angular_y, "z": angular_z}
    }
    return publish_topic(topic, message, "geometry_msgs/msg/Twist")


@mcp.tool
def publish_twist_stamped(
    topic: Annotated[str, Field(description="Topic name for velocity command")] = "/diff_drive_controller/cmd_vel",
    linear_x: Annotated[float, Field(description="Forward/backward velocity (m/s)")] = 0.0,
    linear_y: Annotated[float, Field(description="Left/right velocity (m/s)")] = 0.0,
    angular_z: Annotated[float, Field(description="Yaw velocity (rad/s)")] = 0.0,
    frame_id: Annotated[str, Field(description="Frame ID for the header")] = "base_link"
) -> dict:
    """
    Convenience tool to publish a TwistStamped message.

    Used by ros2_control diff_drive_controller.
    """
    bridge = get_bridge()

    # Get current time from ROS
    now = bridge.node.get_clock().now()
    sec = now.nanoseconds // 1_000_000_000
    nanosec = now.nanoseconds % 1_000_000_000

    message = {
        "header": {
            "stamp": {"sec": sec, "nanosec": nanosec},
            "frame_id": frame_id
        },
        "twist": {
            "linear": {"x": linear_x, "y": linear_y, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": angular_z}
        }
    }
    return publish_topic(topic, message, "geometry_msgs/msg/TwistStamped")


@mcp.tool
def publish_float64_array(
    topic: Annotated[str, Field(description="Topic name")],
    data: Annotated[list[float], Field(description="Array of float values")]
) -> dict:
    """
    Convenience tool to publish a Float64MultiArray message.

    Common use: sending joint commands to position controllers.
    """
    message = {"data": data}
    return publish_topic(topic, message, "std_msgs/msg/Float64MultiArray")


# ============================================================================
# SERVICE TOOLS
# ============================================================================

@mcp.tool
def call_service(
    service: Annotated[str, Field(description="Service name to call")],
    request: Annotated[dict, Field(description="Request data as a dictionary")] = {},
    timeout: Annotated[float, Field(description="Timeout in seconds")] = 5.0
) -> dict:
    """
    Call a ROS2 service.

    The request must be a dictionary matching the service's request type.
    For services with no request fields (like std_srvs/srv/Empty), pass {}.
    """
    bridge = get_bridge()
    success, result = bridge.call_service(service, request, timeout)

    if success:
        return {
            "success": True,
            "response": result,
            "service": service
        }
    else:
        return {
            "success": False,
            "error": result,
            "service": service
        }


# ============================================================================
# RESOURCES
# ============================================================================

@mcp.resource("ros://topics")
def resource_topics() -> list[dict]:
    """List of all ROS2 topics."""
    return list_topics(refresh=True)


@mcp.resource("ros://services")
def resource_services() -> list[dict]:
    """List of all ROS2 services."""
    return list_services(refresh=True)


@mcp.resource("ros://status")
def resource_status() -> dict:
    """Bridge status and discovery summary."""
    return get_bridge_status()


@mcp.resource("ros://subscriptions")
def resource_subscriptions() -> dict:
    """Currently active subscriptions."""
    bridge = get_bridge()
    subs = bridge.get_subscribed_topics()
    return {
        "subscriptions": subs,
        "count": len(subs)
    }


# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

def main():
    """Run the MCP server."""
    import argparse
    import sys

    parser = argparse.ArgumentParser(description="LLMy ROS2 MCP Server")
    parser.add_argument(
        "--config", "-c",
        type=Path,
        help="Path to configuration YAML file"
    )
    args = parser.parse_args()

    # Load config
    config = None
    if args.config:
        config = BridgeConfig.from_yaml(args.config)
        logger.info(f"Loaded config from {args.config}")

    # Initialize bridge with config
    bridge = get_bridge(config)

    # Log discovery summary
    summary = bridge.get_discovery_summary()
    logger.info(
        f"Discovered {summary['topics']['total']} topics "
        f"({summary['topics']['bridgeable']} bridgeable), "
        f"{summary['services']['total']} services "
        f"({summary['services']['bridgeable']} bridgeable)"
    )

    if summary['topics']['unbridgeable'] > 0:
        logger.warning(
            f"{summary['topics']['unbridgeable']} topics not bridgeable - "
            "message types not available"
        )

    try:
        # Run FastMCP server
        mcp.run()
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        shutdown_bridge()


if __name__ == "__main__":
    main()
