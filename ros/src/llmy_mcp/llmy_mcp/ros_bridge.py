"""
ROS2 Bridge - Dynamic ROS2 communication layer for the MCP server.

Handles runtime discovery and bridging of topics, services, and actions.
"""

import threading
import time
import yaml
from pathlib import Path
from typing import Any, Callable, Optional
from dataclasses import dataclass, field
from collections import deque
import logging

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data, qos_profile_system_default

from .type_resolver import TypeResolver, get_resolver, msg_to_dict, dict_to_msg

logger = logging.getLogger(__name__)


@dataclass
class TopicInfo:
    """Information about a ROS2 topic."""
    name: str
    msg_type: str
    publishers: int
    subscribers: int
    is_bridgeable: bool = True
    error: Optional[str] = None


@dataclass
class ServiceInfo:
    """Information about a ROS2 service."""
    name: str
    srv_type: str
    is_bridgeable: bool = True
    error: Optional[str] = None


@dataclass
class MessageCache:
    """Cache for storing recent messages from a topic."""
    messages: deque = field(default_factory=lambda: deque(maxlen=10))
    last_update: float = 0.0
    msg_type: str = ""


@dataclass
class BridgeConfig:
    """Configuration for the ROS2 bridge."""
    # Topics to exclude from listing/bridging (regex patterns)
    excluded_topics: list[str] = field(default_factory=list)
    # Topics to always include even if type resolution fails
    included_topics: list[str] = field(default_factory=list)
    # Services to exclude
    excluded_services: list[str] = field(default_factory=list)
    # Message cache size
    message_cache_size: int = 10
    # Auto-subscribe topics on startup
    auto_subscribe: list[str] = field(default_factory=list)
    # Default QoS settings
    default_reliability: str = "best_effort"
    default_history_depth: int = 10

    @classmethod
    def from_yaml(cls, path: Path) -> "BridgeConfig":
        """Load config from YAML file."""
        if not path.exists():
            return cls()
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        return cls(
            excluded_topics=data.get("excluded_topics", []),
            included_topics=data.get("included_topics", []),
            excluded_services=data.get("excluded_services", []),
            message_cache_size=data.get("message_cache_size", 10),
            auto_subscribe=data.get("auto_subscribe", []),
            default_reliability=data.get("default_reliability", "best_effort"),
            default_history_depth=data.get("default_history_depth", 10),
        )


class ROS2Bridge:
    """Dynamic bridge between MCP server and ROS2."""

    def __init__(self, config: Optional[BridgeConfig] = None):
        self._node: Optional[Node] = None
        self._executor: Optional[MultiThreadedExecutor] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._running = False
        self._config = config or BridgeConfig()
        self._resolver = get_resolver()

        # Dynamic subscriptions and publishers
        self._message_caches: dict[str, MessageCache] = {}
        self._subscriptions: dict[str, Any] = {}
        self._publishers: dict[str, Any] = {}
        self._service_clients: dict[str, Any] = {}

        # Discovery cache
        self._discovered_topics: dict[str, TopicInfo] = {}
        self._discovered_services: dict[str, ServiceInfo] = {}
        self._last_discovery: float = 0.0
        self._discovery_interval: float = 5.0  # Refresh every 5 seconds

    def init(self):
        """Initialize ROS2 and create the bridge node."""
        if self._node is not None:
            return

        try:
            rclpy.init()
        except RuntimeError:
            pass  # Already initialized

        self._node = rclpy.create_node('llmy_mcp_bridge')
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)

        self._running = True
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        # Initial discovery
        self._discover_all()

        # Auto-subscribe configured topics
        for topic in self._config.auto_subscribe:
            self._auto_subscribe(topic)

        logger.info('LLMy MCP Bridge initialized')

    def _spin(self):
        """Spin the executor in a background thread."""
        while self._running:
            self._executor.spin_once(timeout_sec=0.1)

    def shutdown(self):
        """Shutdown ROS2 cleanly."""
        self._running = False
        if self._spin_thread:
            self._spin_thread.join(timeout=2.0)
        if self._executor:
            self._executor.shutdown()
        if self._node:
            self._node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    @property
    def node(self) -> Optional[Node]:
        return self._node

    def _should_exclude_topic(self, name: str) -> bool:
        """Check if a topic should be excluded."""
        import re
        for pattern in self._config.excluded_topics:
            if re.match(pattern, name):
                return True
        return False

    def _should_exclude_service(self, name: str) -> bool:
        """Check if a service should be excluded."""
        import re
        for pattern in self._config.excluded_services:
            if re.match(pattern, name):
                return True
        return False

    def _discover_all(self):
        """Discover all topics and services, check bridgeability."""
        if not self._node:
            return

        now = time.time()
        if now - self._last_discovery < self._discovery_interval:
            return

        self._last_discovery = now

        # Discover topics
        self._discovered_topics.clear()
        for name, types in self._node.get_topic_names_and_types():
            if self._should_exclude_topic(name):
                continue

            msg_type = types[0] if types else 'unknown'
            pub_info = self._node.get_publishers_info_by_topic(name)
            sub_info = self._node.get_subscriptions_info_by_topic(name)

            is_bridgeable = self._resolver.is_bridgeable(msg_type)
            error = None
            if not is_bridgeable:
                type_info = self._resolver.resolve(msg_type)
                error = type_info.error if type_info else f"Cannot resolve {msg_type}"

            self._discovered_topics[name] = TopicInfo(
                name=name,
                msg_type=msg_type,
                publishers=len(pub_info),
                subscribers=len(sub_info),
                is_bridgeable=is_bridgeable,
                error=error
            )

        # Discover services
        self._discovered_services.clear()
        for name, types in self._node.get_service_names_and_types():
            if self._should_exclude_service(name):
                continue

            srv_type = types[0] if types else 'unknown'
            is_bridgeable = self._resolver.is_bridgeable(srv_type)
            error = None
            if not is_bridgeable:
                type_info = self._resolver.resolve(srv_type)
                error = type_info.error if type_info else f"Cannot resolve {srv_type}"

            self._discovered_services[name] = ServiceInfo(
                name=name,
                srv_type=srv_type,
                is_bridgeable=is_bridgeable,
                error=error
            )

    def _auto_subscribe(self, topic: str):
        """Auto-subscribe to a topic by looking up its type."""
        if not self._node:
            return

        topics = self._node.get_topic_names_and_types()
        for name, types in topics:
            if name == topic and types:
                self.subscribe_topic(topic, types[0])
                break

    def list_topics(self, refresh: bool = False) -> list[TopicInfo]:
        """List all discovered topics."""
        if refresh or not self._discovered_topics:
            self._last_discovery = 0  # Force refresh
            self._discover_all()
        return list(self._discovered_topics.values())

    def list_services(self, refresh: bool = False) -> list[ServiceInfo]:
        """List all discovered services."""
        if refresh or not self._discovered_services:
            self._last_discovery = 0
            self._discover_all()
        return list(self._discovered_services.values())

    def get_topic_info(self, topic: str) -> Optional[TopicInfo]:
        """Get info about a specific topic."""
        self._discover_all()
        return self._discovered_topics.get(topic)

    def _get_qos_profile(self, topic: str) -> QoSProfile:
        """Get appropriate QoS for a topic."""
        # Use sensor data profile for common sensor topics
        sensor_patterns = ['/scan', '/imu', '/camera', '/image', '/odom', '/joint_states']
        for pattern in sensor_patterns:
            if pattern in topic:
                return qos_profile_sensor_data

        # Default profile
        reliability = (ReliabilityPolicy.BEST_EFFORT
                      if self._config.default_reliability == "best_effort"
                      else ReliabilityPolicy.RELIABLE)
        return QoSProfile(
            reliability=reliability,
            history=HistoryPolicy.KEEP_LAST,
            depth=self._config.default_history_depth
        )

    def subscribe_topic(
        self,
        topic: str,
        msg_type_str: Optional[str] = None,
        callback: Optional[Callable] = None
    ) -> tuple[bool, str]:
        """
        Subscribe to a topic dynamically.

        Returns (success, message).
        """
        if not self._node:
            return False, "Node not initialized"

        if topic in self._subscriptions:
            return True, "Already subscribed"

        # Auto-detect type if not provided
        if not msg_type_str:
            topic_info = self.get_topic_info(topic)
            if not topic_info:
                return False, f"Topic {topic} not found"
            msg_type_str = topic_info.msg_type

        # Resolve the type
        msg_class = self._resolver.get_class(msg_type_str)
        if not msg_class:
            return False, f"Cannot resolve message type: {msg_type_str}"

        # Create cache
        self._message_caches[topic] = MessageCache(
            messages=deque(maxlen=self._config.message_cache_size),
            msg_type=msg_type_str
        )

        def cache_callback(msg):
            cache = self._message_caches.get(topic)
            if cache:
                cache.messages.append(msg)
                cache.last_update = time.time()
            if callback:
                callback(msg)

        qos = self._get_qos_profile(topic)
        self._subscriptions[topic] = self._node.create_subscription(
            msg_class, topic, cache_callback, qos
        )

        logger.info(f"Subscribed to {topic} ({msg_type_str})")
        return True, f"Subscribed to {topic}"

    def unsubscribe_topic(self, topic: str) -> tuple[bool, str]:
        """Unsubscribe from a topic."""
        if topic not in self._subscriptions:
            return False, f"Not subscribed to {topic}"

        self._node.destroy_subscription(self._subscriptions[topic])
        del self._subscriptions[topic]
        if topic in self._message_caches:
            del self._message_caches[topic]

        return True, f"Unsubscribed from {topic}"

    def get_subscribed_topics(self) -> list[str]:
        """Get list of currently subscribed topics."""
        return list(self._subscriptions.keys())

    def get_cached_messages(self, topic: str, count: int = 1) -> list[dict]:
        """Get cached messages from a subscribed topic as dictionaries."""
        cache = self._message_caches.get(topic)
        if not cache:
            return []
        messages = list(cache.messages)[-count:]
        return [msg_to_dict(msg) for msg in messages]

    def get_latest_message(self, topic: str) -> Optional[dict]:
        """Get the latest message from a subscribed topic as dictionary."""
        messages = self.get_cached_messages(topic, 1)
        return messages[0] if messages else None

    def get_latest_message_raw(self, topic: str) -> Optional[Any]:
        """Get the latest raw ROS message object."""
        cache = self._message_caches.get(topic)
        if not cache or not cache.messages:
            return None
        return cache.messages[-1]

    def publish(
        self,
        topic: str,
        data: dict,
        msg_type_str: Optional[str] = None
    ) -> tuple[bool, str]:
        """
        Publish a message to a topic.

        Args:
            topic: Topic name
            data: Message data as dictionary
            msg_type_str: Optional message type (auto-detected if not provided)

        Returns (success, message).
        """
        if not self._node:
            return False, "Node not initialized"

        # Auto-detect type if not provided
        if not msg_type_str:
            topic_info = self.get_topic_info(topic)
            if topic_info:
                msg_type_str = topic_info.msg_type
            elif topic in self._message_caches:
                msg_type_str = self._message_caches[topic].msg_type
            else:
                return False, f"Cannot determine message type for {topic}"

        msg_class = self._resolver.get_class(msg_type_str)
        if not msg_class:
            return False, f"Cannot resolve message type: {msg_type_str}"

        # Create publisher if needed
        if topic not in self._publishers:
            qos = self._get_qos_profile(topic)
            self._publishers[topic] = self._node.create_publisher(msg_class, topic, qos)

        # Convert dict to message
        try:
            msg = dict_to_msg(data, msg_class)
        except Exception as e:
            return False, f"Failed to create message: {e}"

        self._publishers[topic].publish(msg)
        return True, f"Published to {topic}"

    def call_service(
        self,
        service: str,
        request: dict,
        timeout: float = 5.0
    ) -> tuple[bool, Any]:
        """
        Call a service synchronously.

        Args:
            service: Service name
            request: Request data as dictionary
            timeout: Timeout in seconds

        Returns (success, response_dict or error_message).
        """
        if not self._node:
            return False, "Node not initialized"

        # Get service info
        service_info = self._discovered_services.get(service)
        if not service_info:
            self._discover_all()
            service_info = self._discovered_services.get(service)
        if not service_info:
            return False, f"Service {service} not found"

        if not service_info.is_bridgeable:
            return False, f"Service type not bridgeable: {service_info.srv_type}"

        srv_class = self._resolver.get_class(service_info.srv_type)
        if not srv_class:
            return False, f"Cannot resolve service type: {service_info.srv_type}"

        # Create client if needed
        if service not in self._service_clients:
            self._service_clients[service] = self._node.create_client(srv_class, service)

        client = self._service_clients[service]

        # Wait for service
        if not client.wait_for_service(timeout_sec=min(timeout, 2.0)):
            return False, f"Service {service} not available"

        # Create request
        try:
            req = dict_to_msg(request, srv_class.Request)
        except Exception as e:
            return False, f"Failed to create request: {e}"

        # Call service
        future = client.call_async(req)

        # Wait for response
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.05)

        if not future.done():
            return False, f"Service call timed out after {timeout}s"

        try:
            response = future.result()
            return True, msg_to_dict(response)
        except Exception as e:
            return False, f"Service call failed: {e}"

    def get_discovery_summary(self) -> dict:
        """Get a summary of discovered and bridgeable interfaces."""
        self._discover_all()

        bridgeable_topics = [t for t in self._discovered_topics.values() if t.is_bridgeable]
        unbridgeable_topics = [t for t in self._discovered_topics.values() if not t.is_bridgeable]
        bridgeable_services = [s for s in self._discovered_services.values() if s.is_bridgeable]
        unbridgeable_services = [s for s in self._discovered_services.values() if not s.is_bridgeable]

        return {
            "topics": {
                "total": len(self._discovered_topics),
                "bridgeable": len(bridgeable_topics),
                "unbridgeable": len(unbridgeable_topics),
                "unbridgeable_list": [
                    {"name": t.name, "type": t.msg_type, "error": t.error}
                    for t in unbridgeable_topics
                ]
            },
            "services": {
                "total": len(self._discovered_services),
                "bridgeable": len(bridgeable_services),
                "unbridgeable": len(unbridgeable_services),
                "unbridgeable_list": [
                    {"name": s.name, "type": s.srv_type, "error": s.error}
                    for s in unbridgeable_services
                ]
            },
            "subscriptions": {
                "active": list(self._subscriptions.keys())
            }
        }


# Global bridge instance
_bridge: Optional[ROS2Bridge] = None


def get_bridge(config: Optional[BridgeConfig] = None) -> ROS2Bridge:
    """Get or create the global ROS2 bridge instance."""
    global _bridge
    if _bridge is None:
        _bridge = ROS2Bridge(config)
        _bridge.init()
    return _bridge


def shutdown_bridge():
    """Shutdown the global bridge."""
    global _bridge
    if _bridge:
        _bridge.shutdown()
        _bridge = None
