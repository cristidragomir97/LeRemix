"""
Dynamic ROS2 message and service type resolver.

Handles runtime introspection of ROS2 types, converting between
type strings and actual Python classes.
"""

import importlib
from typing import Any, Optional, Type
from dataclasses import dataclass
import logging

logger = logging.getLogger(__name__)


@dataclass
class TypeInfo:
    """Information about a resolved ROS2 type."""
    type_string: str  # e.g., "std_msgs/msg/String"
    package: str      # e.g., "std_msgs"
    category: str     # "msg" or "srv"
    name: str         # e.g., "String"
    python_class: Type
    is_bridgeable: bool = True
    error: Optional[str] = None


class TypeResolver:
    """Resolves ROS2 type strings to Python classes dynamically."""

    def __init__(self):
        self._cache: dict[str, TypeInfo] = {}
        self._failed_packages: set[str] = set()

    def resolve(self, type_string: str) -> Optional[TypeInfo]:
        """
        Resolve a ROS2 type string to its Python class.

        Args:
            type_string: Type string like "std_msgs/msg/String" or "std_srvs/srv/Empty"

        Returns:
            TypeInfo if resolvable, None otherwise
        """
        if type_string in self._cache:
            return self._cache[type_string]

        try:
            # Parse type string: "package/category/TypeName"
            parts = type_string.split('/')
            if len(parts) != 3:
                logger.warning(f"Invalid type format: {type_string}")
                return None

            package, category, type_name = parts

            if package in self._failed_packages:
                return None

            # Try to import the module
            module_name = f"{package}.{category}"
            try:
                module = importlib.import_module(module_name)
            except ImportError as e:
                logger.debug(f"Cannot import {module_name}: {e}")
                self._failed_packages.add(package)
                return None

            # Get the class
            if not hasattr(module, type_name):
                logger.debug(f"Type {type_name} not found in {module_name}")
                return None

            python_class = getattr(module, type_name)

            info = TypeInfo(
                type_string=type_string,
                package=package,
                category=category,
                name=type_name,
                python_class=python_class,
                is_bridgeable=True
            )
            self._cache[type_string] = info
            return info

        except Exception as e:
            logger.warning(f"Failed to resolve {type_string}: {e}")
            info = TypeInfo(
                type_string=type_string,
                package=parts[0] if len(parts) > 0 else "",
                category=parts[1] if len(parts) > 1 else "",
                name=parts[2] if len(parts) > 2 else "",
                python_class=None,
                is_bridgeable=False,
                error=str(e)
            )
            self._cache[type_string] = info
            return info

    def is_bridgeable(self, type_string: str) -> bool:
        """Check if a type can be bridged (imported and used)."""
        info = self.resolve(type_string)
        return info is not None and info.is_bridgeable

    def get_class(self, type_string: str) -> Optional[Type]:
        """Get the Python class for a type string."""
        info = self.resolve(type_string)
        return info.python_class if info and info.is_bridgeable else None


def msg_to_dict(msg: Any) -> Any:
    """
    Convert a ROS2 message to a JSON-serializable dictionary.

    Handles nested messages, arrays, and numpy types.
    """
    if msg is None:
        return None

    # Handle primitive types
    if isinstance(msg, (bool, int, float, str)):
        return msg

    # Handle bytes
    if isinstance(msg, bytes):
        return list(msg)

    # Handle numpy arrays and array.array
    if hasattr(msg, 'tolist'):
        return msg.tolist()

    # Handle sequences (list, tuple)
    if isinstance(msg, (list, tuple)):
        return [msg_to_dict(item) for item in msg]

    # Handle ROS2 messages (have __slots__)
    if hasattr(msg, '__slots__'):
        result = {}
        for slot in msg.__slots__:
            # Remove leading underscore from slot name
            attr_name = slot[1:] if slot.startswith('_') else slot
            try:
                value = getattr(msg, attr_name)
                result[attr_name] = msg_to_dict(value)
            except AttributeError:
                # Try with the original slot name
                try:
                    value = getattr(msg, slot)
                    result[attr_name] = msg_to_dict(value)
                except AttributeError:
                    pass
        return result

    # Handle dict-like objects
    if isinstance(msg, dict):
        return {k: msg_to_dict(v) for k, v in msg.items()}

    # Fallback: try to convert to string
    try:
        return str(msg)
    except Exception:
        return f"<unconvertible: {type(msg).__name__}>"


def dict_to_msg(data: dict, msg_class: Type) -> Any:
    """
    Convert a dictionary to a ROS2 message.

    Args:
        data: Dictionary with field values
        msg_class: The ROS2 message class to instantiate

    Returns:
        Populated message instance
    """
    msg = msg_class()

    for key, value in data.items():
        if not hasattr(msg, key):
            continue

        current_value = getattr(msg, key)

        # Handle nested messages
        if hasattr(current_value, '__slots__'):
            if isinstance(value, dict):
                nested_msg = dict_to_msg(value, type(current_value))
                setattr(msg, key, nested_msg)
        # Handle arrays
        elif isinstance(current_value, (list, tuple)) or hasattr(current_value, 'tolist'):
            if isinstance(value, (list, tuple)):
                # Check if it's an array of messages
                if len(value) > 0 and isinstance(value[0], dict):
                    # Need to determine element type - this is tricky
                    # For now, just set the value directly for primitive arrays
                    setattr(msg, key, value)
                else:
                    setattr(msg, key, value)
            else:
                setattr(msg, key, value)
        else:
            setattr(msg, key, value)

    return msg


# Global resolver instance
_resolver: Optional[TypeResolver] = None


def get_resolver() -> TypeResolver:
    """Get or create the global type resolver."""
    global _resolver
    if _resolver is None:
        _resolver = TypeResolver()
    return _resolver
