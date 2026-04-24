#!/usr/bin/env python3
"""
LLMy Navigation Mode Manager

Orchestrates switching between navigation modes:
- navigation: Map-based autonomous navigation (map_server + AMCL + Nav2)
- mapping: SLAM for building maps (slam_toolbox)
- mapfree: Local navigation without a map (Nav2 with rolling costmaps)

Phase 1 Implementation: Uses subprocess spawning for mode switching.
Phase 2 will migrate to lifecycle-based management for faster switching.

Services:
- /nav/change_mode: Switch between modes
- /nav/save_map: Save current SLAM map (mapping mode only)

Topics:
- /nav/mode: Current mode (String)
- /nav/maps: Available maps (String, JSON list)
- /nav/current_map: Active map name (String)
"""

import os
import sys
import glob
import json
import signal
import subprocess
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class ModeManager(Node):
    """Navigation mode manager for LLMy robot."""

    VALID_MODES = ['navigation', 'mapping', 'mapfree', 'none']

    def __init__(self):
        super().__init__('mode_manager')

        # Declare parameters
        self.declare_parameter('maps_dir', '~/.llmy/maps')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('default_mode', 'none')
        self.declare_parameter('default_map', 'home.yaml')

        # Get parameters
        maps_dir_param = self.get_parameter('maps_dir').get_parameter_value().string_value
        self.maps_dir = Path(os.path.expanduser(maps_dir_param))
        self.auto_start = self.get_parameter('auto_start').get_parameter_value().bool_value
        self.default_mode = self.get_parameter('default_mode').get_parameter_value().string_value
        self.default_map = self.get_parameter('default_map').get_parameter_value().string_value

        # Ensure maps directory exists
        self.maps_dir.mkdir(parents=True, exist_ok=True)

        # State directory for persistence
        self.state_dir = self.maps_dir.parent
        self.mode_file = self.state_dir / '.last_mode'
        self.map_file = self.state_dir / '.last_map'

        # Current state
        self.current_mode = 'none'
        self.current_map = self.load_last_map()
        self.available_maps = self.discover_maps()
        self.current_process: Optional[subprocess.Popen] = None

        # Publishers
        self.mode_pub = self.create_publisher(String, '/nav/mode', 10)
        self.maps_pub = self.create_publisher(String, '/nav/maps', 10)
        self.current_map_pub = self.create_publisher(String, '/nav/current_map', 10)

        # Services
        self.create_service(Trigger, '/nav/change_mode', self.change_mode_callback)
        self.create_service(Trigger, '/nav/save_map', self.save_map_callback)
        self.create_service(Trigger, '/nav/stop', self.stop_callback)

        # Note: For full functionality, we need custom service types.
        # For Phase 1, we use Trigger services with mode/map in a simple format.
        # Real implementation would use:
        # - ChangeMode.srv: string mode -> bool success, string message
        # - SaveMap.srv: string name, bool overwrite -> bool success, string message
        # - ChangeMap.srv: string map_name -> bool success, string message

        # Status publishing timer
        self.status_timer = self.create_timer(1.0, self.publish_status)

        # Auto-start timer (delayed)
        if self.auto_start:
            self.startup_timer = self.create_timer(3.0, self.auto_start_mode)
        else:
            self.startup_timer = None

        self.get_logger().info(f'Mode Manager initialized')
        self.get_logger().info(f'Maps directory: {self.maps_dir}')
        self.get_logger().info(f'Available maps: {self.available_maps}')

    # =========================================================================
    # Persistence
    # =========================================================================

    def load_last_mode(self) -> str:
        """Load the last used mode from disk."""
        if self.mode_file.exists():
            try:
                mode = self.mode_file.read_text().strip()
                if mode in self.VALID_MODES:
                    return mode
            except Exception as e:
                self.get_logger().warn(f'Failed to load last mode: {e}')
        return self.default_mode

    def save_last_mode(self, mode: str):
        """Save the current mode to disk."""
        try:
            self.mode_file.write_text(mode)
        except Exception as e:
            self.get_logger().warn(f'Failed to save mode: {e}')

    def load_last_map(self) -> str:
        """Load the last used map from disk."""
        if self.map_file.exists():
            try:
                return self.map_file.read_text().strip()
            except Exception as e:
                self.get_logger().warn(f'Failed to load last map: {e}')
        return self.default_map

    def save_last_map(self, map_name: str):
        """Save the current map to disk."""
        try:
            self.map_file.write_text(map_name)
        except Exception as e:
            self.get_logger().warn(f'Failed to save map: {e}')

    # =========================================================================
    # Map Discovery
    # =========================================================================

    def discover_maps(self) -> list[str]:
        """Discover available map files."""
        pattern = self.maps_dir / '*.yaml'
        yaml_files = glob.glob(str(pattern))
        maps = [Path(f).name for f in yaml_files]
        return sorted(maps)

    def refresh_maps(self):
        """Refresh the list of available maps."""
        self.available_maps = self.discover_maps()

    # =========================================================================
    # Process Management (Phase 1: Subprocess)
    # =========================================================================

    def launch_mode(self, mode: str) -> bool:
        """Launch a navigation mode as a subprocess."""
        if mode == 'none':
            return True

        # Build launch command
        if mode == 'navigation':
            map_path = self.maps_dir / self.current_map
            if not map_path.exists():
                self.get_logger().error(f'Map not found: {map_path}')
                return False
            cmd = [
                'ros2', 'launch', 'llmy_nav', 'navigation.launch.py',
                f'map:={map_path}'
            ]
        elif mode == 'mapping':
            cmd = ['ros2', 'launch', 'llmy_nav', 'mapping.launch.py']
        elif mode == 'mapfree':
            cmd = ['ros2', 'launch', 'llmy_nav', 'mapfree.launch.py']
        else:
            self.get_logger().error(f'Invalid mode: {mode}')
            return False

        self.get_logger().info(f'Launching {mode} mode: {" ".join(cmd)}')

        try:
            # Launch in new process group for clean shutdown
            self.current_process = subprocess.Popen(
                cmd,
                preexec_fn=os.setsid,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )

            # Wait briefly and check if still running
            time.sleep(2.0)
            if self.current_process.poll() is not None:
                self.get_logger().error(f'Mode {mode} failed to start')
                return False

            return True

        except Exception as e:
            self.get_logger().error(f'Failed to launch {mode}: {e}')
            return False

    def kill_current_process(self) -> bool:
        """Kill the current mode process."""
        if self.current_process is None:
            return True

        self.get_logger().info(f'Stopping {self.current_mode} mode...')

        try:
            # Send SIGINT to process group
            pgid = os.getpgid(self.current_process.pid)
            os.killpg(pgid, signal.SIGINT)

            # Wait for graceful shutdown
            try:
                self.current_process.wait(timeout=10.0)
                self.get_logger().info('Mode stopped gracefully')
            except subprocess.TimeoutExpired:
                # Force kill
                self.get_logger().warn('Mode did not stop, force killing...')
                os.killpg(pgid, signal.SIGKILL)
                self.current_process.wait(timeout=5.0)

            self.current_process = None
            return True

        except Exception as e:
            self.get_logger().error(f'Error stopping mode: {e}')
            self.current_process = None
            return False

    # =========================================================================
    # Mode Switching
    # =========================================================================

    def switch_mode(self, target_mode: str) -> tuple[bool, str]:
        """Switch to a new navigation mode."""
        if target_mode not in self.VALID_MODES:
            return False, f'Invalid mode: {target_mode}'

        if target_mode == self.current_mode:
            # Check if process is still running
            if self.current_process and self.current_process.poll() is None:
                return True, f'Already in {target_mode} mode'
            # Process died, restart
            self.get_logger().warn(f'{target_mode} mode process died, restarting...')

        self.get_logger().info(f'Switching from {self.current_mode} to {target_mode}')

        # Stop current mode
        if self.current_mode != 'none':
            if not self.kill_current_process():
                return False, f'Failed to stop {self.current_mode} mode'
            time.sleep(1.0)  # Brief pause between modes

        # Start new mode
        if target_mode != 'none':
            if not self.launch_mode(target_mode):
                self.current_mode = 'none'
                return False, f'Failed to start {target_mode} mode'

        self.current_mode = target_mode
        self.save_last_mode(target_mode)

        return True, f'Switched to {target_mode} mode'

    # =========================================================================
    # Service Callbacks
    # =========================================================================

    def change_mode_callback(self, request, response):
        """
        Handle mode change requests.

        Note: Using Trigger service for Phase 1.
        The mode should be passed via a topic or we parse the last message.
        For now, this cycles through modes for testing.

        TODO Phase 2: Use proper ChangeMode.srv with mode field.
        """
        # For Phase 1, cycle through modes for testing
        modes = ['none', 'mapfree', 'mapping', 'navigation']
        try:
            current_idx = modes.index(self.current_mode)
            next_idx = (current_idx + 1) % len(modes)
            target_mode = modes[next_idx]
        except ValueError:
            target_mode = 'none'

        success, message = self.switch_mode(target_mode)
        response.success = success
        response.message = message
        return response

    def save_map_callback(self, request, response):
        """
        Save the current SLAM map.

        Note: Using Trigger service for Phase 1.
        Map name would typically come from the request.

        TODO Phase 2: Use proper SaveMap.srv with name and overwrite fields.
        """
        if self.current_mode != 'mapping':
            response.success = False
            response.message = 'Must be in mapping mode to save map'
            return response

        # Generate map name with timestamp
        import datetime
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        map_name = f'map_{timestamp}'
        map_path = self.maps_dir / map_name

        # Use map_saver_cli
        cmd = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', str(map_path),
            '--ros-args', '-p', 'save_map_timeout:=5000.0'
        ]

        self.get_logger().info(f'Saving map to {map_path}')

        try:
            result = subprocess.run(cmd, timeout=30, capture_output=True, text=True)
            if result.returncode == 0:
                self.refresh_maps()
                response.success = True
                response.message = f'Map saved as {map_name}.yaml'
            else:
                response.success = False
                response.message = f'Map save failed: {result.stderr}'
        except subprocess.TimeoutExpired:
            response.success = False
            response.message = 'Map save timed out'
        except Exception as e:
            response.success = False
            response.message = f'Map save error: {e}'

        return response

    def stop_callback(self, request, response):
        """Stop current navigation mode."""
        success, message = self.switch_mode('none')
        response.success = success
        response.message = message
        return response

    # =========================================================================
    # Status Publishing
    # =========================================================================

    def publish_status(self):
        """Publish current status on topics."""
        # Current mode
        mode_msg = String()
        mode_msg.data = self.current_mode
        self.mode_pub.publish(mode_msg)

        # Available maps
        maps_msg = String()
        maps_msg.data = json.dumps({'available_maps': self.available_maps})
        self.maps_pub.publish(maps_msg)

        # Current map
        map_msg = String()
        map_msg.data = self.current_map
        self.current_map_pub.publish(map_msg)

    # =========================================================================
    # Auto-Start
    # =========================================================================

    def auto_start_mode(self):
        """Auto-start the last used mode."""
        if self.startup_timer:
            self.startup_timer.cancel()
            self.startup_timer = None

        last_mode = self.load_last_mode()
        if last_mode and last_mode != 'none':
            self.get_logger().info(f'Auto-starting {last_mode} mode...')
            success, message = self.switch_mode(last_mode)
            if not success:
                self.get_logger().error(f'Auto-start failed: {message}')

    # =========================================================================
    # Cleanup
    # =========================================================================

    def destroy_node(self):
        """Clean shutdown."""
        self.get_logger().info('Shutting down mode manager...')
        self.kill_current_process()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    mode_manager = ModeManager()

    try:
        rclpy.spin(mode_manager)
    except KeyboardInterrupt:
        pass
    finally:
        mode_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
