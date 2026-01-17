# llmy_camera

Camera integration package for the LLMy robot, providing both a head-mounted RealSense RGB-D camera and a wrist-mounted USB camera with compressed image transport and laser scan conversion capabilities.

## Overview

The llmy_camera package manages two camera systems:
- **Head Camera**: Intel RealSense RGB-D camera (D435i, D455, etc.) mounted on the pan/tilt head
- **Wrist Camera**: USB camera (640x480) mounted on the end-effector for close-up manipulation tasks

## Launch Files

### `camera.launch.py`

Main launch file that brings up both camera systems with full feature set:

**Components:**
- RealSense RGB-D camera (head_camera namespace)
- USB wrist camera (wrist_camera namespace)
- Compressed image transport for both cameras
- Depth-to-laser-scan conversion for navigation

```bash
# Launch both cameras with all features
ros2 launch llmy_camera camera.launch.py

# Launch without wrist camera
ros2 launch llmy_camera camera.launch.py enable_wrist_camera:=false

# Launch without laser scan conversion
ros2 launch llmy_camera camera.launch.py enable_laser_scan:=false

# Disable compressed transport
ros2 launch llmy_camera camera.launch.py enable_compressed:=false
```

### `realsense_only.launch.py`

Minimal launch file for RealSense camera only (no wrist camera, no laser scan, no compression):

```bash
# Launch only RealSense camera
ros2 launch llmy_camera realsense_only.launch.py

# Specify device type
ros2 launch llmy_camera realsense_only.launch.py device_type:=d435i
```

## Launch Arguments

### `camera.launch.py` Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `camera_name` | `head_camera` | RealSense camera name |
| `device_type` | `` | RealSense device type (auto-detect if empty) |
| `enable_wrist_camera` | `true` | Enable USB wrist camera |
| `wrist_camera_device` | `/dev/video0` | Video device path for wrist camera |
| `enable_compressed` | `true` | Enable compressed image transport |
| `enable_laser_scan` | `true` | Enable depth to laser scan conversion |
| `laser_scan_min_height` | `-0.5` | Minimum height for laser scan slice (m) |
| `laser_scan_max_height` | `0.5` | Maximum height for laser scan slice (m) |
| `laser_scan_angle_min` | `-1.57` | Minimum scan angle (-90°) |
| `laser_scan_angle_max` | `1.57` | Maximum scan angle (+90°) |
| `laser_scan_angle_increment` | `0.005` | Angular resolution |
| `laser_scan_range_min` | `0.2` | Minimum range (m) |
| `laser_scan_range_max` | `10.0` | Maximum range (m) |

### `realsense_only.launch.py` Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `camera_name` | `camera` | RealSense camera name |
| `device_type` | `d435i` | RealSense device type |

## Topics Published

### Head Camera (RealSense)

**Color Stream:**
- `/head_camera/head_camera/color/image_raw` - Raw color image (640x480@30Hz)
- `/head_camera/head_camera/color/image_compressed` - Compressed color image (JPEG)
- `/head_camera/head_camera/color/camera_info` - Camera calibration

**Depth Stream:**
- `/head_camera/head_camera/depth/image_rect_raw` - Rectified depth image
- `/head_camera/head_camera/depth/camera_info` - Depth camera calibration

**Laser Scan:**
- `/scan` - 2D laser scan converted from depth (for navigation)

### Wrist Camera (USB)

- `/wrist_camera/image_raw` - Raw wrist camera image (640x480@30Hz)
- `/wrist_camera/image_compressed` - Compressed wrist image (JPEG)
- `/wrist_camera/camera_info` - Camera calibration

## Supported Hardware

### RealSense Cameras
- Intel RealSense D435i (recommended)
- Intel RealSense D455
- Intel RealSense L515
- Other RealSense RGB-D cameras

### Wrist Camera
- Any UVC-compatible USB camera
- Default: 640x480@30fps, MJPEG format

## Usage Examples

### View Camera Streams

```bash
# View head camera color stream
ros2 run rqt_image_view rqt_image_view /head_camera/head_camera/color/image_raw

# View head camera depth
ros2 run rqt_image_view rqt_image_view /head_camera/head_camera/depth/image_rect_raw

# View wrist camera
ros2 run rqt_image_view rqt_image_view /wrist_camera/image_raw

# View compressed images (bandwidth efficient)
ros2 run rqt_image_view rqt_image_view /head_camera/head_camera/color/image_compressed/compressed
ros2 run rqt_image_view rqt_image_view /wrist_camera/image_compressed/compressed
```
