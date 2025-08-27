# leremix_camera

RealSense camera integration package for the LeRemix robot, providing depth and color streams with compressed image transport and laser scan conversion capabilities.

## Features

- **Intel RealSense Integration**: Full support for RealSense camera family (D435i, D455, L515, etc.)
- **Compressed Image Transport**: Bandwidth-efficient image streaming for both color and depth
- **Depth to Laser Scan**: Convert depth images to 2D laser scans for navigation
- **Stream Alignment**: Aligned depth and color streams for computer vision tasks
- **Configurable Parameters**: Flexible launch arguments for different use cases
- **TF2 Integration**: Automatic camera frame transforms

## Supported Hardware

- Intel RealSense D435i (recommended)
- Intel RealSense D455
- Intel RealSense L515
- Other RealSense depth cameras

## Topics Published

### Camera Streams
- `/camera/color/image_raw` - Color image stream (640x480@30Hz)
- `/camera/color/image_raw/compressed` - Compressed color image
- `/camera/color/camera_info` - Color camera calibration info
- `/camera/aligned_depth_to_color/image_raw` - Aligned depth image
- `/camera/aligned_depth_to_color/image_raw/compressedDepth` - Compressed depth image
- `/camera/aligned_depth_to_color/camera_info` - Depth camera calibration info

### Laser Scan
- `/camera/scan` - 2D laser scan converted from depth image

### Transform Frames
- `camera_link` - Camera base frame
- `camera_depth_frame` - Depth sensor frame
- `camera_color_frame` - Color sensor frame

## Usage

### Basic Launch

```bash
# Launch with default settings
ros2 launch leremix_camera camera.launch.py

# Launch with specific device type
ros2 launch leremix_camera camera.launch.py device_type:=d435i

# Launch without compressed transport
ros2 launch leremix_camera camera.launch.py enable_compressed:=false

# Launch without laser scan conversion
ros2 launch leremix_camera camera.launch.py enable_laser_scan:=false
```

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `camera_name` | `camera` | RealSense camera namespace |
| `device_type` | `` | RealSense device type (auto-detect if empty) |
| `enable_compressed` | `true` | Enable compressed image transport |
| `enable_laser_scan` | `true` | Enable depth to laser scan conversion |
| `laser_scan_range_min` | `0.2` | Minimum laser scan range (meters) |
| `laser_scan_range_max` | `10.0` | Maximum laser scan range (meters) |

### Integration with Navigation

```bash
# Launch camera with navigation stack
ros2 launch leremix_camera camera.launch.py &
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false

# The laser scan topic /camera/scan can be used as input to nav2
```

### View Camera Streams

```bash
# View color stream
ros2 run rqt_image_view rqt_image_view /camera/color/image_raw

# View depth stream  
ros2 run rqt_image_view rqt_image_view /camera/aligned_depth_to_color/image_raw

# View laser scan in RViz
ros2 run rviz2 rviz2
# Add LaserScan display, set topic to /camera/scan
```

## Configuration

### RealSense Parameters

Edit `config/realsense.yaml` to customize camera settings:

```yaml
realsense2_camera:
  ros__parameters:
    # Resolution and framerate
    depth_width: 640
    depth_height: 480
    depth_fps: 30
    color_width: 640
    color_height: 480  
    color_fps: 30
    
    # Processing filters
    spatial_filter.enable: true
    temporal_filter.enable: true
    hole_filling_filter.enable: true
```

### Laser Scan Parameters

Edit `config/depthimage_to_laserscan.yaml` for laser scan settings:

```yaml
depthimage_to_laserscan:
  ros__parameters:
    scan_height: 10      # Pixel rows for scanning
    range_min: 0.2       # Minimum range (m)
    range_max: 10.0      # Maximum range (m)
    scan_time: 0.033     # Scan period (30Hz)
```

### Camera Mounting

Update the camera transform in the launch file to match your robot's camera mounting:

```python
# In camera.launch.py, modify these values:
arguments=[
    '0.15', '0', '0.3',  # x, y, z translation (meters)
    '0', '0', '0', '1',  # quaternion rotation
    'base_link',         # parent frame  
    'camera_link'        # child frame
]
```

## Installation

### Prerequisites

```bash
# Install RealSense SDK and ROS wrapper
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-description

# Install image transport packages
sudo apt install ros-humble-image-transport ros-humble-compressed-image-transport

# Install depth to laser scan
sudo apt install ros-humble-depthimage-to-laserscan

# Install additional dependencies
sudo apt install ros-humble-tf2-ros
```

### Build Package

```bash
cd /path/to/your/ros2_workspace
colcon build --packages-select leremix_camera
source install/setup.bash
```

## Troubleshooting

### Camera Not Detected

```bash
# Check if RealSense camera is connected
rs-enumerate-devices

# Check USB connection and permissions
lsusb | grep Intel
sudo chmod 666 /dev/video*
```

### Low Frame Rate / Bandwidth Issues

```bash
# Use compressed transport
ros2 launch leremix_camera camera.launch.py enable_compressed:=true

# Reduce resolution in config/realsense.yaml
depth_width: 424
depth_height: 240
```

### No Laser Scan Output

```bash
# Check depth image is publishing
ros2 topic echo /camera/aligned_depth_to_color/image_raw --once

# Verify depthimage_to_laserscan node is running
ros2 node list | grep depthimage

# Check laser scan topic
ros2 topic echo /camera/scan --once
```

### TF Frame Issues

```bash
# View transform tree
ros2 run tf2_tools view_frames

# Check camera transforms
ros2 run tf2_ros tf2_echo base_link camera_link
```

## Performance Optimization

### Bandwidth Reduction
- Enable compressed transport: `enable_compressed:=true`
- Disable point cloud: `pointcloud.enable: false`
- Use lower resolution: Modify width/height in config
- Disable infrared streams: `enable_infra1: false`

### Processing Optimization
- Enable depth filters for better quality
- Use appropriate QoS settings for your network
- Adjust laser scan parameters for your use case

## Integration with LeRemix

This camera package is designed to work with other LeRemix components:

- **leremix_description**: Camera frames defined in URDF
- **leremix_gazebo**: Camera simulation integration
- **Navigation Stack**: Laser scan input for SLAM/navigation
- **Computer Vision**: Aligned depth and color streams

## RealSense SDK Tools

```bash
# View camera streams
realsense-viewer

# Enumerate connected devices
rs-enumerate-devices

# Firmware update
rs-fw-update

# Depth quality tool
rs-depth-quality
```

## License

Compatible with the LeRemix robot system licensing.