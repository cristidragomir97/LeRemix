# llmy_imu

ROS2 package for reading IMU data from an ICM20948 sensor using the Adafruit CircuitPython ICM20X library.

## Features

- Reads acceleration, gyroscope, and magnetometer data from ICM20948
- Publishes raw IMU data and separate magnetometer data
- Includes Madgwick filter for orientation estimation
- Configurable publishing rate and frame ID

## Topics

- `/imu/raw` - Raw IMU data (sensor_msgs/Imu) with linear acceleration and angular velocity
- `/imu/mag` - Magnetometer data (sensor_msgs/MagneticField)
- `/imu/fused` - Filtered IMU data with orientation (sensor_msgs/Imu) from Madgwick filter

## Dependencies

- Adafruit CircuitPython ICM20X library
- I2C connection to ICM20948 sensor
- `imu_filter_madgwick` package

## Hardware Setup

1. Connect ICM20948 to I2C bus (SCL/SDA pins)
2. Install Adafruit CircuitPython libraries:
   ```bash
   pip3 install adafruit-circuitpython-icm20x
   ```

## Usage

### Build and Source

```bash
cd /path/to/your/ros2_ws
colcon build --packages-select llmy_imu
source install/setup.bash
```

### Launch

```bash
# Launch with default settings (50Hz, magnetometer enabled)
ros2 launch llmy_imu imu.launch.py

# Launch with custom parameters
ros2 launch llmy_imu imu.launch.py publish_rate:=100.0 frame_id:=base_imu use_mag:=false
```

### Parameters

- `publish_rate` - Publishing frequency in Hz (default: 50.0)
- `frame_id` - Frame ID for IMU data (default: "imu_link") 
- `use_mag` - Enable magnetometer in orientation filter (default: true)

### Monitor Topics

```bash
# View raw IMU data
ros2 topic echo /imu/raw

# View magnetometer data  
ros2 topic echo /imu/mag

# View fused IMU data with orientation
ros2 topic echo /imu/fused
```