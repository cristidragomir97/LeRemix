# llmy_imu_bno055

BNO055 9-DOF IMU driver for the LLMy robot. Communicates via I2C using smbus2, providing fused orientation (quaternion), raw acceleration, gyroscope, and magnetometer data.

## Launch

```bash
ros2 launch llmy_imu_bno055 imu.launch.py

# Custom I2C bus and address
ros2 launch llmy_imu_bno055 imu.launch.py i2c_bus:=1 i2c_addr:=41
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `publish_rate` | `100.0` | Publishing frequency (Hz) |
| `frame_id` | `imu_link` | TF frame ID |
| `i2c_bus` | `7` | I2C bus number |
| `i2c_addr` | `40` (0x28) | I2C device address |

## Topics Published

| Topic | Type | Description |
|-------|------|-------------|
| `/imu/data` | sensor_msgs/Imu | Fused orientation + accel + gyro |
| `/imu/raw` | sensor_msgs/Imu | Raw accel + gyro (no orientation) |
| `/imu/mag` | sensor_msgs/MagneticField | Magnetic field data |
