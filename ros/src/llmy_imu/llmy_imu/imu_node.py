#!/usr/bin/env python3
"""ROS2 IMU node for LSM6DSOX (accel/gyro) + MMC5983MA (magnetometer).

Publishes raw IMU data and magnetometer data. Orientation fusion is handled
by imu_filter_madgwick launched alongside this node.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

from .lsm6dsox import LSM6DSOX
from .mmc5983ma import MMC5983MA


class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # Declare parameters
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('imu_addr', 0x6B)
        self.declare_parameter('mag_addr', 0x30)
        self.declare_parameter('use_mag', True)

        # Load parameters
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        i2c_bus = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        imu_addr = self.get_parameter('imu_addr').get_parameter_value().integer_value
        mag_addr = self.get_parameter('mag_addr').get_parameter_value().integer_value
        self.use_mag = self.get_parameter('use_mag').get_parameter_value().bool_value

        # Initialize LSM6DSOX
        self.imu = None
        try:
            self.imu = LSM6DSOX(i2c_bus, imu_addr)
            self.imu.initialize()
            self.get_logger().info(
                f'LSM6DSOX initialized on bus {i2c_bus}, addr 0x{imu_addr:02X}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize LSM6DSOX: {e}')
            return

        # Initialize MMC5983MA
        self.mag = None
        if self.use_mag:
            try:
                self.mag = MMC5983MA(i2c_bus, mag_addr)
                self.mag.initialize()
                self.get_logger().info(
                    f'MMC5983MA initialized on bus {i2c_bus}, addr 0x{mag_addr:02X}')
            except Exception as e:
                self.get_logger().warn(f'Failed to initialize MMC5983MA: {e}')
                self.get_logger().warn('Continuing without magnetometer')

        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/raw', 10)
        self.get_logger().info('  Publishing: imu/raw')

        if self.mag is not None:
            self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
            self.get_logger().info('  Publishing: imu/mag')

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish)
        self.get_logger().info(
            f'IMU node started at {self.publish_rate} Hz, frame_id={self.frame_id}')

    def _publish(self):
        if self.imu is None:
            return

        try:
            ax, ay, az = self.imu.acceleration
            gx, gy, gz = self.imu.gyro
        except Exception as e:
            self.get_logger().error(f'IMU read error: {e}')
            return

        now = self.get_clock().now().to_msg()

        # Apply frame transform: sensor Y -> -Y for ROS convention
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self.frame_id

        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = -ay
        imu_msg.linear_acceleration.z = az

        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = -gy
        imu_msg.angular_velocity.z = gz

        # No orientation from raw sensor
        imu_msg.orientation_covariance[0] = -1.0

        imu_msg.linear_acceleration_covariance[0] = 0.01
        imu_msg.linear_acceleration_covariance[4] = 0.01
        imu_msg.linear_acceleration_covariance[8] = 0.01

        imu_msg.angular_velocity_covariance[0] = 0.01
        imu_msg.angular_velocity_covariance[4] = 0.01
        imu_msg.angular_velocity_covariance[8] = 0.01

        self.imu_pub.publish(imu_msg)

        # Magnetometer
        if self.mag is not None:
            try:
                mx, my, mz = self.mag.magnetic
            except Exception as e:
                self.get_logger().error(f'Mag read error: {e}')
                return

            mag_msg = MagneticField()
            mag_msg.header.stamp = now
            mag_msg.header.frame_id = self.frame_id

            # Convert µT to Tesla, apply Y negation
            mag_msg.magnetic_field.x = mx * 1e-6
            mag_msg.magnetic_field.y = -my * 1e-6
            mag_msg.magnetic_field.z = mz * 1e-6

            mag_msg.magnetic_field_covariance[0] = 0.01
            mag_msg.magnetic_field_covariance[4] = 0.01
            mag_msg.magnetic_field_covariance[8] = 0.01

            self.mag_pub.publish(mag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
