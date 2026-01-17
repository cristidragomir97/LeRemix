#!/usr/bin/env python3
"""
ICM20948 IMU Node using direct smbus2 access.
Works in Docker containers without platform detection issues.
"""

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import smbus2


class ICM20948:
    """Direct smbus2 driver for ICM20948 9-DOF IMU"""

    # Register addresses (Bank 0)
    REG_BANK_SEL = 0x7F
    WHO_AM_I = 0x00
    PWR_MGMT_1 = 0x06
    PWR_MGMT_2 = 0x07
    ACCEL_XOUT_H = 0x2D
    GYRO_XOUT_H = 0x33
    TEMP_OUT_H = 0x39

    # Bank 2 registers
    GYRO_SMPLRT_DIV = 0x00
    GYRO_CONFIG_1 = 0x01
    ACCEL_SMPLRT_DIV_1 = 0x10
    ACCEL_SMPLRT_DIV_2 = 0x11
    ACCEL_CONFIG = 0x14

    # Bank 3 registers (magnetometer passthrough)
    I2C_MST_CTRL = 0x01
    I2C_SLV0_ADDR = 0x03
    I2C_SLV0_REG = 0x04
    I2C_SLV0_CTRL = 0x05

    # Magnetometer (AK09916) registers
    AK09916_I2C_ADDR = 0x0C
    AK09916_WIA2 = 0x01
    AK09916_ST1 = 0x10
    AK09916_HXL = 0x11
    AK09916_CNTL2 = 0x31
    AK09916_CNTL3 = 0x32

    # Scale factors
    ACCEL_SCALE_2G = 16384.0  # LSB/g for ±2g range
    GYRO_SCALE_250DPS = 131.0  # LSB/(°/s) for ±250°/s range
    MAG_SCALE = 0.15  # µT/LSB

    def __init__(self, bus_num, address=0x69):
        self.bus = smbus2.SMBus(bus_num)
        self.address = address
        self._initialized = False

    def initialize(self):
        """Initialize the ICM20948"""
        # Check WHO_AM_I
        who_am_i = self._read_byte(self.WHO_AM_I)
        if who_am_i != 0xEA:
            raise RuntimeError(f"ICM20948 not found. WHO_AM_I=0x{who_am_i:02X}, expected 0xEA")

        # Select Bank 0
        self._set_bank(0)

        # Check power state
        pwr = self._read_byte(self.PWR_MGMT_1)
        if pwr & 0x40:  # Sleep bit set
            # Try to wake device
            self._write_byte(self.PWR_MGMT_1, 0x00)
            time.sleep(0.1)
            pwr = self._read_byte(self.PWR_MGMT_1)
            if pwr & 0x40:
                raise RuntimeError(f"Cannot wake ICM20948 - PWR_MGMT_1=0x{pwr:02X}. Hardware defect?")

        # Enable all sensors (accel + gyro)
        self._write_byte(self.PWR_MGMT_2, 0x00)
        time.sleep(0.05)

        # Configure gyro and accel (Bank 2)
        self._set_bank(2)
        self._write_byte(self.GYRO_SMPLRT_DIV, 0x04)  # ~220Hz
        self._write_byte(self.GYRO_CONFIG_1, 0x01)    # ±250°/s, DLPF enabled
        self._write_byte(self.ACCEL_SMPLRT_DIV_1, 0x00)
        self._write_byte(self.ACCEL_SMPLRT_DIV_2, 0x04)  # ~220Hz
        self._write_byte(self.ACCEL_CONFIG, 0x01)    # ±2g, DLPF enabled

        # Setup I2C master for magnetometer (Bank 3)
        self._set_bank(3)
        self._write_byte(self.I2C_MST_CTRL, 0x07)  # 400kHz I2C master clock

        # Reset magnetometer
        self._write_mag(self.AK09916_CNTL3, 0x01)
        time.sleep(0.1)

        # Set magnetometer to continuous mode 4 (100Hz)
        self._write_mag(self.AK09916_CNTL2, 0x08)
        time.sleep(0.01)

        # Setup auto-read of magnetometer data
        self._set_bank(3)
        self._write_byte(self.I2C_SLV0_ADDR, self.AK09916_I2C_ADDR | 0x80)  # Read mode
        self._write_byte(self.I2C_SLV0_REG, self.AK09916_HXL)
        self._write_byte(self.I2C_SLV0_CTRL, 0x88)  # Enable, read 8 bytes

        # Back to Bank 0
        self._set_bank(0)

        # Enable I2C master
        self._write_byte(0x03, 0x20)  # USER_CTRL: I2C_MST_EN

        self._initialized = True
        return True

    def _set_bank(self, bank):
        """Select register bank (0-3)"""
        self._write_byte(self.REG_BANK_SEL, (bank & 0x03) << 4)
        time.sleep(0.001)

    def _read_byte(self, reg):
        """Read a single byte from register"""
        return self.bus.read_byte_data(self.address, reg)

    def _write_byte(self, reg, value):
        """Write a single byte to register"""
        self.bus.write_byte_data(self.address, reg, value)

    def _read_bytes(self, reg, count):
        """Read multiple bytes starting from register"""
        return self.bus.read_i2c_block_data(self.address, reg, count)

    def _write_mag(self, reg, value):
        """Write to magnetometer via I2C master"""
        self._set_bank(3)
        self._write_byte(self.I2C_SLV0_ADDR, self.AK09916_I2C_ADDR)  # Write mode
        self._write_byte(self.I2C_SLV0_REG, reg)
        self._write_byte(0x06, value)  # I2C_SLV0_DO
        self._write_byte(self.I2C_SLV0_CTRL, 0x81)  # Enable, write 1 byte
        time.sleep(0.01)

    def _to_signed_16(self, msb, lsb):
        """Convert two bytes to signed 16-bit value"""
        value = (msb << 8) | lsb
        if value > 32767:
            value -= 65536
        return value

    @property
    def acceleration(self):
        """Read accelerometer data in m/s²"""
        self._set_bank(0)
        data = self._read_bytes(self.ACCEL_XOUT_H, 6)
        ax = self._to_signed_16(data[0], data[1]) / self.ACCEL_SCALE_2G * 9.80665
        ay = self._to_signed_16(data[2], data[3]) / self.ACCEL_SCALE_2G * 9.80665
        az = self._to_signed_16(data[4], data[5]) / self.ACCEL_SCALE_2G * 9.80665
        return (ax, ay, az)

    @property
    def gyro(self):
        """Read gyroscope data in rad/s"""
        self._set_bank(0)
        data = self._read_bytes(self.GYRO_XOUT_H, 6)
        gx = self._to_signed_16(data[0], data[1]) / self.GYRO_SCALE_250DPS * math.pi / 180.0
        gy = self._to_signed_16(data[2], data[3]) / self.GYRO_SCALE_250DPS * math.pi / 180.0
        gz = self._to_signed_16(data[4], data[5]) / self.GYRO_SCALE_250DPS * math.pi / 180.0
        return (gx, gy, gz)

    @property
    def magnetic(self):
        """Read magnetometer data in µT"""
        self._set_bank(0)
        # Magnetometer data is at EXT_SLV_SENS_DATA (0x3B)
        data = self._read_bytes(0x3B, 8)
        # AK09916 data is little-endian
        mx = self._to_signed_16(data[1], data[0]) * self.MAG_SCALE
        my = self._to_signed_16(data[3], data[2]) * self.MAG_SCALE
        mz = self._to_signed_16(data[5], data[4]) * self.MAG_SCALE
        return (mx, my, mz)

    def close(self):
        """Close the I2C bus"""
        self.bus.close()


class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # Declare parameters
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('i2c_addr', 0x69)
        self.declare_parameter('i2c_bus', 7)

        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        i2c_addr = self.get_parameter('i2c_addr').get_parameter_value().integer_value
        i2c_bus_num = self.get_parameter('i2c_bus').get_parameter_value().integer_value

        self.icm = None

        # Initialize ICM20948 using direct smbus2 driver
        try:
            self.icm = ICM20948(i2c_bus_num, i2c_addr)
            self.icm.initialize()
            self.get_logger().info(f'ICM20948 initialized on bus {i2c_bus_num}, addr 0x{i2c_addr:02x}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize ICM20948: {str(e)}')
            self.get_logger().error('Check I2C connection and ensure device is properly powered.')
            return

        # Create publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)

        # Create timer
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_imu_data)

        self.get_logger().info(f'IMU node started, publishing at {self.publish_rate} Hz')

    def publish_imu_data(self):
        if self.icm is None or not self.icm._initialized:
            return

        try:
            # Read sensor data
            raw_accel_x, raw_accel_y, raw_accel_z = self.icm.acceleration
            raw_gyro_x, raw_gyro_y, raw_gyro_z = self.icm.gyro
            raw_mag_x, raw_mag_y, raw_mag_z = self.icm.magnetic

            # Apply frame transform: sensor Y points right, ROS Y should point left
            # Robot frame: X forward, Y left, Z up (ROS convention)
            accel_x = raw_accel_x
            accel_y = -raw_accel_y
            accel_z = raw_accel_z

            gyro_x = raw_gyro_x
            gyro_y = -raw_gyro_y
            gyro_z = raw_gyro_z

            mag_x = raw_mag_x
            mag_y = -raw_mag_y
            mag_z = raw_mag_z

            # Get current timestamp
            now = self.get_clock().now()

            # Create and publish IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = now.to_msg()
            imu_msg.header.frame_id = self.frame_id

            # Linear acceleration (m/s²)
            imu_msg.linear_acceleration.x = accel_x
            imu_msg.linear_acceleration.y = accel_y
            imu_msg.linear_acceleration.z = accel_z

            # Angular velocity (rad/s)
            imu_msg.angular_velocity.x = gyro_x
            imu_msg.angular_velocity.y = gyro_y
            imu_msg.angular_velocity.z = gyro_z

            # Orientation not provided, set covariance to -1
            imu_msg.orientation_covariance[0] = -1.0

            # Set covariance matrices
            imu_msg.linear_acceleration_covariance[0] = 0.01
            imu_msg.linear_acceleration_covariance[4] = 0.01
            imu_msg.linear_acceleration_covariance[8] = 0.01

            imu_msg.angular_velocity_covariance[0] = 0.01
            imu_msg.angular_velocity_covariance[4] = 0.01
            imu_msg.angular_velocity_covariance[8] = 0.01

            self.imu_pub.publish(imu_msg)

            # Create and publish MagneticField message (convert µT to T)
            mag_msg = MagneticField()
            mag_msg.header.stamp = now.to_msg()
            mag_msg.header.frame_id = self.frame_id
            mag_msg.magnetic_field.x = mag_x * 1e-6
            mag_msg.magnetic_field.y = mag_y * 1e-6
            mag_msg.magnetic_field.z = mag_z * 1e-6

            mag_msg.magnetic_field_covariance[0] = 0.01
            mag_msg.magnetic_field_covariance[4] = 0.01
            mag_msg.magnetic_field_covariance[8] = 0.01

            self.mag_pub.publish(mag_msg)

        except Exception as e:
            self.get_logger().error(f'Error reading IMU data: {str(e)}')


def main(args=None):
    rclpy.init(args=args)

    imu_node = IMUNode()

    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        pass
    finally:
        imu_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
