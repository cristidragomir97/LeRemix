#!/usr/bin/env python3
"""
BNO055 IMU Node using direct smbus2 access.
Works in Docker containers without platform detection issues.

The BNO055 is a 9-DOF IMU with built-in sensor fusion that provides
quaternion orientation output directly.
"""

import math
import time
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion
import smbus2


class BNO055:
    """Direct smbus2 driver for BNO055 9-DOF IMU"""

    # I2C addresses
    DEFAULT_ADDRESS = 0x28
    ALT_ADDRESS = 0x29

    # Register addresses
    CHIP_ID = 0x00
    PAGE_ID = 0x07
    ACCEL_DATA = 0x08
    MAG_DATA = 0x0E
    GYRO_DATA = 0x14
    EULER_DATA = 0x1A
    QUATERNION_DATA = 0x20
    LINEAR_ACCEL_DATA = 0x28
    GRAVITY_DATA = 0x2E
    TEMP = 0x34
    CALIB_STAT = 0x35
    ST_RESULT = 0x36
    INT_STA = 0x37
    SYS_CLK_STATUS = 0x38
    SYS_STATUS = 0x39
    SYS_ERR = 0x3A
    UNIT_SEL = 0x3B
    OPR_MODE = 0x3D
    PWR_MODE = 0x3E
    SYS_TRIGGER = 0x3F
    TEMP_SOURCE = 0x40
    AXIS_MAP_CONFIG = 0x41
    AXIS_MAP_SIGN = 0x42

    # Operation modes
    CONFIG_MODE = 0x00
    ACCONLY_MODE = 0x01
    MAGONLY_MODE = 0x02
    GYROONLY_MODE = 0x03
    ACCMAG_MODE = 0x04
    ACCGYRO_MODE = 0x05
    MAGGYRO_MODE = 0x06
    AMG_MODE = 0x07
    IMU_MODE = 0x08
    COMPASS_MODE = 0x09
    M4G_MODE = 0x0A
    NDOF_FMC_OFF_MODE = 0x0B
    NDOF_MODE = 0x0C

    # Power modes
    NORMAL_MODE = 0x00
    LOW_POWER_MODE = 0x01
    SUSPEND_MODE = 0x02

    # Expected chip ID
    BNO055_ID = 0xA0

    # Scale factors
    ACCEL_SCALE = 100.0  # 1 m/s² = 100 LSB
    GYRO_SCALE = 16.0    # 1 rad/s = 900 LSB (but we use dps: 16 LSB/dps)
    MAG_SCALE = 16.0     # 1 µT = 16 LSB
    EULER_SCALE = 16.0   # 1 degree = 16 LSB
    QUAT_SCALE = (1 << 14)  # 2^14 = 16384

    def __init__(self, bus_num, address=DEFAULT_ADDRESS):
        self.bus = smbus2.SMBus(bus_num)
        self.address = address
        self._initialized = False

    def initialize(self, mode=NDOF_MODE):
        """Initialize the BNO055"""
        # Check chip ID
        chip_id = self._read_byte(self.CHIP_ID)
        if chip_id != self.BNO055_ID:
            raise RuntimeError(f"BNO055 not found. CHIP_ID=0x{chip_id:02X}, expected 0x{self.BNO055_ID:02X}")

        # Switch to config mode
        self._set_mode(self.CONFIG_MODE)
        time.sleep(0.025)

        # Reset
        self._write_byte(self.SYS_TRIGGER, 0x20)
        time.sleep(0.65)

        # Wait for chip to be ready
        while self._read_byte(self.CHIP_ID) != self.BNO055_ID:
            time.sleep(0.01)

        # Set power mode to normal
        self._write_byte(self.PWR_MODE, self.NORMAL_MODE)
        time.sleep(0.01)

        # Set page to 0
        self._write_byte(self.PAGE_ID, 0x00)

        # Set units:
        # Accelerometer: m/s²
        # Gyroscope: rad/s
        # Euler angles: degrees
        # Temperature: Celsius
        # Fusion output: Windows (pitch: -180 to 180)
        # 0x04 = gyro rad/s, 0x01 = euler radians -> we want rad for gyro but degrees for euler
        # Actually: bit 1 = gyro (0=dps, 1=rps), bit 2 = euler (0=deg, 1=rad)
        # We want gyro in rad/s (bit 1 = 1) and euler in degrees (bit 2 = 0) = 0x02
        self._write_byte(self.UNIT_SEL, 0x02)
        time.sleep(0.01)

        # Trigger use of external crystal (more accurate)
        self._write_byte(self.SYS_TRIGGER, 0x80)
        time.sleep(0.5)

        # Set to requested operation mode
        self._set_mode(mode)
        time.sleep(0.03)

        self._initialized = True
        return True

    def _set_mode(self, mode):
        """Set operation mode"""
        self._write_byte(self.OPR_MODE, mode)
        time.sleep(0.03)

    def _read_byte(self, reg):
        """Read a single byte from register"""
        return self.bus.read_byte_data(self.address, reg)

    def _write_byte(self, reg, value):
        """Write a single byte to register"""
        self.bus.write_byte_data(self.address, reg, value)

    def _read_bytes(self, reg, count):
        """Read multiple bytes starting from register"""
        return self.bus.read_i2c_block_data(self.address, reg, count)

    def _to_signed_16(self, lsb, msb):
        """Convert two bytes (LSB first) to signed 16-bit value"""
        value = (msb << 8) | lsb
        if value > 32767:
            value -= 65536
        return value

    @property
    def calibration_status(self):
        """Get calibration status (sys, gyro, accel, mag) each 0-3"""
        status = self._read_byte(self.CALIB_STAT)
        sys = (status >> 6) & 0x03
        gyro = (status >> 4) & 0x03
        accel = (status >> 2) & 0x03
        mag = status & 0x03
        return (sys, gyro, accel, mag)

    @property
    def is_calibrated(self):
        """Check if fully calibrated (all values = 3)"""
        sys, gyro, accel, mag = self.calibration_status
        return sys == 3 and gyro == 3 and accel == 3 and mag == 3

    @property
    def acceleration(self):
        """Read accelerometer data in m/s²"""
        data = self._read_bytes(self.ACCEL_DATA, 6)
        ax = self._to_signed_16(data[0], data[1]) / self.ACCEL_SCALE
        ay = self._to_signed_16(data[2], data[3]) / self.ACCEL_SCALE
        az = self._to_signed_16(data[4], data[5]) / self.ACCEL_SCALE
        return (ax, ay, az)

    @property
    def linear_acceleration(self):
        """Read linear acceleration (gravity removed) in m/s²"""
        data = self._read_bytes(self.LINEAR_ACCEL_DATA, 6)
        ax = self._to_signed_16(data[0], data[1]) / self.ACCEL_SCALE
        ay = self._to_signed_16(data[2], data[3]) / self.ACCEL_SCALE
        az = self._to_signed_16(data[4], data[5]) / self.ACCEL_SCALE
        return (ax, ay, az)

    @property
    def gyro(self):
        """Read gyroscope data in rad/s"""
        data = self._read_bytes(self.GYRO_DATA, 6)
        # With UNIT_SEL bit 1 = 1, gyro is in rad/s (900 LSB per rad/s)
        gx = self._to_signed_16(data[0], data[1]) / 900.0
        gy = self._to_signed_16(data[2], data[3]) / 900.0
        gz = self._to_signed_16(data[4], data[5]) / 900.0
        return (gx, gy, gz)

    @property
    def magnetic(self):
        """Read magnetometer data in µT"""
        data = self._read_bytes(self.MAG_DATA, 6)
        mx = self._to_signed_16(data[0], data[1]) / self.MAG_SCALE
        my = self._to_signed_16(data[2], data[3]) / self.MAG_SCALE
        mz = self._to_signed_16(data[4], data[5]) / self.MAG_SCALE
        return (mx, my, mz)

    @property
    def quaternion(self):
        """Read quaternion orientation (w, x, y, z)"""
        data = self._read_bytes(self.QUATERNION_DATA, 8)
        w = self._to_signed_16(data[0], data[1]) / self.QUAT_SCALE
        x = self._to_signed_16(data[2], data[3]) / self.QUAT_SCALE
        y = self._to_signed_16(data[4], data[5]) / self.QUAT_SCALE
        z = self._to_signed_16(data[6], data[7]) / self.QUAT_SCALE
        return (w, x, y, z)

    @property
    def euler(self):
        """Read Euler angles (heading, roll, pitch) in degrees"""
        data = self._read_bytes(self.EULER_DATA, 6)
        heading = self._to_signed_16(data[0], data[1]) / self.EULER_SCALE
        roll = self._to_signed_16(data[2], data[3]) / self.EULER_SCALE
        pitch = self._to_signed_16(data[4], data[5]) / self.EULER_SCALE
        return (heading, roll, pitch)

    @property
    def gravity(self):
        """Read gravity vector in m/s²"""
        data = self._read_bytes(self.GRAVITY_DATA, 6)
        gx = self._to_signed_16(data[0], data[1]) / self.ACCEL_SCALE
        gy = self._to_signed_16(data[2], data[3]) / self.ACCEL_SCALE
        gz = self._to_signed_16(data[4], data[5]) / self.ACCEL_SCALE
        return (gx, gy, gz)

    @property
    def temperature(self):
        """Read temperature in Celsius"""
        return self._read_byte(self.TEMP)

    def close(self):
        """Close the I2C bus"""
        self.bus.close()


class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # Declare parameters
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('i2c_addr', 0x28)
        self.declare_parameter('i2c_bus', 7)

        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        i2c_addr = self.get_parameter('i2c_addr').get_parameter_value().integer_value
        i2c_bus_num = self.get_parameter('i2c_bus').get_parameter_value().integer_value

        self.bno = None
        self._last_calib_log = 0

        # Initialize BNO055 using direct smbus2 driver
        try:
            self.bno = BNO055(i2c_bus_num, i2c_addr)
            self.bno.initialize(BNO055.NDOF_MODE)
            self.get_logger().info(f'BNO055 initialized on bus {i2c_bus_num}, addr 0x{i2c_addr:02x}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize BNO055: {str(e)}')
            self.get_logger().error('Check I2C connection and ensure device is properly powered.')
            return

        # Create publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.imu_raw_pub = self.create_publisher(Imu, 'imu/raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)

        # Create timer
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_imu_data)

        self.get_logger().info(f'IMU node started, publishing at {self.publish_rate} Hz')

    def publish_imu_data(self):
        if self.bno is None or not self.bno._initialized:
            return

        try:
            # Read sensor data
            raw_accel_x, raw_accel_y, raw_accel_z = self.bno.acceleration
            raw_gyro_x, raw_gyro_y, raw_gyro_z = self.bno.gyro
            raw_mag_x, raw_mag_y, raw_mag_z = self.bno.magnetic
            qw, qx, qy, qz = self.bno.quaternion

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

            # Log calibration status periodically
            current_time = now.nanoseconds / 1e9
            if current_time - self._last_calib_log > 5.0:
                sys, gyro, accel, mag = self.bno.calibration_status
                self.get_logger().info(f'Calibration: sys={sys}, gyro={gyro}, accel={accel}, mag={mag}')
                self._last_calib_log = current_time

            # Create and publish fused IMU message (with orientation)
            imu_msg = Imu()
            imu_msg.header.stamp = now.to_msg()
            imu_msg.header.frame_id = self.frame_id

            # Orientation from BNO055 fusion
            imu_msg.orientation.w = qw
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = -qy  # Transform Y
            imu_msg.orientation.z = qz

            # Linear acceleration (m/s²)
            imu_msg.linear_acceleration.x = accel_x
            imu_msg.linear_acceleration.y = accel_y
            imu_msg.linear_acceleration.z = accel_z

            # Angular velocity (rad/s)
            imu_msg.angular_velocity.x = gyro_x
            imu_msg.angular_velocity.y = gyro_y
            imu_msg.angular_velocity.z = gyro_z

            # Set covariance matrices
            imu_msg.orientation_covariance[0] = 0.01
            imu_msg.orientation_covariance[4] = 0.01
            imu_msg.orientation_covariance[8] = 0.01

            imu_msg.linear_acceleration_covariance[0] = 0.01
            imu_msg.linear_acceleration_covariance[4] = 0.01
            imu_msg.linear_acceleration_covariance[8] = 0.01

            imu_msg.angular_velocity_covariance[0] = 0.01
            imu_msg.angular_velocity_covariance[4] = 0.01
            imu_msg.angular_velocity_covariance[8] = 0.01

            self.imu_pub.publish(imu_msg)

            # Create and publish raw IMU message (without orientation)
            imu_raw_msg = Imu()
            imu_raw_msg.header.stamp = now.to_msg()
            imu_raw_msg.header.frame_id = self.frame_id

            imu_raw_msg.linear_acceleration.x = accel_x
            imu_raw_msg.linear_acceleration.y = accel_y
            imu_raw_msg.linear_acceleration.z = accel_z

            imu_raw_msg.angular_velocity.x = gyro_x
            imu_raw_msg.angular_velocity.y = gyro_y
            imu_raw_msg.angular_velocity.z = gyro_z

            # Orientation not provided in raw message
            imu_raw_msg.orientation_covariance[0] = -1.0

            imu_raw_msg.linear_acceleration_covariance[0] = 0.01
            imu_raw_msg.linear_acceleration_covariance[4] = 0.01
            imu_raw_msg.linear_acceleration_covariance[8] = 0.01

            imu_raw_msg.angular_velocity_covariance[0] = 0.01
            imu_raw_msg.angular_velocity_covariance[4] = 0.01
            imu_raw_msg.angular_velocity_covariance[8] = 0.01

            self.imu_raw_pub.publish(imu_raw_msg)

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
