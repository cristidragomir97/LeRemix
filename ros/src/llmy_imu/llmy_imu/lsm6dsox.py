#!/usr/bin/env python3
"""smbus2 driver for ST LSM6DSOX 6-DOF IMU (accelerometer + gyroscope).

Default I2C address: 0x6B (SA0 high) or 0x6A (SA0 low).
"""

import math
import time
import smbus2


class LSM6DSOX:
    """Direct I2C driver for LSM6DSOX accelerometer + gyroscope."""

    # Register addresses
    WHO_AM_I = 0x0F
    CTRL1_XL = 0x10    # Accel ODR and full-scale
    CTRL2_G = 0x11     # Gyro ODR and full-scale
    CTRL3_C = 0x12     # Control register 3 (BDU, IF_INC, etc.)
    STATUS_REG = 0x1E
    OUTX_L_G = 0x22    # Gyro data start
    OUTX_L_A = 0x28    # Accel data start

    # WHO_AM_I expected value
    DEVICE_ID = 0x6C

    # Accel full-scale settings (CTRL1_XL bits [3:2])
    ACCEL_FS_2G = 0x00
    ACCEL_FS_4G = 0x08
    ACCEL_FS_8G = 0x0C
    ACCEL_FS_16G = 0x04

    # Accel sensitivity (mg/LSB)
    ACCEL_SENSITIVITY = {
        ACCEL_FS_2G: 0.061,
        ACCEL_FS_4G: 0.122,
        ACCEL_FS_8G: 0.244,
        ACCEL_FS_16G: 0.488,
    }

    # Gyro full-scale settings (CTRL2_G bits [3:1])
    GYRO_FS_250DPS = 0x00
    GYRO_FS_500DPS = 0x04
    GYRO_FS_1000DPS = 0x08
    GYRO_FS_2000DPS = 0x0C

    # Gyro sensitivity (mdps/LSB)
    GYRO_SENSITIVITY = {
        GYRO_FS_250DPS: 8.75,
        GYRO_FS_500DPS: 17.50,
        GYRO_FS_1000DPS: 35.0,
        GYRO_FS_2000DPS: 70.0,
    }

    # ODR settings (bits [7:4], same for accel and gyro)
    ODR_104HZ = 0x40
    ODR_208HZ = 0x50
    ODR_416HZ = 0x60

    def __init__(self, bus_num: int, address: int = 0x6B):
        self.bus = smbus2.SMBus(bus_num)
        self.address = address
        self._accel_fs = self.ACCEL_FS_2G
        self._gyro_fs = self.GYRO_FS_250DPS

    def initialize(self, accel_odr=None, gyro_odr=None,
                   accel_fs=None, gyro_fs=None):
        """Initialize the sensor with specified settings."""
        if accel_odr is None:
            accel_odr = self.ODR_104HZ
        if gyro_odr is None:
            gyro_odr = self.ODR_104HZ
        if accel_fs is None:
            accel_fs = self.ACCEL_FS_2G
        if gyro_fs is None:
            gyro_fs = self.GYRO_FS_250DPS

        # Verify device ID
        who = self.bus.read_byte_data(self.address, self.WHO_AM_I)
        if who != self.DEVICE_ID:
            raise RuntimeError(
                f"LSM6DSOX not found. WHO_AM_I=0x{who:02X}, expected 0x{self.DEVICE_ID:02X}")

        # Enable BDU (block data update) and auto-increment
        self.bus.write_byte_data(self.address, self.CTRL3_C, 0x44)
        time.sleep(0.01)

        # Configure accelerometer: ODR + full-scale
        self._accel_fs = accel_fs
        self.bus.write_byte_data(self.address, self.CTRL1_XL, accel_odr | accel_fs)
        time.sleep(0.01)

        # Configure gyroscope: ODR + full-scale
        self._gyro_fs = gyro_fs
        self.bus.write_byte_data(self.address, self.CTRL2_G, gyro_odr | gyro_fs)
        time.sleep(0.02)

    @staticmethod
    def _to_signed_16(lsb: int, msb: int) -> int:
        value = (msb << 8) | lsb
        if value > 32767:
            value -= 65536
        return value

    @property
    def acceleration(self) -> tuple[float, float, float]:
        """Read accelerometer in m/s²."""
        data = self.bus.read_i2c_block_data(self.address, self.OUTX_L_A, 6)
        sens = self.ACCEL_SENSITIVITY[self._accel_fs]  # mg/LSB
        ax = self._to_signed_16(data[0], data[1]) * sens * 0.001 * 9.80665
        ay = self._to_signed_16(data[2], data[3]) * sens * 0.001 * 9.80665
        az = self._to_signed_16(data[4], data[5]) * sens * 0.001 * 9.80665
        return (ax, ay, az)

    @property
    def gyro(self) -> tuple[float, float, float]:
        """Read gyroscope in rad/s."""
        data = self.bus.read_i2c_block_data(self.address, self.OUTX_L_G, 6)
        sens = self.GYRO_SENSITIVITY[self._gyro_fs]  # mdps/LSB
        gx = self._to_signed_16(data[0], data[1]) * sens * 0.001 * math.pi / 180.0
        gy = self._to_signed_16(data[2], data[3]) * sens * 0.001 * math.pi / 180.0
        gz = self._to_signed_16(data[4], data[5]) * sens * 0.001 * math.pi / 180.0
        return (gx, gy, gz)

    def close(self):
        self.bus.close()
