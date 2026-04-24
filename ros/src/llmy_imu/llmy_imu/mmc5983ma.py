#!/usr/bin/env python3
"""smbus2 driver for MEMSIC MMC5983MA 3-axis magnetometer.

Default I2C address: 0x30.
Output range: ±8 Gauss (±800 µT), 18-bit resolution.
"""

import time
import smbus2


class MMC5983MA:
    """Direct I2C driver for MMC5983MA magnetometer."""

    # Register addresses
    XOUT_0 = 0x00       # X data [17:10]
    XOUT_1 = 0x01       # X data [9:2]
    YOUT_0 = 0x02       # Y data [17:10]
    YOUT_1 = 0x03       # Y data [9:2]
    ZOUT_0 = 0x04       # Z data [17:10]
    ZOUT_1 = 0x05       # Z data [9:2]
    XYZOUT_2 = 0x06     # X[1:0], Y[1:0], Z[1:0] (bits 7-2)
    TOUT = 0x07          # Temperature output
    STATUS = 0x08        # Status register
    CTRL0 = 0x09         # Control register 0
    CTRL1 = 0x0A         # Control register 1
    CTRL2 = 0x0B         # Control register 2
    PRODUCT_ID = 0x2F    # Product ID register

    # Expected product ID
    DEVICE_ID = 0x30

    # Control register 0 bits
    TM_M = 0x01          # Take measurement (magnetic)
    TM_T = 0x02          # Take measurement (temperature)
    SET = 0x08           # SET operation
    RESET = 0x10         # RESET operation

    # Control register 2 bits
    CMM_EN = 0x08        # Continuous measurement mode enable
    CM_FREQ_100HZ = 0x05 # 100 Hz continuous mode

    # Status register bits
    MEAS_M_DONE = 0x01   # Magnetic measurement done

    # Scale: 18-bit output, range ±8 Gauss = ±800 µT
    # Midpoint at 2^17 = 131072, full range 2^18 = 262144
    # 1 LSB = 1600 µT / 262144 = 0.006103515625 µT
    COUNTS_PER_UT = 262144.0 / 1600.0  # ~163.84 counts/µT
    OFFSET = 131072  # midpoint for unsigned 18-bit

    def __init__(self, bus_num: int, address: int = 0x30):
        self.bus = smbus2.SMBus(bus_num)
        self.address = address

    def initialize(self):
        """Initialize the magnetometer."""
        # Verify product ID
        pid = self.bus.read_byte_data(self.address, self.PRODUCT_ID)
        if pid != self.DEVICE_ID:
            raise RuntimeError(
                f"MMC5983MA not found. Product ID=0x{pid:02X}, expected 0x{self.DEVICE_ID:02X}")

        # Software reset
        self.bus.write_byte_data(self.address, self.CTRL1, 0x80)
        time.sleep(0.02)

        # Perform SET/RESET to remove offset
        self.bus.write_byte_data(self.address, self.CTRL0, self.SET)
        time.sleep(0.001)
        self.bus.write_byte_data(self.address, self.CTRL0, self.RESET)
        time.sleep(0.001)

        # Enable continuous measurement at 100Hz
        self.bus.write_byte_data(self.address, self.CTRL2,
                                 self.CMM_EN | self.CM_FREQ_100HZ)
        time.sleep(0.01)

    @property
    def magnetic(self) -> tuple[float, float, float]:
        """Read magnetometer data in µT."""
        # Read 7 bytes: X[17:10], X[9:2], Y[17:10], Y[9:2], Z[17:10], Z[9:2], XYZ[1:0]
        data = self.bus.read_i2c_block_data(self.address, self.XOUT_0, 7)

        # Reconstruct 18-bit unsigned values
        x = (data[0] << 10) | (data[1] << 2) | ((data[6] >> 6) & 0x03)
        y = (data[2] << 10) | (data[3] << 2) | ((data[6] >> 4) & 0x03)
        z = (data[4] << 10) | (data[5] << 2) | ((data[6] >> 2) & 0x03)

        # Convert to µT (subtract offset, divide by scale)
        mx = (x - self.OFFSET) / self.COUNTS_PER_UT
        my = (y - self.OFFSET) / self.COUNTS_PER_UT
        mz = (z - self.OFFSET) / self.COUNTS_PER_UT

        return (mx, my, mz)

    def close(self):
        self.bus.close()
