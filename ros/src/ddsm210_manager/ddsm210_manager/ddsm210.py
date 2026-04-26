#!/usr/bin/env python3
"""Low-level serial protocol driver for Waveshare DDSM210 motors.

Protocol: 115200 baud, 8N1, 10-byte frames, CRC-8/MAXIM checksum.

Frame format (send):
  [ID, CMD, D2, D3, D4, D5, D6, D7, D8, CRC8]

Commands:
  0x64 - Drive (velocity/position depending on mode)
  0x74 - Get mileage/position feedback
  0x75 - Query current mode
  0xA0 - Set mode (0x00=open loop, 0x02=velocity loop, 0x03=position loop)

ID set: [0xAA, 0x55, 0x53, ID, 0, 0, 0, 0, 0, CRC8] (send 5 times)

Speed: signed 16-bit, -2100..2100 (unit 0.1 RPM, so -210..210 RPM)
Position: unsigned 16-bit, 0..32767 (maps to 0..360 degrees)
"""

import struct
import time
import serial
import threading


# CRC-8/MAXIM lookup table (polynomial x^8 + x^5 + x^4 + 1 = 0x31, reflected)
_CRC8_TABLE = [
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
    0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E,
    0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0,
    0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D,
    0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5,
    0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58,
    0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6,
    0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B,
    0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F,
    0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92,
    0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C,
    0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1,
    0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49,
    0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4,
    0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A,
    0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7,
    0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
]

# Mode constants
MODE_OPEN_LOOP = 0x00
MODE_VELOCITY = 0x02
MODE_POSITION = 0x03

# Feedback content codes for Protocol 1
FEEDBACK_SPEED = 0x01
FEEDBACK_CURRENT = 0x02
FEEDBACK_POSITION = 0x03

# Speed limits (unit: 0.1 RPM)
MAX_SPEED_RAW = 2100   # 210 RPM
MIN_SPEED_RAW = -2100  # -210 RPM

# Position range
MAX_POSITION = 32767   # maps to 360 degrees

# Encoder resolution (Protocol 2 position field)
ENCODER_TICKS = 65536  # 0-65535 maps to 0-360 degrees


def _crc8_maxim(data: bytes) -> int:
    """Compute CRC-8/MAXIM over data."""
    crc = 0x00
    for b in data:
        crc = _CRC8_TABLE[crc ^ b]
    return crc


class DDSM210:
    """Low-level driver for a DDSM210 motor over UART."""

    FRAME_SIZE = 10

    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.1):
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        )
        self._lock = threading.Lock()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    # -- frame helpers --------------------------------------------------------

    def _build_frame(self, data: list[int]) -> bytes:
        """Build a 10-byte frame: 9 data bytes + CRC8."""
        assert len(data) == 9
        raw = bytes(data)
        crc = _crc8_maxim(raw)
        return raw + bytes([crc])

    def _send_receive(self, frame: bytes, retries: int = 2) -> bytes | None:
        """Send a frame and read the 10-byte response. Returns None on timeout.

        Flushes the input buffer before sending to discard stale data from
        other motors on the same bus. Retries on CRC failure or short reads.
        """
        for attempt in range(retries + 1):
            with self._lock:
                self.ser.reset_input_buffer()
                self.ser.write(frame)
                self.ser.flush()
                resp = self.ser.read(self.FRAME_SIZE)
            if len(resp) != self.FRAME_SIZE:
                time.sleep(0.005)
                continue
            if _crc8_maxim(resp[:9]) != resp[9]:
                time.sleep(0.005)
                continue
            return resp
        return None

    def _send_no_response(self, frame: bytes):
        """Send a frame without waiting for response."""
        with self._lock:
            self.ser.write(frame)
            self.ser.flush()

    # -- Protocol 1: Drive motor (0x64) ---------------------------------------

    def drive(self, motor_id: int, speed_or_pos: int,
              feedback1: int = FEEDBACK_SPEED,
              feedback2: int = FEEDBACK_POSITION,
              accel_time: int = 0,
              brake: bool = False) -> dict | None:
        """Send a drive command and return feedback.

        In velocity loop: speed_or_pos is signed 16-bit (-2100..2100), unit 0.1 RPM.
        In position loop: speed_or_pos is unsigned 16-bit (0..32767), maps to 0..360 deg.

        Args:
            motor_id: Motor ID (1-254).
            speed_or_pos: Speed (velocity mode) or position (position mode).
            feedback1: What to request in feedback slot 1 (FEEDBACK_SPEED/CURRENT/POSITION).
            feedback2: What to request in feedback slot 2.
            accel_time: Acceleration time per 1 RPM in 0.1ms units (0 = default 0.1ms).
            brake: If True, send brake command (velocity loop only).

        Returns:
            Dict with keys: feedback1, feedback2, accel_time, temperature, error_code.
            None if no response.
        """
        # Pack speed/position as signed 16-bit big-endian
        sp_hi = (speed_or_pos >> 8) & 0xFF
        sp_lo = speed_or_pos & 0xFF

        brake_byte = 0xFF if brake else 0x00

        frame = self._build_frame([
            motor_id, 0x64,
            sp_hi, sp_lo,
            feedback1, feedback2,
            accel_time, brake_byte,
            0x00,
        ])
        resp = self._send_receive(frame)
        if resp is None:
            return None

        # Parse response: [ID, 0x64, fb1_hi, fb1_lo, fb2_hi, fb2_lo, accel, temp, error, CRC]
        fb1 = struct.unpack('>h', resp[2:4])[0]  # signed 16-bit
        fb2 = struct.unpack('>h', resp[4:6])[0]  # signed 16-bit
        return {
            'feedback1': fb1,
            'feedback2': fb2,
            'accel_time': resp[6],
            'temperature': resp[7],
            'error_code': resp[8],
        }

    def set_velocity(self, motor_id: int, rpm_x10: int,
                     accel_time: int = 0) -> dict | None:
        """Set velocity in velocity loop mode.

        Args:
            motor_id: Motor ID.
            rpm_x10: Speed in 0.1 RPM units (-2100..2100 = -210..210 RPM).
            accel_time: Acceleration time per 1 RPM in 0.1ms units.

        Returns feedback dict or None.
        """
        rpm_x10 = max(MIN_SPEED_RAW, min(MAX_SPEED_RAW, rpm_x10))
        # Convert to unsigned for frame packing (struct handles sign)
        if rpm_x10 < 0:
            rpm_x10 = rpm_x10 & 0xFFFF
        return self.drive(motor_id, rpm_x10,
                          feedback1=FEEDBACK_SPEED,
                          feedback2=FEEDBACK_POSITION,
                          accel_time=accel_time)

    def brake(self, motor_id: int) -> dict | None:
        """Send brake command (velocity loop only)."""
        return self.drive(motor_id, 0, brake=True)

    def set_position(self, motor_id: int, position: int) -> dict | None:
        """Set position in position loop mode.

        Args:
            motor_id: Motor ID.
            position: 0..32767 maps to 0..360 degrees.
        """
        position = max(0, min(MAX_POSITION, position))
        return self.drive(motor_id, position,
                          feedback1=FEEDBACK_SPEED,
                          feedback2=FEEDBACK_POSITION)

    # -- Protocol 2: Get mileage/position feedback (0x74) ---------------------

    def get_odometry(self, motor_id: int) -> dict | None:
        """Get mileage laps and absolute position.

        Returns:
            Dict with keys: mileage_laps (signed 32-bit), position (0-65535 = 0-360 deg),
            error_code. None if no response.
        """
        frame = self._build_frame([motor_id, 0x74, 0, 0, 0, 0, 0, 0, 0])
        resp = self._send_receive(frame)
        if resp is None:
            return None

        # [ID, 0x74, mile_hi, mile_2hi, mile_2lo, mile_lo, pos_hi, pos_lo, error, CRC]
        mileage = struct.unpack('>i', resp[2:6])[0]  # signed 32-bit big-endian
        position = struct.unpack('>H', resp[6:8])[0]  # unsigned 16-bit
        return {
            'mileage_laps': mileage,
            'position': position,
            'error_code': resp[8],
        }

    # -- Protocol 3: Set mode (0xA0) ------------------------------------------

    def set_mode(self, motor_id: int, mode: int) -> dict | None:
        """Set motor operating mode.

        Args:
            mode: MODE_OPEN_LOOP (0x00), MODE_VELOCITY (0x02), MODE_POSITION (0x03).

        Returns response dict or None. Note: mode switch command may not return feedback.
        """
        frame = self._build_frame([motor_id, 0xA0, mode, 0, 0, 0, 0, 0, 0])
        resp = self._send_receive(frame)
        if resp is None:
            return None
        return {
            'mode': resp[2],
        }

    # -- Protocol 4: Set ID ---------------------------------------------------

    def set_motor_id(self, new_id: int):
        """Set motor ID. Only one motor must be on the bus. Sends command 5 times."""
        frame = self._build_frame([0xAA, 0x55, 0x53, new_id, 0, 0, 0, 0, 0])
        for _ in range(5):
            self._send_no_response(frame)
            time.sleep(0.05)

    # -- Protocol 5: Query mode (0x75) ----------------------------------------

    def get_mode(self, motor_id: int) -> int | None:
        """Query current motor mode. Returns mode value or None."""
        frame = self._build_frame([motor_id, 0x75, 0, 0, 0, 0, 0, 0, 0])
        resp = self._send_receive(frame)
        if resp is None:
            return None
        return resp[2]

    # -- ID query (0xC8 0x64) -------------------------------------------------

    def query_id(self) -> int | None:
        """Query motor ID on the bus. Only one motor should be connected."""
        frame = self._build_frame([0xC8, 0x64, 0, 0, 0, 0, 0, 0, 0])
        resp = self._send_receive(frame)
        if resp is None:
            return None
        return resp[0]
