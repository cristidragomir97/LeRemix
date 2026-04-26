#!/usr/bin/env python3
"""Interactive terminal for DDSM210 motors — no ROS required.

Usage:
    ros2 run ddsm210_manager terminal -- --port /dev/ttyUSB0
    ros2 run ddsm210_manager terminal -- --port /dev/ttyUSB0 --port /dev/ttyUSB1
"""

import argparse
import cmd
import sys

from .ddsm210 import (
    DDSM210, MODE_VELOCITY, MODE_POSITION, MODE_OPEN_LOOP,
)

MODE_NAMES = {0x00: "open_loop", 0x02: "velocity", 0x03: "position"}
MODE_VALUES = {"open_loop": MODE_OPEN_LOOP, "velocity": MODE_VELOCITY, "position": MODE_POSITION}


class DDSM210Shell(cmd.Cmd):
    intro = (
        "\n"
        "  DDSM210 Motor Terminal\n"
        "  Type 'help' for commands, 'quit' to exit.\n"
        "  All motors are braked on exit.\n"
    )
    prompt = "ddsm210> "

    def __init__(self, drivers: dict[str, DDSM210], ports: list[str]):
        super().__init__()
        self.drivers = drivers          # port -> DDSM210
        self.ports = ports              # ordered list of ports
        self.motor_port: dict[int, str] = {}   # motor_id -> port (learned from scan/ping)
        self.active_motors: set[int] = set()

    # -- helpers --------------------------------------------------------------

    def _parse_id(self, arg: str) -> int | None:
        try:
            return int(arg)
        except ValueError:
            print(f"  Invalid motor ID: {arg}")
            return None

    def _get_driver(self, motor_id: int) -> DDSM210 | None:
        """Get the driver for a motor. If not yet known, try all ports."""
        if motor_id in self.motor_port:
            return self.drivers[self.motor_port[motor_id]]

        # Try each port to find this motor
        for port, driver in self.drivers.items():
            mode = driver.get_mode(motor_id)
            if mode is not None:
                self.motor_port[motor_id] = port
                return driver

        print(f"  Motor {motor_id}: not found on any port")
        return None

    def _brake_all(self):
        for mid in list(self.active_motors):
            driver = self._get_driver(mid)
            if driver:
                try:
                    driver.brake(mid)
                except Exception:
                    pass
        self.active_motors.clear()

    # -- commands -------------------------------------------------------------

    def do_ports(self, _arg):
        """ports    List connected serial ports."""
        for i, port in enumerate(self.ports):
            print(f"  [{i}] {port}")
        if self.motor_port:
            print("  Known motor assignments:")
            for mid, port in sorted(self.motor_port.items()):
                print(f"    Motor {mid} -> {port}")

    def do_ping(self, arg):
        """ping <id>    Check if motor is online (searches all ports)."""
        mid = self._parse_id(arg)
        if mid is None:
            return
        for port, driver in self.drivers.items():
            mode = driver.get_mode(mid)
            if mode is not None:
                self.motor_port[mid] = port
                print(f"  Motor {mid}: ONLINE on {port} "
                      f"(mode={MODE_NAMES.get(mode, hex(mode))})")
                return
        print(f"  Motor {mid}: no response on any port")

    def do_scan(self, arg):
        """scan [max_id]    Scan all ports for motors (default: IDs 1-10)."""
        max_id = int(arg) if arg.strip() else 10
        found = []
        for port, driver in self.drivers.items():
            for mid in range(1, max_id + 1):
                mode = driver.get_mode(mid)
                if mode is not None:
                    self.motor_port[mid] = port
                    found.append((mid, port))
                    print(f"  Motor {mid}: ONLINE on {port} "
                          f"(mode={MODE_NAMES.get(mode, hex(mode))})")
        if not found:
            print("  No motors found.")
        else:
            print(f"  Found {len(found)} motor(s)")

    def do_mode(self, arg):
        """mode <id> <velocity|position|open_loop>    Set motor operating mode."""
        parts = arg.split()
        if len(parts) != 2:
            print("  Usage: mode <id> <velocity|position|open_loop>")
            return
        mid = self._parse_id(parts[0])
        if mid is None:
            return
        mode_str = parts[1].lower()
        if mode_str not in MODE_VALUES:
            print(f"  Invalid mode: {mode_str}. Use: velocity, position, open_loop")
            return
        driver = self._get_driver(mid)
        if not driver:
            return
        resp = driver.set_mode(mid, MODE_VALUES[mode_str])
        if resp:
            print(f"  Motor {mid}: mode set to {mode_str}")
        else:
            print(f"  Motor {mid}: no response (mode switch may not reply)")

    def do_speed(self, arg):
        """speed <id> <rpm>    Set velocity in RPM (-210 to 210). Use 0 to stop."""
        parts = arg.split()
        if len(parts) != 2:
            print("  Usage: speed <id> <rpm>")
            return
        mid = self._parse_id(parts[0])
        if mid is None:
            return
        try:
            rpm = float(parts[1])
        except ValueError:
            print(f"  Invalid RPM: {parts[1]}")
            return
        driver = self._get_driver(mid)
        if not driver:
            return
        rpm_x10 = int(rpm * 10)
        fb = driver.set_velocity(mid, rpm_x10)
        self.active_motors.add(mid)
        if fb:
            print(f"  Motor {mid}: speed={rpm:.1f} RPM | "
                  f"fb_speed={fb['feedback1']} | "
                  f"temp={fb['temperature']}C | "
                  f"error=0x{fb['error_code']:02X}")
        else:
            print(f"  Motor {mid}: no response")

    def do_pos(self, arg):
        """pos <id> <degrees>    Set position in degrees (0-360). Requires position mode."""
        parts = arg.split()
        if len(parts) != 2:
            print("  Usage: pos <id> <degrees>")
            return
        mid = self._parse_id(parts[0])
        if mid is None:
            return
        try:
            deg = float(parts[1])
        except ValueError:
            print(f"  Invalid degrees: {parts[1]}")
            return
        driver = self._get_driver(mid)
        if not driver:
            return
        position = int((deg % 360.0) / 360.0 * 32767)
        fb = driver.set_position(mid, position)
        if fb:
            print(f"  Motor {mid}: target={deg:.1f} deg ({position}) | "
                  f"temp={fb['temperature']}C | "
                  f"error=0x{fb['error_code']:02X}")
        else:
            print(f"  Motor {mid}: no response")

    def do_brake(self, arg):
        """brake <id>    Brake motor (velocity mode only). Use 'brake all' for all."""
        if arg.strip().lower() == "all":
            self._brake_all()
            print("  All motors braked.")
            return
        mid = self._parse_id(arg)
        if mid is None:
            return
        driver = self._get_driver(mid)
        if not driver:
            return
        driver.brake(mid)
        self.active_motors.discard(mid)
        print(f"  Motor {mid}: braked")

    def do_odom(self, arg):
        """odom <id>    Read odometry (mileage laps + absolute position)."""
        mid = self._parse_id(arg)
        if mid is None:
            return
        driver = self._get_driver(mid)
        if not driver:
            return
        odom = driver.get_odometry(mid)
        if odom:
            deg = (odom['position'] / 65536.0) * 360.0
            print(f"  Motor {mid}:")
            print(f"    Mileage laps: {odom['mileage_laps']}")
            print(f"    Position:     {odom['position']} ({deg:.1f} deg)")
            print(f"    Error code:   0x{odom['error_code']:02X}")
        else:
            print(f"  Motor {mid}: no response")

    def do_set_id(self, arg):
        """set_id <new_id>    Set motor ID (ONE motor on bus only, sends 5x)."""
        mid = self._parse_id(arg)
        if mid is None:
            return
        if len(self.drivers) > 1:
            print("  Which port? Use: set_id <new_id> <port_index>")
            print("  Run 'ports' to see port indices.")
            return
        confirm = input(f"  Set motor ID to {mid}? Only 1 motor must be on the bus. [y/N] ")
        if confirm.lower() != 'y':
            print("  Cancelled.")
            return
        driver = list(self.drivers.values())[0]
        driver.set_motor_id(mid)
        print(f"  ID set command sent 5x. Power cycle the motor to verify.")

    def do_query_id(self, _arg):
        """query_id    Query the ID of the motor on each port (one motor per port only)."""
        for port, driver in self.drivers.items():
            mid = driver.query_id()
            if mid is not None:
                print(f"  {port}: Motor ID = {mid}")
                self.motor_port[mid] = port
            else:
                print(f"  {port}: no response")

    def do_quit(self, _arg):
        """Exit the terminal."""
        self._brake_all()
        print("  Braked all motors. Goodbye.")
        return True

    do_exit = do_quit
    do_q = do_quit

    def do_EOF(self, _arg):
        print()
        return self.do_quit("")

    def emptyline(self):
        pass


def main():
    parser = argparse.ArgumentParser(description="DDSM210 interactive terminal")
    parser.add_argument("--port", action="append", dest="ports",
                        help="Serial port(s) — specify once per port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    args = parser.parse_args()

    ports = args.ports or ["/dev/ttyUSB0"]

    drivers: dict[str, DDSM210] = {}
    for port in ports:
        print(f"  Connecting to {port} @ {args.baud}...")
        try:
            drivers[port] = DDSM210(port, args.baud)
        except Exception as e:
            print(f"  ERROR: Could not open {port}: {e}")
            sys.exit(1)

    try:
        shell = DDSM210Shell(drivers, ports)
        shell.cmdloop()
    except KeyboardInterrupt:
        print("\n  Interrupted — braking all motors...")
        shell._brake_all()
    finally:
        for driver in drivers.values():
            driver.close()


if __name__ == "__main__":
    main()
