#!/usr/bin/env python3
"""Standalone wrapper — runs the terminal without ROS.

Usage:
    python3 scripts/test_motor.py --port /dev/ttyUSB0
    python3 scripts/test_motor.py --port /dev/ttyUSB0 --port /dev/ttyUSB1

After building with colcon you can also use:
    ros2 run ddsm210_manager terminal -- --port /dev/ttyUSB0
"""

import sys
sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parent.parent))

from ddsm210_manager.terminal import main

if __name__ == "__main__":
    main()
