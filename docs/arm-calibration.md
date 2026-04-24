# Arm Calibration

This guide explains how to align the physical arm's encoder zero with the URDF's zero configuration, so the robot's internal model matches reality.

## Why this matters

The whole software stack (`st3215_manager`, `llmy_control_plugin`) hardcodes a single assumption:

> **Servo tick `2048` corresponds to `0 rad` in the URDF.**

See `st3215_manager/st3215_manager/command_handler.py:77-78` and `telemetry.py:101` — there is no per-joint offset parameter anywhere. Whatever mechanical pose the servo reports as tick 2048 **is** the URDF zero, by definition.

If the arm was assembled such that (for example) joint 2's "straight down" reads as tick 1500, then:
- `rviz` renders the joint bent ~48° when the real arm is actually straight.
- MoveIt plans in URDF frame and commands motion the real arm can't execute without self-collision.
- `command_handler.py:79` silently clamps to `[0, 4095]`, so out-of-range commands produce truncated motion instead of errors.

The fix is to burn the current physical pose as the servo's new center into EEPROM, so `ReadPosition` returns ~2048 when the arm is in the URDF-zero configuration.

## What won't break

Turning the nodes on with an uncalibrated arm does **not** cause runaway motion. Three guardrails prevent jumps at startup:

1. `motor_manager.py:102-114` reads the current tick and immediately sends `MoveTo(current_pos)` — the arm holds wherever it is.
2. `ros2_control_bridge.cpp:186-200` forces `cmd_pos_ = pos_state_` until meaningful joint states arrive — any controller command is overridden with "hold current position" during the handshake.
3. `command_handler.py:79` clamps ticks to the servo's electrical range.

The danger shows up only when something **plans** from the reported state (MoveIt, scripted trajectories, teleop). That's why calibration matters before using the arm, not before boot.

## Prerequisites

- All ROS nodes using the servo bus must be stopped (`st3215_manager`, `llmy_bringup`, etc.) — the calibration script and the manager node can't share `/dev/ttyTHS1`.
- Arm assembled and powered, but unrestrained enough that you can move each joint by hand.
- You know what URDF zero looks like for this arm. If unsure, launch `llmy_description` in rviz with all joint states at `0.0` and use that visual as your reference pose.

## Procedure

### 1. Verify the motor IDs

Open `scripts/calibrate_arm.py` and confirm the IDs match `ros/src/st3215_manager/config/llmy.yaml`:

```yaml
# config/llmy.yaml
arm:
  motor_ids: [3, 4, 5, 6, 7, 8]
```

Edit `calibrate_arm.py:36` if needed:

```python
'arm_ids': [3, 4, 5, 6, 7, 8],
```

The script ships with `[4, 5, 6, 7, 8, 9]` as a default; running with the wrong IDs will skip motor 3 and fail to talk to a nonexistent motor 9.

### 2. Shut down the servo bus

```bash
# Stop the bringup or any node opening /dev/ttyTHS1
# e.g. Ctrl-C the terminal running `ros2 launch llmy_bringup bringup_robot.launch.py`
```

### 3. Pose the arm

The script disables torque (`StopServo`) so you can move each joint freely. Pose the arm to match URDF zero — typically all links aligned with their parent frame's origin (straightened / neutral). Check against rviz if needed.

### 4. Run the live calibration

```bash
python3 scripts/calibrate_arm.py --live --port /dev/ttyTHS1
```

The live mode continuously prints each joint's current tick, radian, and degree value. When all readings look right (the arm is still in the intended pose), press **Enter**.

For each servo the script runs:

1. `UnLockEprom(id)` — make EEPROM writable.
2. `DefineMiddle(id)` — write current position as the new tick-2048 center.
3. `LockEprom(id)` — commit and protect.

Press `q` to exit when done.

### 5. Power-cycle the servos

Feetech EEPROM writes don't always take effect until the servo reboots. Power the bus off and on.

### 6. Verify

Bring the arm back up and spot-check:

```bash
ros2 launch llmy_bringup bringup_robot.launch.py
ros2 topic echo /joint_states
```

With the arm physically in the URDF-zero pose, every arm joint's `position` should be near `0.0` rad (within a few mrad is fine). `rviz` rendering the `robot_description` should now match reality.

## After calibration

Once hardware zero matches URDF zero, tighten the URDF joint limits in `llmy_description/urdf/LLMy.xacro` (`<limit lower=... upper=...>`) to match the real mechanical range. Use the full calibration flow (`python3 scripts/calibrate_arm.py` without `--live`) to capture min/max ticks for each joint — the wizard prompts through each joint individually and logs the range in degrees. Translate those into radian limits in the URDF so planners reject unreachable goals up front instead of hitting the driver clamp.

## Troubleshooting

- **`DefineMiddle` appears to succeed but ticks don't change.** The lock byte (register 55) wasn't cleared. The script handles this via `UnLockEprom`; if you're calling the library directly, unlock first.
- **Joint reads near 0 ticks or near 4095 after calibration.** The servo was already near its electrical end-stop when you pressed Enter. Back it off toward the middle of its mechanical range, re-pose, and recalibrate.
- **`received_meaningful_joint_states_` never flips true, arm ignores commands.** See `ros2_control_bridge.cpp:158-161` — the gate requires `|pos| > 0.005 rad` on at least one arm/camera joint. If every joint calibrates to *exactly* zero, nudge one joint a fraction of a degree before bringing up ros2_control, or relax the threshold.
