#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def normalize_axis(raw):
    # Scale int16-style axes to [-1,1], otherwise clamp
    if abs(raw) > 1.5:
        return clamp(raw / 32767.0, -1.0, 1.0)
    return clamp(raw, -1.0, 1.0)

class XboxTeleop(Node):
    def __init__(self):
        super().__init__('leremix_teleop_xbox')

        # Topics / rates
        self.declare_parameter('cmd_vel_topic', '/omnidirectional_controller/cmd_vel_unstamped')
        self.declare_parameter('arm_cmd_topic', '/arm_group_position_controller/commands')

        self.declare_parameter('linear_scale', 0.6)
        self.declare_parameter('lateral_scale', 0.6)

        # Arm tuning
        self.declare_parameter('arm_increment', 0.03)          # per tick step from analog value
        self.declare_parameter('arm_button_step', 0.1)         # per tick when RB/LB pressed
        self.declare_parameter('arm_deadband', 0.25)           # larger threshold, per your ask
        self.declare_parameter('arm_rate', 50.0)

        # Axes (typical ROS2 joy_node + Xbox)
        self.axis_ls_x = 0
        self.axis_ls_y = 1
        self.axis_lt   = 2
        self.axis_rs_x = 3
        self.axis_rs_y = 4
        self.axis_rt   = 5

        # Buttons
        self.btn_a    = 0
        self.btn_b    = 1
        self.btn_x    = 2
        self.btn_y    = 3
        self.btn_lb   = 4
        self.btn_rb   = 5
        self.btn_back = 6    # camera_tilt down
        self.btn_start= 7    # camera_tilt up
        self.btn_lsb  = 9    # LS press
        self.btn_rsb  = 10   # RS press

        # Joints list MUST match controller YAML order
        self.arm_joints = ['1','2','3','4','5','6','camera_tilt']
        self.jidx = {name: i for i, name in enumerate(self.arm_joints)}

        # Load params
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.arm_cmd_topic = self.get_parameter('arm_cmd_topic').get_parameter_value().string_value
        self.linear_scale  = float(self.get_parameter('linear_scale').value)
        self.lateral_scale = float(self.get_parameter('lateral_scale').value)
        self.arm_inc       = float(self.get_parameter('arm_increment').value)
        self.arm_btn_step  = float(self.get_parameter('arm_button_step').value)
        self.arm_deadband  = float(self.get_parameter('arm_deadband').value)
        self.arm_rate      = float(self.get_parameter('arm_rate').value)

        # Publishers/subscriber
        self.pub_twist = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pub_arm   = self.create_publisher(Float64MultiArray, self.arm_cmd_topic, 10)
        self.sub       = self.create_subscription(Joy, 'joy', self.on_joy, 10)

        # State
        self.targets = [0.0] * len(self.arm_joints)
        self._last_arm_log = 0.0

        # Timer
        self.timer_arm = self.create_timer(1.0 / self.arm_rate, self.publish_arm)

        self.get_logger().info("Xbox teleop node started")
        self.get_logger().info(f"Motion topic: {self.cmd_vel_topic}, Arm topic: {self.arm_cmd_topic}")
        self.get_logger().info(f"Arm joints: {self.arm_joints} (len={len(self.arm_joints)}), deadband={self.arm_deadband}")

    # Helpers
    def _axis(self, axes, idx, invert=False):
        if idx < 0 or idx >= len(axes):
            return 0.0
        val = normalize_axis(axes[idx])
        return -val if invert else val

    def _btn(self, btns, idx):
        return (idx < len(btns)) and (btns[idx] == 1)

    def add_to_joint(self, name, delta, src=""):
        if name in self.jidx:
            i = self.jidx[name]
            self.targets[i] += delta
            self.get_logger().info(f"Joint {name} += {delta:.3f} from {src}, new={self.targets[i]:.3f}")
        else:
            self.get_logger().warn(f"Attempted to write unknown joint '{name}'")

    # Main callback
    def on_joy(self, msg: Joy):
        axes = msg.axes
        btns = msg.buttons

        # ----- Motion: face buttons -----
        vx = 0.0; vy = 0.0
        if self._btn(btns, self.btn_y):      # Y → forward
            vx =  self.linear_scale
            self.get_logger().info("Motion: Forward (Y)")
        elif self._btn(btns, self.btn_a):    # A → backward
            vx = -self.linear_scale
            self.get_logger().info("Motion: Backward (A)")
        elif self._btn(btns, self.btn_x):    # X → strafe left
            vy = -self.lateral_scale
            self.get_logger().info("Motion: Left strafe (X)")
        elif self._btn(btns, self.btn_b):    # B → strafe right
            vy =  self.lateral_scale
            self.get_logger().info("Motion: Right strafe (B)")
        else:
            # Publish explicit stop whenever no face button is pressed
            self.get_logger().info("Motion: STOP")

        tw = Twist()
        tw.linear.x = vx
        tw.linear.y = vy
        self.pub_twist.publish(tw)

        # ----- Arm joints -----
        inc = self.arm_inc
        step = self.arm_btn_step

        # LSB pressed → joints 1 & 2 via Left Stick
        if self._btn(btns, self.btn_lsb):
            ls_x = self._axis(axes, self.axis_ls_x)
            ls_y = self._axis(axes, self.axis_ls_y, invert=True)  # up=+
            if abs(ls_x) > self.arm_deadband:
                self.add_to_joint("1", ls_x * inc, "LS_x via LSB")
            if abs(ls_y) > self.arm_deadband:
                self.add_to_joint("2", ls_y * inc, "LS_y via LSB")

        # RT analog + RB button → joint 3
        rt_val = self._axis(axes, self.axis_rt)
        if abs(rt_val) > self.arm_deadband:
            self.add_to_joint("3", rt_val * inc, "RT axis")
        if self._btn(btns, self.btn_rb):
            self.add_to_joint("3", step, "RB button")

        # LT analog + LB button → joint 4
        lt_val = self._axis(axes, self.axis_lt)
        if abs(lt_val) > self.arm_deadband:
            self.add_to_joint("4", lt_val * inc, "LT axis")
        if self._btn(btns, self.btn_lb):
            self.add_to_joint("4", step, "LB button")

        # RSB pressed → joints 5 & 6 via Right Stick
        if self._btn(btns, self.btn_rsb):
            rs_x = self._axis(axes, self.axis_rs_x)
            rs_y = self._axis(axes, self.axis_rs_y, invert=True)  # up=+
            if abs(rs_x) > self.arm_deadband:
                self.add_to_joint("5", rs_x * inc, "RS_x via RSB")
            if abs(rs_y) > self.arm_deadband:
                self.add_to_joint("6", rs_y * inc, "RS_y via RSB")

        # Camera tilt: START = up (+), BACK = down (−)
        if self._btn(btns, self.btn_start):
            self.add_to_joint("camera_tilt", step, "START")
        if self._btn(btns, self.btn_back):
            self.add_to_joint("camera_tilt", -step, "BACK")

        # TEST: both START+BACK pressed -> obvious nudge to all joints (for wiring/namespace sanity)
        if self._btn(btns, self.btn_start) and self._btn(btns, self.btn_back):
            for j in self.arm_joints:
                self.add_to_joint(j, 0.05, "TEST nudge (START+BACK)")

    def publish_arm(self):
        # Safety: ensure size matches joints
        if len(self.targets) != len(self.arm_joints):
            self.get_logger().error(
                f"Arm targets size {len(self.targets)} != joints {len(self.arm_joints)} — not publishing"
            )
            return

        msg = Float64MultiArray()
        msg.data = self.targets
        self.pub_arm.publish(msg)

        # Rate-limited log of the full array (2 Hz)
        now = time.time()
        if now - self._last_arm_log > 0.5:
            self._last_arm_log = now
            self.get_logger().info(f"Publish arm -> {self.arm_cmd_topic}: {['%.3f'%v for v in self.targets]}")

def main():
    rclpy.init()
    node = XboxTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
