#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from .joystick_driver import JoystickDriver

class XboxTeleop(Node):
    def __init__(self):
        super().__init__('leremix_teleop_xbox')

        # Topics / rates
        self.declare_parameter('cmd_vel_topic', '/omnidirectional_controller/cmd_vel_unstamped')
        self.declare_parameter('arm_cmd_topic', '/arm_controller/commands')
        self.declare_parameter('linear_scale', 0.6)
        self.declare_parameter('lateral_scale', 0.6)
        self.declare_parameter('arm_increment', 0.02)
        self.declare_parameter('stick_deadband', 0.25)
        self.declare_parameter('trigger_threshold', 0.3)
        self.declare_parameter('arm_rate', 50.0)

        # Joints list includes camera_tilt
        self.arm_joints = ['1','2','3','4','5','6','camera_tilt']
        self.jidx = {name: i for i, name in enumerate(self.arm_joints)}

        # Load params
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.arm_cmd_topic = self.get_parameter('arm_cmd_topic').get_parameter_value().string_value
        self.linear_scale  = float(self.get_parameter('linear_scale').value)
        self.lateral_scale = float(self.get_parameter('lateral_scale').value)
        self.arm_inc       = float(self.get_parameter('arm_increment').value)
        self.stick_deadband = float(self.get_parameter('stick_deadband').value)
        self.trigger_threshold = float(self.get_parameter('trigger_threshold').value)
        self.arm_rate      = float(self.get_parameter('arm_rate').value)

        # Publishers/subscriber
        self.pub_twist = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pub_arm   = self.create_publisher(Float64MultiArray, self.arm_cmd_topic, 10)
        self.sub       = self.create_subscription(Joy, 'joy', self.on_joy, 10)

        # State
        self.targets = [0.0] * len(self.arm_joints)
        self.joystick_driver = JoystickDriver(self.trigger_threshold)

        # Timer
        self.timer_arm = self.create_timer(1.0 / self.arm_rate, self.publish_arm)

        self.get_logger().info("Xbox teleop node started")
        self.get_logger().info(f"Motion topic: {self.cmd_vel_topic}, Arm topic: {self.arm_cmd_topic}")
        self.get_logger().info(f"Arm joints: {self.arm_joints}, stick deadband={self.stick_deadband}")

    def add_to_joint(self, name, delta, src=""):
        if name in self.jidx:
            i = self.jidx[name]
            self.targets[i] += delta
            self.get_logger().info(f"Joint {name} += {delta:.3f} from {src}, new={self.targets[i]:.3f}")

    def on_joy(self, msg: Joy):
        events = self.joystick_driver.process_joy_message(msg)

        # ----- Motion: face buttons -----
        vx = 0.0; vy = 0.0
        if events.get('y', False):
            vx = self.linear_scale
            self.get_logger().info("Motion: Forward (Y)")
        elif events.get('a', False):
            vx = -self.linear_scale
            self.get_logger().info("Motion: Backward (A)")
        elif events.get('x', False):
            vy = -self.lateral_scale
            self.get_logger().info("Motion: Left strafe (X)")
        elif events.get('b', False):
            vy = self.lateral_scale
            self.get_logger().info("Motion: Right strafe (B)")
        else:
            self.get_logger().info("Motion: STOP")

        tw = Twist()
        tw.linear.x = vx
        tw.linear.y = vy
        self.pub_twist.publish(tw)

        # ----- Arm joints -----
        inc = self.arm_inc

        # Left Stick → joints 1 & 2 (no button press required)
        left_stick = events.get('left_stick', {})
        ls_x = left_stick.get('x', 0.0)
        ls_y = left_stick.get('y', 0.0)
        if abs(ls_x) > self.stick_deadband:
            self.add_to_joint("1", ls_x * inc, "LS_x")
        if abs(ls_y) > self.stick_deadband:
            self.add_to_joint("2", ls_y * inc, "LS_y")

        # RT/RB button presses → joint 3
        if events.get('rt_press', False):
            direction = events.get('rt_direction', 1)
            self.add_to_joint("3", direction * inc, "RT press")
        if events.get('rb_press', False):
            self.add_to_joint("3", inc, "RB press")

        # LT/LB button presses → joint 4
        if events.get('lt_press', False):
            direction = events.get('lt_direction', 1)
            self.add_to_joint("4", direction * inc, "LT press")
        if events.get('lb_press', False):
            self.add_to_joint("4", inc, "LB press")

        # Right Stick → joints 5 & 6 (no button press required)
        right_stick = events.get('right_stick', {})
        rs_x = right_stick.get('x', 0.0)
        rs_y = right_stick.get('y', 0.0)
        if abs(rs_x) > self.stick_deadband:
            self.add_to_joint("5", rs_x * inc, "RS_x")
        if abs(rs_y) > self.stick_deadband:
            self.add_to_joint("6", rs_y * inc, "RS_y")

        # Camera tilt: START = up (+), BACK = down (−)
        if events.get('start_press', False):
            self.add_to_joint("camera_tilt", inc, "START press")
        if events.get('back_press', False):
            self.add_to_joint("camera_tilt", -inc, "BACK press")

    def publish_arm(self):
        msg = Float64MultiArray()
        msg.data = self.targets
        self.pub_arm.publish(msg)

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
