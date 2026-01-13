#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float64MultiArray
from .joystick_driver import JoystickDriver

class XboxTeleop(Node):
    def __init__(self):
        super().__init__('llmy_teleop_xbox')

        # Topics / rates
        self.declare_parameter('cmd_vel_topic', '/diff_drive_controller/cmd_vel')
        self.declare_parameter('arm_cmd_topic', '/arm_controller/commands')
        self.declare_parameter('head_cmd_topic', '/head_controller/commands')
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('lateral_scale', 1.0)
        self.declare_parameter('angular_scale', 1.0)
        self.declare_parameter('arm_increment', 0.01745)  # 10.0 degrees
        self.declare_parameter('arm_increment_fast', 0.1396)  # 8.0 degrees for joints 1-4
        self.declare_parameter('stick_deadband', 0.1)
        self.declare_parameter('trigger_threshold', 0.3)
        self.declare_parameter('arm_rate', 50.0)      # Reduced from 30 to 20 Hz
        self.declare_parameter('head_rate', 50.0)     # Separate head rate
        self.declare_parameter('base_rate', 50.0)
        self.declare_parameter('acceleration_limit', 5.0)

        # Separate arm and head joints with limits
        self.arm_joints = ['1','2','3','4','5','6']
        self.head_joints = ['pan', 'tilt']
        self.jidx_arm = {name: i for i, name in enumerate(self.arm_joints)}
        self.jidx_head = {name: i for i, name in enumerate(self.head_joints)}

        self.head_limits = {
            'pan': (-3.14, 3.14),    # full rotation
            'tilt': (-1.57, 0.86)     # tilt limits
        }

        # Load params
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.arm_cmd_topic = self.get_parameter('arm_cmd_topic').get_parameter_value().string_value
        self.head_cmd_topic = self.get_parameter('head_cmd_topic').get_parameter_value().string_value
        self.linear_scale  = float(self.get_parameter('linear_scale').value)
        self.lateral_scale = float(self.get_parameter('lateral_scale').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)
        self.arm_inc       = float(self.get_parameter('arm_increment').value)
        self.arm_inc_fast  = float(self.get_parameter('arm_increment_fast').value)
        self.stick_deadband = float(self.get_parameter('stick_deadband').value)
        self.trigger_threshold = float(self.get_parameter('trigger_threshold').value)
        self.arm_rate      = float(self.get_parameter('arm_rate').value)
        self.head_rate     = float(self.get_parameter('head_rate').value)
        self.base_rate     = float(self.get_parameter('base_rate').value)
        self.acceleration_limit = float(self.get_parameter('acceleration_limit').value)

        # Publishers/subscriber
        # Use BEST_EFFORT QoS to match controller expectations
        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pub_twist = self.create_publisher(TwistStamped, self.cmd_vel_topic, cmd_vel_qos)
        self.pub_arm   = self.create_publisher(Float64MultiArray, self.arm_cmd_topic, 10)
        self.pub_head  = self.create_publisher(Float64MultiArray, self.head_cmd_topic, 10)
        self.sub       = self.create_subscription(Joy, 'joy', self.on_joy, 10)
        
        # Subscribe to joint states to initialize current positions
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.on_joint_state, 10)

        # State - will be initialized from joint states
        self.arm_targets = [0.0] * len(self.arm_joints)
        self.head_targets = [0.0] * len(self.head_joints)
        self.joint_states_received = False
        self.initialization_complete = False
        self.joystick_driver = JoystickDriver(self.trigger_threshold)
        
        # Smooth movement state
        self.current_vel_angular = 0.0
        self.current_vel_linear = 0.0
        self.current_vel_lateral = 0.0
        self.target_vel_angular = 0.0
        self.target_vel_linear = 0.0
        self.target_vel_lateral = 0.0
        self.dt = 1.0 / self.base_rate

        # Timer
        self.timer_arm = self.create_timer(1.0 / self.arm_rate, self.publish_arm)
        self.timer_head = self.create_timer(1.0 / self.head_rate, self.publish_head)
        self.timer_base = self.create_timer(1.0 / self.base_rate, self.publish_base)

        self.get_logger().info("Xbox teleop node started")
        self.get_logger().info(f"Motion topic: {self.cmd_vel_topic}, Arm topic: {self.arm_cmd_topic}, Head topic: {self.head_cmd_topic}")
        self.get_logger().info(f"Publishing rates: Arm={self.arm_rate}Hz, Head={self.head_rate}Hz, Base={self.base_rate}Hz")
        self.get_logger().info(f"Arm joints: {self.arm_joints}, Head joints: {self.head_joints}, stick deadband={self.stick_deadband}")
        self.get_logger().info("🎮 Controls: Right stick=forward/rotate, Left stick X=strafe, Buttons=arm joints")
        self.get_logger().info("⏳ Waiting for joint states to initialize current positions...")

    def on_joint_state(self, msg: JointState):
        """Initialize arm and head targets from current joint states"""
        if self.initialization_complete:
            return  # Only initialize once
            
        # Map joint state names to our target arrays
        for i, joint_name in enumerate(msg.name):
            if i < len(msg.position):
                position = msg.position[i]
                
                # Update arm targets
                if joint_name in self.jidx_arm:
                    arm_idx = self.jidx_arm[joint_name]
                    self.arm_targets[arm_idx] = position
                    self.get_logger().info(f"Initialized arm joint {joint_name}: {position:.3f} rad")
                
                # Update head targets  
                elif joint_name in self.jidx_head:
                    head_idx = self.jidx_head[joint_name]
                    # Apply head joint limits
                    min_pos, max_pos = self.head_limits[joint_name]
                    clamped_pos = max(min_pos, min(max_pos, position))
                    self.head_targets[head_idx] = clamped_pos
                    self.get_logger().info(f"Initialized head joint {joint_name}: {clamped_pos:.3f} rad (from {position:.3f})")
        
        # Check if we have all required joints
        arm_joints_found = sum(1 for name in msg.name if name in self.jidx_arm)
        head_joints_found = sum(1 for name in msg.name if name in self.jidx_head)
        
        if arm_joints_found >= len(self.arm_joints) and head_joints_found >= len(self.head_joints):
            self.initialization_complete = True
            self.joint_states_received = True
            self.get_logger().info(f"✅ Initialization complete! Arm: {self.arm_targets}, Head: {self.head_targets}")
            self.get_logger().info("🎮 Ready for Xbox controller input!")
        elif not self.joint_states_received:
            self.joint_states_received = True
            self.get_logger().info(f"📊 Partial joint states received: {arm_joints_found}/{len(self.arm_joints)} arm, {head_joints_found}/{len(self.head_joints)} head")

    def add_to_arm_joint(self, name, delta, src=""):
        if name in self.jidx_arm:
            i = self.jidx_arm[name]
            self.arm_targets[i] += delta
            self.get_logger().info(f"Arm joint {name} += {delta:.3f} from {src}, target={self.arm_targets[i]:.3f}")
    
    def add_to_head_joint(self, name, delta, src=""):
        if name in self.jidx_head:
            i = self.jidx_head[name]
            # Apply joint limits
            min_pos, max_pos = self.head_limits[name]
            new_pos = self.head_targets[i] + delta
            self.head_targets[i] = max(min_pos, min(max_pos, new_pos))
            self.get_logger().info(f"Head joint {name} += {delta:.3f} from {src}, clamped={self.head_targets[i]:.3f}")

    def on_joy(self, msg: Joy):
        # Don't process joystick inputs until we've initialized from joint states
        if not self.initialization_complete:
            return
            
        events = self.joystick_driver.process_joy_message(msg)

        # ----- Motion: joysticks -----
        # Right stick: forward/backward and rotation
        right_stick = events.get('right_stick', {})
        rs_x = right_stick.get('x', 0.0)  # Forward/backward
        rs_y = right_stick.get('y', 0.0)  # Left/right rotation

        # Left stick X: lateral movement (strafe)
        left_stick = events.get('left_stick', {})
        ls_x = left_stick.get('x', 0.0)  # Left/right strafe

        # Apply deadband and scaling for forward/backward
        # Inverted because joystick forward is negative
        if abs(rs_y) > self.stick_deadband:
            self.target_vel_linear = -rs_y * self.linear_scale
        else:
            self.target_vel_linear = 0.0

        # Apply deadband and scaling for rotation
        if abs(rs_x) > self.stick_deadband:
            self.target_vel_angular = rs_x * self.angular_scale
        else:
            self.target_vel_angular = 0.0
            
        # Apply deadband and scaling for lateral movement (strafe)
        if abs(ls_x) > self.stick_deadband:
            self.target_vel_lateral = ls_x * self.lateral_scale
        else:
            self.target_vel_lateral = 0.0

        # ----- Arm joints: buttons only (left stick now used for lateral movement) -----
        inc = self.arm_inc
        inc_fast = self.arm_inc_fast

        # Bumpers → joints 1 & 2 (instead of left stick for smoother arm control)
        if events.get('rb_press', False):
            self.add_to_arm_joint("1", inc, "RB press (joint 1)")
        elif events.get('rb_long_press_active', False):
            self.add_to_arm_joint("1", inc, "RB long press (joint 1)")
            
        if events.get('lb_press', False):
            self.add_to_arm_joint("1", -inc, "LB press (joint 1)")
        elif events.get('lb_long_press_active', False):
            self.add_to_arm_joint("1", -inc, "LB long press (joint 1)")
            
        # Triggers → joint 2
        if events.get('rt_press', False):
            self.add_to_arm_joint("2", inc, "RT press (joint 2)")
        if events.get('lt_press', False):
            self.add_to_arm_joint("2", -inc, "LT press (joint 2)")

        # Face buttons → joints 3 & 4 (single press for small increment, long press for continuous)
        if events.get('y_press', False):
            self.add_to_arm_joint("3", inc_fast, "Y press")
        elif events.get('y_long_press_active', False):
            self.add_to_arm_joint("3", inc_fast, "Y long press")
            
        if events.get('a_press', False):
            self.add_to_arm_joint("3", -inc_fast, "A press")
        elif events.get('a_long_press_active', False):
            self.add_to_arm_joint("3", -inc_fast, "A long press")
            
        if events.get('x_press', False):
            self.add_to_arm_joint("4", -inc_fast, "X press")
        elif events.get('x_long_press_active', False):
            self.add_to_arm_joint("4", -inc_fast, "X long press")
            
        if events.get('b_press', False):
            self.add_to_arm_joint("4", inc_fast, "B press")
        elif events.get('b_long_press_active', False):
            self.add_to_arm_joint("4", inc_fast, "B long press")

        # Face buttons control joints 5 & 6 (moved from shoulders/triggers)
        # Start/Back buttons → joint 5 
        if events.get('start_press', False):
            self.add_to_arm_joint("5", inc, "Start press (joint 5)")
        if events.get('back_press', False):
            self.add_to_arm_joint("5", -inc, "Back press (joint 5)")
            
        # Left/Right stick clicks → joint 6
        if events.get('right_stick_press', False):
            self.add_to_arm_joint("6", inc, "Right stick click (joint 6)")
        if events.get('left_stick_press', False):
            self.add_to_arm_joint("6", -inc, "Left stick click (joint 6)")

        # D-pad for camera control
        # D-pad left/right for camera tilt
        if events.get('dpad_left_press', False):
            self.add_to_head_joint("tilt", -inc, "D-pad left press")
        elif events.get('dpad_left_long_press_active', False):
            self.add_to_head_joint("tilt", -inc, "D-pad left long press")

        if events.get('dpad_right_press', False):
            self.add_to_head_joint("tilt", inc, "D-pad right press")
        elif events.get('dpad_right_long_press_active', False):
            self.add_to_head_joint("tilt", inc, "D-pad right long press")

        # D-pad up/down for camera pan
        if events.get('dpad_up_press', False):
            self.add_to_head_joint("pan", inc, "D-pad up press")
        elif events.get('dpad_up_long_press_active', False):
            self.add_to_head_joint("pan", inc, "D-pad up long press")

        if events.get('dpad_down_press', False):
            self.add_to_head_joint("pan", -inc, "D-pad down press")
        elif events.get('dpad_down_long_press_active', False):
            self.add_to_head_joint("pan", -inc, "D-pad down long press")

    def publish_arm(self):
        # Only publish arm commands after initialization
        if not self.initialization_complete:
            return
            
        msg = Float64MultiArray()
        msg.data = self.arm_targets
        self.pub_arm.publish(msg)
    
    def publish_head(self):
        # Only publish head commands after initialization
        if not self.initialization_complete:
            return
            
        msg = Float64MultiArray()
        msg.data = self.head_targets
        self.pub_head.publish(msg)

    def publish_base(self):
        # Only publish base commands after initialization
        if not self.initialization_complete:
            return

        # Smooth acceleration/deceleration
        max_delta = self.acceleration_limit * self.dt
        
        # Angular velocity
        vel_diff_angular = self.target_vel_angular - self.current_vel_angular
        if abs(vel_diff_angular) > max_delta:
            vel_diff_angular = max_delta if vel_diff_angular > 0 else -max_delta
        self.current_vel_angular += vel_diff_angular
        
        # Linear velocity (forward/backward)
        vel_diff_linear = self.target_vel_linear - self.current_vel_linear
        if abs(vel_diff_linear) > max_delta:
            vel_diff_linear = max_delta if vel_diff_linear > 0 else -max_delta
        self.current_vel_linear += vel_diff_linear
        
        # Lateral velocity (strafe left/right)
        vel_diff_lateral = self.target_vel_lateral - self.current_vel_lateral
        if abs(vel_diff_lateral) > max_delta:
            vel_diff_lateral = max_delta if vel_diff_lateral > 0 else -max_delta
        self.current_vel_lateral += vel_diff_lateral
        
        # Publish smoothed twist
        tw = TwistStamped()
        tw.header.stamp = self.get_clock().now().to_msg()
        tw.header.frame_id = 'base_link'
        tw.twist.linear.x = self.current_vel_linear  # Forward/backward
        tw.twist.linear.y = self.current_vel_lateral  # Left/right strafe (ignored by diff_drive)
        tw.twist.angular.z = self.current_vel_angular  # Rotation
        self.pub_twist.publish(tw)

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
