#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from .joystick_driver import JoystickDriver

class JoystickTester(Node):
    def __init__(self):
        super().__init__('joystick_tester')
        
        self.joystick_driver = JoystickDriver(trigger_threshold=0.3)
        
        self.sub = self.create_subscription(
            Joy, 
            'joy', 
            self.on_joy, 
            10
        )
        
        self.get_logger().info("Joystick tester started - listening to /joy")
        self.get_logger().info("Press buttons and move sticks to see events...")

    def on_joy(self, msg: Joy):
        events = self.joystick_driver.process_joy_message(msg)

        # Print all events for debugging
        active_events = []
        
        # Face buttons
        for btn in ['a', 'b', 'x', 'y', 'back', 'start', 'lsb', 'rsb']:
            if events.get(btn, False):
                active_events.append(f"{btn.upper()}")
            if events.get(f'{btn}_press', False):
                active_events.append(f"{btn.upper()}_PRESS")
            if events.get(f'{btn}_release', False):
                active_events.append(f"{btn.upper()}_RELEASE")
        
        # Triggers and bumpers
        for trigger in ['rt', 'lt']:
            if events.get(trigger, False):
                value = events.get(f'{trigger}_value', 0.0)
                active_events.append(f"{trigger.upper()}({value:.2f})")
            if events.get(f'{trigger}_press', False):
                direction = events.get(f'{trigger}_direction', 1)
                active_events.append(f"{trigger.upper()}_PRESS(dir={direction})")
            if events.get(f'{trigger}_release', False):
                active_events.append(f"{trigger.upper()}_RELEASE")
        
        for bumper in ['rb', 'lb']:
            if events.get(bumper, False):
                active_events.append(f"{bumper.upper()}")
            if events.get(f'{bumper}_press', False):
                active_events.append(f"{bumper.upper()}_PRESS")
            if events.get(f'{bumper}_release', False):
                active_events.append(f"{bumper.upper()}_RELEASE")
        
        # Analog sticks
        left_stick = events.get('left_stick', {})
        ls_x = left_stick.get('x', 0.0)
        ls_y = left_stick.get('y', 0.0)
        if abs(ls_x) > 0.1 or abs(ls_y) > 0.1:
            active_events.append(f"LS({ls_x:.2f},{ls_y:.2f})")
        
        right_stick = events.get('right_stick', {})
        rs_x = right_stick.get('x', 0.0)
        rs_y = right_stick.get('y', 0.0)
        if abs(rs_x) > 0.1 or abs(rs_y) > 0.1:
            active_events.append(f"RS({rs_x:.2f},{rs_y:.2f})")
        
        if active_events:
            self.get_logger().info(f"Active: {' | '.join(active_events)}")

def main():
    rclpy.init()
    node = JoystickTester()
    
    print("\n" + "="*60)
    print("JOYSTICK TEST SCRIPT")
    print("="*60)
    print("This script will show all joystick events in real-time.")
    print("Make sure you have your Xbox controller connected and /joy topic publishing.")
    print("\nTesting:")
    print("• Face buttons: A, B, X, Y, BACK, START")
    print("• Triggers: RT/LT (now work as button presses)")
    print("• Bumpers: RB/LB")
    print("• Sticks: Left/Right (no button press required)")
    print("• Stick buttons: LSB/RSB")
    print("\nPress Ctrl+C to exit")
    print("="*60 + "\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nTest completed.")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()