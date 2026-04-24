#!/usr/bin/env python3
"""
Motor State Diagnostic Tool
Checks the current mode and status of all servos
"""

import argparse
import sys
from st3215 import ST3215


def check_motor_states(port="/dev/ttyACM1"):
    """Check the mode and status of all motors"""
    
    try:
        servo = ST3215(port)
        print(f"✅ Connected to {port}")
    except Exception as e:
        print(f"❌ Failed to connect to {port}: {e}")
        return False
    
    # Motor IDs to check
    motor_ids = {
        'Locomotion': [1, 2, 3],
        'Arm': [4, 5, 6, 7, 8, 9],
        'Head': [10, 11]
    }
    
    print("\n" + "="*70)
    print("MOTOR STATUS DIAGNOSTIC")
    print("="*70)
    
    for group_name, ids in motor_ids.items():
        print(f"\n{group_name} Motors:")
        print("-" * 40)
        
        for motor_id in ids:
            print(f"\nMotor ID {motor_id}:")
            
            # Check if motor responds
            if not servo.PingServo(motor_id):
                print(f"  ❌ OFFLINE - No response")
                continue
            
            try:
                # Check position
                pos = servo.ReadPosition(motor_id)
                print(f"  Position: {pos} ticks" if pos is not None else "  Position: ❌ Read failed")
                
                # Check speed/velocity
                speed_data = servo.ReadSpeed(motor_id)
                if speed_data is not None:
                    speed = speed_data[0] if isinstance(speed_data, tuple) else speed_data
                    print(f"  Speed: {speed}")
                else:
                    print(f"  Speed: ❌ Read failed")
                
                # Try to read voltage
                try:
                    voltage = servo.ReadVoltage(motor_id)
                    print(f"  Voltage: {voltage/10:.1f}V" if voltage is not None else "  Voltage: ❌ Read failed")
                except:
                    print(f"  Voltage: ❌ Not supported")
                
                # Try to read temperature
                try:
                    temp = servo.ReadTemperature(motor_id)
                    print(f"  Temperature: {temp}°C" if temp is not None else "  Temperature: ❌ Read failed")
                except:
                    print(f"  Temperature: ❌ Not supported")
                
                print(f"  Status: ✅ ONLINE")
                
            except Exception as e:
                print(f"  ❌ Error reading status: {e}")
    
    print("\n" + "="*70)
    print("MOTOR MODE CHECK (Position vs Velocity)")
    print("="*70)
    
    # Check specific mode for problematic motor ID 4
    for motor_id in [4, 5, 6]:  # Check first few arm motors
        if servo.PingServo(motor_id):
            try:
                # Try to determine mode by testing commands
                print(f"\nTesting Motor ID {motor_id}:")
                
                # Read current position
                current_pos = servo.ReadPosition(motor_id)
                print(f"  Current position: {current_pos} ticks")
                
                # Try setting mode to position explicitly
                print(f"  Setting to position mode...")
                servo.SetMode(motor_id, 0)  # Position mode
                servo.StartServo(motor_id)
                
                # Test small movement
                if current_pos is not None:
                    test_pos = current_pos + 10  # Small 10-tick movement
                    test_pos = max(0, min(4095, test_pos))
                    
                    print(f"  Testing movement: {current_pos} -> {test_pos}")
                    servo.MoveTo(motor_id, test_pos, 100, 50)
                    
                    import time
                    time.sleep(0.5)
                    
                    new_pos = servo.ReadPosition(motor_id)
                    if new_pos is not None:
                        diff = abs(new_pos - test_pos)
                        print(f"  Result: {new_pos} ticks (error: {diff} ticks)")
                        if diff < 5:
                            print(f"  ✅ Position mode working correctly")
                        else:
                            print(f"  ⚠️  Large position error - possible mode issue")
                    
                    # Return to original position
                    servo.MoveTo(motor_id, current_pos, 100, 50)
                
            except Exception as e:
                print(f"  ❌ Mode test failed: {e}")
    
    return True


def main():
    parser = argparse.ArgumentParser(description='Motor State Diagnostic Tool')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port')
    args = parser.parse_args()
    
    print("LLMy Motor State Diagnostic")
    print("This tool checks the status and mode of all servos")
    print("Make sure no ROS nodes are running!")
    print()
    
    if check_motor_states(args.port):
        print("\nDiagnostic complete!")
    else:
        print("\nDiagnostic failed!")
        sys.exit(1)


if __name__ == '__main__':
    main()