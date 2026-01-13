#!/usr/bin/env python3
"""Interactive menu for controlling ST3215 servos."""

from st3215 import ST3215

SERIAL_PORT = '/dev/ttyTHS1'


def scan_servos(servo):
    print("\nScanning for servos...")
    ids = servo.ListServos()
    if not ids:
        print("No servos found.")
        return []
    print(f"Found {len(ids)} servo(s): {ids}")
    return ids


def move_servo(servo, servo_id, position, speed=None, relative=False):
    if relative:
        current = servo.ReadPosition(servo_id)
        if current is None:
            print(f"Could not read position for servo {servo_id}")
            return
        position = current + position
        print(f"Current: {current}, moving by {'+' if position >= current else ''}{position - current}")

    position = max(0, min(4095, position))  # Clamp to valid range

    if speed:
        servo.MoveTo(servo_id, position, speed)
    else:
        servo.MoveTo(servo_id, position)
    print(f"Moving servo {servo_id} to position {position}")


def read_position(servo, servo_id):
    pos = servo.ReadPosition(servo_id)
    print(f"Servo {servo_id} position: {pos}")


def change_id(servo, old_id, new_id, ids):
    if new_id in ids and new_id != old_id:
        print(f"ID {new_id} is already in use.")
        return ids
    servo.ChangeId(old_id, new_id)
    print(f"Changed ID from {old_id} to {new_id}")
    if old_id in ids:
        ids.remove(old_id)
    ids.append(new_id)
    return ids


def parse_command(cmd, servo, ids):
    """Parse and execute a command. Returns updated ids list."""
    parts = cmd.lower().split()
    if not parts:
        return ids, False

    # scan
    if parts[0] == 'scan':
        return scan_servos(servo), True

    # move <id> <pos> [speed]  OR  move <id> inc/dec <amount> [speed]
    if parts[0] == 'move' and len(parts) >= 3:
        try:
            servo_id = int(parts[1])
            # Check for inc/dec keywords
            if parts[2] in ('inc', 'dec') and len(parts) >= 4:
                amount = int(parts[3])
                if parts[2] == 'dec':
                    amount = -amount
                speed = int(parts[4]) if len(parts) > 4 else None
                move_servo(servo, servo_id, amount, speed, relative=True)
            else:
                pos = int(parts[2])
                speed = int(parts[3]) if len(parts) > 3 else None
                move_servo(servo, servo_id, pos, speed, relative=False)
            return ids, True
        except ValueError:
            print("Usage: move <id> <position> [speed]  OR  move <id> inc/dec <amount> [speed]")
            return ids, True

    # read <id>
    if parts[0] == 'read' and len(parts) >= 2:
        try:
            servo_id = int(parts[1])
            read_position(servo, servo_id)
            return ids, True
        except ValueError:
            print("Usage: read <id>")
            return ids, True

    # change <old_id> to <new_id>  OR  change <old_id> <new_id>
    if parts[0] == 'change' and len(parts) >= 3:
        try:
            old_id = int(parts[1])
            # Support both "change 1 to 3" and "change 1 3"
            if parts[2] == 'to' and len(parts) >= 4:
                new_id = int(parts[3])
            else:
                new_id = int(parts[2])
            ids = change_id(servo, old_id, new_id, ids)
            return ids, True
        except ValueError:
            print("Usage: change <old_id> to <new_id>")
            return ids, True

    # list
    if parts[0] == 'list':
        if ids:
            print(f"Known servos: {ids}")
        else:
            print("No servos known. Run 'scan' first.")
        return ids, True

    # help
    if parts[0] == 'help':
        print_help()
        return ids, True

    return ids, False


def print_help():
    print("""
Commands:
  scan                        Scan for connected servos
  list                        List known servo IDs
  move <id> <pos> [speed]     Move servo to position (0-4095)
  move <id> inc <N> [speed]   Increment position by N
  move <id> dec <N> [speed]   Decrement position by N
  read <id>                   Read servo position
  change <id> to <new_id>     Change servo ID
  help                        Show this help
  quit / q                    Exit
""")


def print_menu():
    print("\n--- Servo Control ---")
    print("Type a command or 'help' for options")


def main():
    servo = ST3215(SERIAL_PORT)
    ids = []

    print("Servo Control - type 'help' for commands")

    while True:
        try:
            cmd = input("\n> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nBye!")
            break

        if not cmd:
            continue

        if cmd.lower() in ('quit', 'q', 'exit'):
            print("Bye!")
            break

        ids, handled = parse_command(cmd, servo, ids)
        if not handled:
            print(f"Unknown command: {cmd}")
            print("Type 'help' for available commands.")


if __name__ == "__main__":
    main()