#!/usr/bin/env python3
"""
Servo Hardware Test Script

Tests the tilt servo without the full ROS 2 stack.
Use this to verify your servo wiring is correct before running the full tracking node.

Wiring:
  - Brown/Black (GND)  → Jetson Pin 6
  - Red (5V)          → Jetson Pin 2 or Pin 4  
  - Orange (Signal)   → Jetson Pin 33 (GPIO13)

Usage:
  python3 test_servo_hardware.py [--gpio PIN] [--min ANGLE] [--max ANGLE]

Examples:
  python3 test_servo_hardware.py                    # Default: GPIO13, 60-120 degrees
  python3 test_servo_hardware.py --gpio 18          # Use GPIO18 instead
  python3 test_servo_hardware.py --min 45 --max 135 # Different angle range
"""

import argparse
import time
import sys

# Add the r2d2_head_control package to path for standalone testing
sys.path.insert(0, '/home/severin/dev/r2d2/ros2_ws/src/r2d2_head_control')

from r2d2_head_control.servo_driver import ServoDriver


def main():
    parser = argparse.ArgumentParser(description='Test servo hardware')
    parser.add_argument('--gpio', type=int, default=13, help='GPIO pin (BCM number)')
    parser.add_argument('--min', type=float, default=60.0, help='Minimum angle (degrees)')
    parser.add_argument('--max', type=float, default=120.0, help='Maximum angle (degrees)')
    parser.add_argument('--simulate', action='store_true', help='Run in simulation mode')
    args = parser.parse_args()
    
    print("=" * 60)
    print("Servo Hardware Test")
    print("=" * 60)
    print(f"GPIO Pin: {args.gpio} (Physical Pin 33 if using GPIO13)")
    print(f"Angle Range: {args.min}° - {args.max}°")
    print(f"Neutral: {(args.min + args.max) / 2}°")
    print()
    print("Expected wiring (for GPIO13):")
    print("  Servo Brown/Black  → Jetson Pin 6 (GND)")
    print("  Servo Red (5V)     → Jetson Pin 2 or 4 (5V)")
    print("  Servo Orange       → Jetson Pin 33 (GPIO13)")
    print()
    
    # Create servo driver
    servo = ServoDriver(
        gpio_pin=args.gpio,
        min_angle=args.min,
        max_angle=args.max,
        initial_angle=(args.min + args.max) / 2,
        simulate=args.simulate
    )
    
    if not servo.initialize():
        print("ERROR: Failed to initialize servo!")
        print("Make sure you're running with appropriate permissions (sudo if needed)")
        return 1
    
    print(f"Servo initialized. Mode: {'SIMULATION' if servo.simulate else 'HARDWARE'}")
    print()
    
    try:
        # Test 1: Move to neutral
        print("Test 1: Moving to neutral position...")
        servo.center()
        print(f"  Current angle: {servo.get_angle():.1f}°")
        time.sleep(1)
        
        # Test 2: Sweep test
        print()
        print("Test 2: Sweep test (slow)")
        input("Press Enter to start sweep test...")
        servo.sweep_test(steps=6, delay=0.5)
        
        # Test 3: Quick movements
        print()
        print("Test 3: Quick movement test")
        input("Press Enter to test quick movements...")
        test_angles = [args.min, args.max, (args.min + args.max) / 2]
        for angle in test_angles:
            print(f"  Moving to {angle}°...")
            servo.set_angle(angle)
            time.sleep(0.5)
        
        # Test 4: Manual control
        print()
        print("Test 4: Manual angle control")
        print("Enter an angle value to move servo, or 'q' to quit")
        while True:
            try:
                user_input = input(f"Angle ({args.min}-{args.max}): ").strip()
                if user_input.lower() == 'q':
                    break
                angle = float(user_input)
                servo.set_angle(angle)
                print(f"  Moved to: {servo.get_angle():.1f}°")
            except ValueError:
                print("  Invalid input. Enter a number or 'q' to quit.")
        
        # Return to neutral before exiting
        print()
        print("Returning to neutral position...")
        servo.center()
        time.sleep(0.5)
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        servo.cleanup()
    
    print()
    print("Test complete!")
    return 0


if __name__ == '__main__':
    sys.exit(main())

