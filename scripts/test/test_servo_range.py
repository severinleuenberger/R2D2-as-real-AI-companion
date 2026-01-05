
#!/usr/bin/env python3
"""
Servo Range Finder - Test full servo range to determine safe limits.

Starts at CENTER (90°), then tests UP direction, then DOWN direction.

Wiring for Pin 32 (GPIO09 - PWM capable):
  - Brown/Black (GND)  → Jetson Pin 6 (GND)
  - Red (5V Power)     → Jetson Pin 2 or Pin 4 (5V)
  - Orange (Signal)    → Jetson Pin 32 (GPIO09)

Run with: sudo python3 test_servo_range.py
"""

import time
import sys

try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False
    print("WARNING: RPi.GPIO not found - running in SIMULATION mode")

# Configuration
SERVO_PIN = 32        # Physical Pin 32 (GPIO09 - PWM capable)
PWM_FREQ = 50         # Standard servo frequency (50Hz)

# Standard servo pulse widths
MIN_DUTY = 2.5        # ~0.5ms pulse = 0 degrees
MAX_DUTY = 12.5       # ~2.5ms pulse = 180 degrees

def angle_to_duty(angle):
    """Convert angle (0-180) to duty cycle (2.5-12.5)."""
    return MIN_DUTY + (angle / 180.0) * (MAX_DUTY - MIN_DUTY)

def move_servo(pwm, angle):
    """Move servo to angle and wait."""
    if pwm:
        pwm.ChangeDutyCycle(angle_to_duty(angle))
    time.sleep(0.8)

def main():
    print("=" * 60)
    print("SERVO RANGE FINDER")
    print("=" * 60)
    print()
    print("Wiring (Pin 32 = GPIO09):")
    print("  Servo Brown/Black → Jetson Pin 6 (GND)")
    print("  Servo Red (5V)    → Jetson Pin 2 or Pin 4 (5V)")
    print("  Servo Orange      → Jetson Pin 32 (GPIO09)")
    print()
    print("Test order: CENTER → UP direction → DOWN direction")
    print()
    
    pwm = None
    
    if HAS_GPIO:
        try:
            GPIO.setmode(GPIO.BOARD)  # Use physical pin numbers
            GPIO.setwarnings(False)
            GPIO.setup(SERVO_PIN, GPIO.OUT)
            pwm = GPIO.PWM(SERVO_PIN, PWM_FREQ)
            pwm.start(angle_to_duty(90))  # Start at center
            print(f"Pin {SERVO_PIN} initialized - HARDWARE MODE")
        except Exception as e:
            print(f"GPIO init failed: {e}")
            print("Running in SIMULATION mode")
            pwm = None
    else:
        print("Running in SIMULATION mode")
    
    print()
    print("=" * 60)
    print("STEP 1: CENTER POSITION (90°)")
    print("=" * 60)
    move_servo(pwm, 90)
    print("Servo is now at 90° (center)")
    input("Press Enter when ready to test UP direction...")
    
    # =====================================================
    # TEST UP DIRECTION: 90° → 0° (in 9° steps = 5% of range)
    # =====================================================
    print()
    print("=" * 60)
    print("STEP 2: TESTING UP DIRECTION (90° → 0°)")
    print("=" * 60)
    print("Watch for mechanical limits! Press Ctrl+C to stop if needed.")
    print()
    print(f"{'Angle':>8} | {'Duty %':>8} | Status")
    print("-" * 40)
    
    # Start from 90, go to 0 in steps of 9 (5% of 180)
    try:
        for angle in range(90, -1, -9):  # 90, 81, 72, 63, 54, 45, 36, 27, 18, 9, 0
            duty = angle_to_duty(angle)
            print(f"{angle:>7}° | {duty:>7.2f}% | ", end="", flush=True)
            move_servo(pwm, angle)
            print("✓")
    except KeyboardInterrupt:
        print("\nStopped by user!")
    
    # Return to center
    print()
    print("Returning to center (90°)...")
    move_servo(pwm, 90)
    
    print()
    up_limit = input("What was the LOWEST SAFE angle for UP? (Enter number or 'skip'): ").strip()
    
    # =====================================================
    # TEST DOWN DIRECTION: 90° → 180° (in 9° steps)
    # =====================================================
    print()
    print("=" * 60)
    print("STEP 3: TESTING DOWN DIRECTION (90° → 180°)")
    print("=" * 60)
    print("Watch for mechanical limits! Press Ctrl+C to stop if needed.")
    print()
    print(f"{'Angle':>8} | {'Duty %':>8} | Status")
    print("-" * 40)
    
    try:
        for angle in range(90, 181, 9):  # 90, 99, 108, 117, 126, 135, 144, 153, 162, 171, 180
            duty = angle_to_duty(angle)
            print(f"{angle:>7}° | {duty:>7.2f}% | ", end="", flush=True)
            move_servo(pwm, angle)
            print("✓")
    except KeyboardInterrupt:
        print("\nStopped by user!")
    
    # Return to center
    print()
    print("Returning to center (90°)...")
    move_servo(pwm, 90)
    
    print()
    down_limit = input("What was the HIGHEST SAFE angle for DOWN? (Enter number or 'skip'): ").strip()
    
    # =====================================================
    # FINE-TUNING MODE
    # =====================================================
    print()
    print("=" * 60)
    print("STEP 4: FINE-TUNING (optional)")
    print("=" * 60)
    print("Enter an angle (0-180) to test precisely, or 'q' to quit")
    print()
    
    while True:
        try:
            user = input("Angle (0-180) or 'q': ").strip().lower()
            
            if user == 'q':
                break
            
            angle = float(user)
            if 0 <= angle <= 180:
                print(f"  Moving to {angle:.1f}°...")
                move_servo(pwm, angle)
            else:
                print("  Invalid! Enter 0-180")
                
        except ValueError:
            print("  Invalid input")
        except KeyboardInterrupt:
            break
    
    # Cleanup
    print()
    print("Centering servo before exit...")
    move_servo(pwm, 90)
    
    if pwm:
        pwm.stop()
    if HAS_GPIO:
        GPIO.cleanup(SERVO_PIN)
    
    # Summary
    print()
    print("=" * 60)
    print("RESULTS SUMMARY")
    print("=" * 60)
    print(f"  UP limit entered:   {up_limit}")
    print(f"  DOWN limit entered: {down_limit}")
    print()
    print("Tell me these values and I'll update the tracking code!")
    print("=" * 60)

if __name__ == '__main__':
    main()

