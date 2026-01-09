#!/usr/bin/env python3
"""
GPIO Switch Test Script - Tests switch hardware in isolation
Run this BEFORE integrating with audio system
"""
import Jetson.GPIO as GPIO
import time

SWITCH_PIN = 22  # Physical Pin 22 (BOARD numbering)

def setup():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(SWITCH_PIN, GPIO.IN)
    print("="*50)
    print("GPIO SWITCH TEST")
    print("="*50)
    print(f"Pin: Physical Pin {SWITCH_PIN} (GPIO17)")
    print("Expected behavior:")
    print("  - Switch UP (open):   Should read HIGH (1)")
    print("  - Switch DOWN (closed): Should read LOW (0)")
    print("="*50)
    print("\nFlip the switch to test. Press Ctrl+C to exit.\n")

def main():
    setup()
    last_state = None
    
    try:
        while True:
            state = GPIO.input(SWITCH_PIN)
            
            if state != last_state:
                if state == GPIO.HIGH:
                    print(f"[{time.strftime('%H:%M:%S')}] Switch: UP (HIGH) → Would select: BLUETOOTH")
                else:
                    print(f"[{time.strftime('%H:%M:%S')}] Switch: DOWN (LOW) → Would select: PAM8403")
                last_state = state
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n\nTest complete!")
        
    finally:
        GPIO.cleanup()
        print("GPIO cleanup done.")

if __name__ == '__main__':
    main()

