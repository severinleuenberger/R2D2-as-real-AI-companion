#!/usr/bin/env python3
"""
Quick Single LED Test

Use this to quickly test one LED at a time.
Usage: sudo python3 test_single_led.py [red|blue|green|yellow|all]

Examples:
  sudo python3 test_single_led.py red      # Test RED LED only
  sudo python3 test_single_led.py blue     # Test BLUE LED only
  sudo python3 test_single_led.py all      # Test all LEDs
"""

import sys
import time
import Jetson.GPIO as GPIO

# Pin assignments
LEDS = {
    'red': 7,
    'blue': 11,
    'green': 12,
    'yellow': 13
}

def main():
    # Parse argument
    if len(sys.argv) < 2:
        print("Usage: sudo python3 test_single_led.py [red|blue|green|yellow|all]")
        print("\nAvailable LEDs:")
        for name, pin in LEDS.items():
            print(f"  {name:8s} -> Pin {pin}")
        sys.exit(1)
    
    led_name = sys.argv[1].lower()
    
    # Setup GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    
    try:
        if led_name == 'all':
            # Test all LEDs
            print("Testing ALL LEDs...")
            for name, pin in LEDS.items():
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.HIGH)
                print(f"  {name.upper()} LED (Pin {pin}) -> ON")
            
            print("\nAll LEDs should be ON now.")
            print("Press Ctrl+C to turn off and exit.")
            
            while True:
                time.sleep(1)
                
        elif led_name in LEDS:
            # Test specific LED
            pin = LEDS[led_name]
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.HIGH)
            
            print(f"\n{led_name.upper()} LED (Pin {pin}) is now ON")
            print("Press Ctrl+C to turn off and exit.")
            
            while True:
                time.sleep(1)
        else:
            print(f"Unknown LED: {led_name}")
            print("Available: red, blue, green, yellow, all")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n\nTurning off...")
    finally:
        GPIO.cleanup()
        print("Done.")

if __name__ == "__main__":
    main()

