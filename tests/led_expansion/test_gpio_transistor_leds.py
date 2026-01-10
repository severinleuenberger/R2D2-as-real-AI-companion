#!/usr/bin/env python3
"""
GPIO Transistor LED Test Script

Tests 4 LEDs connected via NPN transistors:
  - Pin 7  -> RED LED
  - Pin 11 -> BLUE LED
  - Pin 12 -> GREEN LED
  - Pin 13 -> YELLOW LED

Run with: sudo python3 test_gpio_transistor_leds.py
"""

import time
import Jetson.GPIO as GPIO

# Pin assignments (BOARD numbering)
LED_PINS = {
    'RED': 7,
    'BLUE': 11,
    'GREEN': 12,
    'YELLOW': 13
}

def setup():
    """Initialize GPIO pins"""
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    
    for name, pin in LED_PINS.items():
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        print(f"  Configured Pin {pin} for {name} LED")

def led_on(name):
    """Turn on a specific LED"""
    pin = LED_PINS.get(name.upper())
    if pin:
        GPIO.output(pin, GPIO.HIGH)

def led_off(name):
    """Turn off a specific LED"""
    pin = LED_PINS.get(name.upper())
    if pin:
        GPIO.output(pin, GPIO.LOW)

def all_off():
    """Turn off all LEDs"""
    for pin in LED_PINS.values():
        GPIO.output(pin, GPIO.LOW)

def all_on():
    """Turn on all LEDs"""
    for pin in LED_PINS.values():
        GPIO.output(pin, GPIO.HIGH)

def test_individual_leds():
    """Test each LED one by one"""
    print("\n" + "=" * 50)
    print("TEST 1: Individual LED Test")
    print("=" * 50)
    
    for name, pin in LED_PINS.items():
        print(f"\n  Turning ON: {name} LED (Pin {pin})")
        led_on(name)
        input(f"  >>> Is the {name} LED lit? Press Enter to continue...")
        led_off(name)
    
    print("\n  ✓ Individual LED test complete!")

def test_all_leds():
    """Test all LEDs together"""
    print("\n" + "=" * 50)
    print("TEST 2: All LEDs Test")
    print("=" * 50)
    
    print("\n  Turning ON all LEDs...")
    all_on()
    input("  >>> Are ALL 4 LEDs lit? Press Enter to continue...")
    
    print("  Turning OFF all LEDs...")
    all_off()
    print("\n  ✓ All LEDs test complete!")

def test_sequence():
    """Run a sequence pattern"""
    print("\n" + "=" * 50)
    print("TEST 3: Sequence Pattern (5 cycles)")
    print("=" * 50)
    
    print("\n  Running LED sequence...")
    for cycle in range(5):
        for name in ['RED', 'BLUE', 'GREEN', 'YELLOW']:
            led_on(name)
            time.sleep(0.2)
            led_off(name)
    
    print("  ✓ Sequence test complete!")

def test_blink():
    """Blink all LEDs"""
    print("\n" + "=" * 50)
    print("TEST 4: Blink All LEDs (5 times)")
    print("=" * 50)
    
    print("\n  Blinking all LEDs...")
    for i in range(5):
        all_on()
        time.sleep(0.3)
        all_off()
        time.sleep(0.3)
    
    print("  ✓ Blink test complete!")

def test_status_patterns():
    """Test patterns that will be used for status indication"""
    print("\n" + "=" * 50)
    print("TEST 5: Status Patterns Preview")
    print("=" * 50)
    
    patterns = [
        ("STARTUP", ['RED', 'BLUE', 'GREEN', 'YELLOW'], "All LEDs on"),
        ("OK/RUNNING", ['GREEN'], "Only GREEN on"),
        ("WARNING", ['YELLOW'], "Only YELLOW on"),
        ("ERROR", ['RED'], "Only RED on"),
        ("BUSY", ['BLUE'], "Only BLUE on"),
    ]
    
    for status, leds, description in patterns:
        print(f"\n  {status}: {description}")
        all_off()
        for led in leds:
            led_on(led)
        input("  >>> Press Enter to continue...")
    
    all_off()
    print("\n  ✓ Status patterns test complete!")

def main():
    print("=" * 50)
    print("GPIO Transistor LED Test")
    print("=" * 50)
    print("\nPin Configuration:")
    for name, pin in LED_PINS.items():
        print(f"  Pin {pin:2d} -> {name} LED")
    
    print("\nInitializing GPIO...")
    setup()
    
    try:
        test_individual_leds()
        test_all_leds()
        test_sequence()
        test_blink()
        test_status_patterns()
        
        print("\n" + "=" * 50)
        print("ALL TESTS COMPLETE!")
        print("=" * 50)
        print("\nIf all LEDs worked correctly, the wiring is good!")
        print("You can now use the GPIO LED status node.")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted!")
    finally:
        all_off()
        GPIO.cleanup()
        print("\nGPIO cleaned up.")

if __name__ == "__main__":
    main()

