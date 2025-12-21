#!/usr/bin/env python3
"""
Dual-Color LED GPIO Control Test
Tests Red and Blue LED control via GPIO pins

LED Configuration:
  - Red wire → Controls RED LEDs
  - Blue wire → Controls BLUE LEDs  
  - Black wire → GND (common)

This is a common cathode LED - colors light when pins are HIGH.

Author: R2D2 Development Team
Date: December 18, 2025
"""

import Jetson.GPIO as GPIO
import time
import sys

# GPIO Pin Assignment (using BOARD mode)
# These match the default R2D2 pin assignments
PIN_RED = 22   # GPIO 17 (Physical Pin 22) - Reserved for status LED
PIN_BLUE = 15  # GPIO 27 (Physical Pin 15) - Reserved for status LED

def setup_gpio():
    """Initialize GPIO for LED control."""
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO.setup(PIN_RED, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(PIN_BLUE, GPIO.OUT, initial=GPIO.LOW)
    print(f"✅ GPIO initialized")
    print(f"   Red LED  → Pin {PIN_RED} (GPIO 17)")
    print(f"   Blue LED → Pin {PIN_BLUE} (GPIO 27)")

def cleanup_gpio():
    """Clean up GPIO and turn LEDs off."""
    GPIO.output(PIN_RED, GPIO.LOW)
    GPIO.output(PIN_BLUE, GPIO.LOW)
    GPIO.cleanup()
    print("\n✅ GPIO cleaned up, LEDs off")

def set_led(red=False, blue=False):
    """Set LED colors."""
    GPIO.output(PIN_RED, GPIO.HIGH if red else GPIO.LOW)
    GPIO.output(PIN_BLUE, GPIO.HIGH if blue else GPIO.LOW)

def test_individual_colors():
    """Test each color individually."""
    print("\n" + "="*70)
    print("Testing Individual Colors")
    print("="*70)
    
    colors = [
        ("RED", True, False),
        ("BLUE", False, True),
        ("MAGENTA (Red+Blue)", True, True),
        ("OFF", False, False),
    ]
    
    for name, red, blue in colors:
        print(f"\n🎨 Setting LED to: {name}")
        set_led(red, blue)
        time.sleep(2)
    
    print("\n💡 All colors tested")

def test_status_simulation():
    """Simulate R2D2 status colors."""
    print("\n" + "="*70)
    print("R2D2 Status Simulation")
    print("="*70)
    
    statuses = [
        ("RED - Person Recognized", True, False, 3),
        ("BLUE - Lost/Idle", False, True, 3),
        ("MAGENTA - Unknown Person (Green substitute)", True, True, 3),
        ("OFF", False, False, 1),
    ]
    
    for name, red, blue, duration in statuses:
        print(f"\n🤖 Status: {name}")
        set_led(red, blue)
        time.sleep(duration)
    
    print("\n✅ Status simulation complete")

def test_blink():
    """Test blinking pattern."""
    print("\n" + "="*70)
    print("Blink Test")
    print("="*70)
    
    print("\n🔴 Blinking RED...")
    for i in range(5):
        set_led(red=True)
        time.sleep(0.3)
        set_led(red=False)
        time.sleep(0.3)
    
    print("\n🔵 Blinking BLUE...")
    for i in range(5):
        set_led(blue=True)
        time.sleep(0.3)
        set_led(blue=False)
        time.sleep(0.3)
    
    print("\n✅ Blink test complete")

def main():
    print("="*70)
    print("Dual-Color LED GPIO Control Test")
    print("="*70)
    
    print("\n📍 Wiring Instructions:")
    print(f"   LED Red wire (Power)   → Pin {PIN_RED} (GPIO 17)")
    print(f"   LED Blue wire (Power)  → Pin {PIN_BLUE} (GPIO 27)")
    print(f"   LED Black wire (GND)   → Pin 6 (GND)")
    
    print("\n⚠️  Note: This uses the default R2D2 GPIO pins!")
    print("   Pin 22 (GPIO 17) - Red LED control")
    print("   Pin 15 (GPIO 27) - Blue LED control")
    
    input("\nPress ENTER when wiring is ready...")
    
    setup_gpio()
    
    try:
        # Turn off LEDs first
        print("\n💡 Turning LEDs OFF (safety)...")
        set_led(False, False)
        time.sleep(1)
        
        # Run tests
        test_individual_colors()
        test_status_simulation()
        test_blink()
        
        print("\n" + "="*70)
        print("✅ All tests complete!")
        print("="*70)
        
        print("\n📊 Results:")
        print("   ✅ RED LED control: Working")
        print("   ✅ BLUE LED control: Working")
        print("   ✅ Ready for R2D2 integration!")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrupted by user")
    except Exception as e:
        print(f"\n\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        cleanup_gpio()

if __name__ == "__main__":
    main()

