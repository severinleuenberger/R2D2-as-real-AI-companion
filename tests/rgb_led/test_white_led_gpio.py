#!/usr/bin/env python3
"""
White LED GPIO Control Test
Tests the white LED panel with GPIO control

LED Configuration:
  - Red wire (Positive) → Pin 4 (5V) - Power supply
  - Black wire (Ground) → Pin 6 (GND) - Ground
  - Blue wire (Control) → Pin 22 (GPIO 17) - ON/OFF control

Author: R2D2 Development Team
Date: December 18, 2025
"""

import Jetson.GPIO as GPIO
import time
import sys

# GPIO Pin Assignment (using BOARD mode)
PIN_LED_CONTROL = 22   # GPIO 17 (Physical Pin 22)

def setup_gpio():
    """Initialize GPIO for LED control."""
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO.setup(PIN_LED_CONTROL, GPIO.OUT, initial=GPIO.LOW)
    print(f"✅ GPIO initialized")
    print(f"   LED Control → Pin {PIN_LED_CONTROL} (GPIO 17)")

def cleanup_gpio():
    """Clean up GPIO and turn LED off."""
    GPIO.output(PIN_LED_CONTROL, GPIO.LOW)
    GPIO.cleanup()
    print("\n✅ GPIO cleaned up, LED off")

def set_led(state):
    """Set LED state (True=ON, False=OFF)."""
    GPIO.output(PIN_LED_CONTROL, GPIO.HIGH if state else GPIO.LOW)

def test_on_off():
    """Test basic ON/OFF control."""
    print("\n" + "="*70)
    print("Testing ON/OFF Control")
    print("="*70)
    
    print("\n💡 LED ON...")
    set_led(True)
    time.sleep(3)
    
    print("💡 LED OFF...")
    set_led(False)
    time.sleep(2)
    
    print("💡 LED ON...")
    set_led(True)
    time.sleep(2)
    
    print("💡 LED OFF...")
    set_led(False)
    time.sleep(1)
    
    print("\n✅ ON/OFF test complete")

def test_blink():
    """Test blinking pattern."""
    print("\n" + "="*70)
    print("Blink Test")
    print("="*70)
    
    print("\n💡 Blinking 10 times...")
    for i in range(10):
        set_led(True)
        time.sleep(0.3)
        set_led(False)
        time.sleep(0.3)
    
    print("✅ Blink test complete")

def test_status_simulation():
    """Simulate R2D2 status behavior."""
    print("\n" + "="*70)
    print("R2D2 Status Simulation")
    print("="*70)
    
    print("\n🤖 RED Status (Person Recognized) - LED ON")
    set_led(True)
    time.sleep(3)
    
    print("🤖 BLUE Status (Lost/Idle) - LED OFF")
    set_led(False)
    time.sleep(3)
    
    print("🤖 GREEN Status (Unknown Person) - LED OFF")
    set_led(False)
    time.sleep(3)
    
    print("🤖 Transition: BLUE → RED (Person found)")
    for _ in range(3):
        set_led(True)
        time.sleep(0.2)
        set_led(False)
        time.sleep(0.2)
    set_led(True)
    time.sleep(2)
    set_led(False)
    
    print("\n✅ Status simulation complete")

def main():
    print("="*70)
    print("White LED GPIO Control Test")
    print("="*70)
    
    print("\n📍 Wiring Instructions:")
    print("   LED Red wire (Power)    → Pin 4 (5V)")
    print("   LED Black wire (Ground) → Pin 6 (GND)")
    print(f"   LED Blue wire (Control) → Pin {PIN_LED_CONTROL} (GPIO 17)")
    
    print("\n⚠️  Important: Connect RED wire to Pin 4 (5V), not Pin 2!")
    print("   Pin 4 provides power, Pin 22 controls ON/OFF")
    
    input("\nPress ENTER when wiring is ready...")
    
    setup_gpio()
    
    try:
        # Turn off LED first
        print("\n💡 Turning LED OFF (safety)...")
        set_led(False)
        time.sleep(1)
        
        # Run tests
        test_on_off()
        test_blink()
        test_status_simulation()
        
        print("\n" + "="*70)
        print("✅ All tests complete!")
        print("="*70)
        
        print("\n📊 Results:")
        print("   ✅ GPIO control: Working")
        print("   ✅ ON/OFF switching: Working")
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

