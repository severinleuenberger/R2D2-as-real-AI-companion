#!/usr/bin/env python3
"""
Test 1.2: Identify Data Pin for RGB LED Panel
Jetson AGX Orin - Corrected GPIO Mappings

Purpose:
  Determine which wire (blue/white/green) is the WS2812B data signal
  Method: Slow GPIO toggling to see which wire causes LED response

Wiring:
  - Red wire: 5V (Jetson Pin 2 or 4)
  - Black wire: GND (Jetson Pin 6)
  - Test wire: Connect ONE wire at a time to test pin (via 330Ω resistor)

Usage:
  1. Connect power (red→5V, black→GND)
  2. Connect ONE test wire to Pin 15 (via 330Ω resistor)
  3. Run: python3 test_02_identify_data_pin.py
  4. Watch LED panel for color changes or patterns
  5. If no change, disconnect and try next wire

Safety:
  - Use 330Ω resistor between test wire and GPIO pin
  - Test one wire at a time
  - LED panel should show steady red with power/GND only
"""

import Jetson.GPIO as GPIO
import time
import sys

# Use BOARD mode (physical pin numbers)
GPIO.setmode(GPIO.BOARD)

# Test with Pin 15 (GPIO27) - SAFE and PWM-capable
TEST_PIN = 15

# Test configuration
TOGGLE_INTERVAL = 0.5  # 0.5 seconds (slow, safe)
TEST_DURATION = 10     # 10 seconds total

def setup_gpio():
    """Initialize GPIO pin"""
    GPIO.setwarnings(False)
    GPIO.setup(TEST_PIN, GPIO.OUT)
    GPIO.output(TEST_PIN, GPIO.LOW)
    print(f"✅ Physical Pin {TEST_PIN} initialized (LOW)")

def test_data_pin():
    """Slowly toggle GPIO to test if this is the data pin"""
    print(f"\n🔄 Testing Physical Pin {TEST_PIN} as data signal...")
    print(f"   Toggling every {TOGGLE_INTERVAL}s for {TEST_DURATION}s")
    print(f"   👀 Watch LED panel for ANY changes!\n")
    
    start_time = time.time()
    toggle_count = 0
    
    try:
        while (time.time() - start_time) < TEST_DURATION:
            # Toggle HIGH
            GPIO.output(TEST_PIN, GPIO.HIGH)
            print(f"  [{toggle_count*2:2d}] GPIO HIGH (3.3V) - LED should change if this is data pin")
            time.sleep(TOGGLE_INTERVAL)
            
            # Toggle LOW
            GPIO.output(TEST_PIN, GPIO.LOW)
            print(f"  [{toggle_count*2+1:2d}] GPIO LOW  (0.0V) - LED should change if this is data pin")
            time.sleep(TOGGLE_INTERVAL)
            
            toggle_count += 1
            
    except KeyboardInterrupt:
        print("\n⚠️  Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Error during test: {e}")
    finally:
        GPIO.output(TEST_PIN, GPIO.LOW)
        GPIO.cleanup()
        print(f"\n✅ GPIO {TEST_PIN} cleaned up (LOW)")

def main():
    print("=" * 70)
    print("RGB LED Panel - Test 1.2: Data Pin Identification")
    print("Jetson AGX Orin - Corrected GPIO Mappings")
    print("=" * 70)
    
    print("\n📋 Required Wiring:")
    print("  1. Red wire   → Jetson Pin 2 or 4 (5V)")
    print("  2. Black wire → Jetson Pin 6 (GND)")
    print(f"  3. Test wire  → 330Ω resistor → Jetson Pin {TEST_PIN}")
    
    print("\n⚠️  IMPORTANT:")
    print("  - Use 330Ω resistor between test wire and GPIO pin!")
    print("  - Test ONE wire at a time (blue, white, or green)")
    print("  - LED should show steady red glow before test starts")
    print("  - Watch for ANY color change during test")
    
    print("\n📊 What to Look For:")
    print("  ✅ Data pin:     LED changes colors, turns off, or shows patterns")
    print("  ❌ Not data pin: LED stays steady red (no change)")
    
    print("=" * 70)
    
    # Confirm wiring
    response = input("\n🔌 Is wiring connected correctly? (yes/no): ").strip().lower()
    if response != 'yes':
        print("⚠️  Please connect wiring and run script again")
        sys.exit(0)
    
    print("\n⏱️  Starting test in 3 seconds...")
    time.sleep(1)
    print("⏱️  2...")
    time.sleep(1)
    print("⏱️  1...")
    time.sleep(1)
    print("▶️  Starting test NOW!\n")
    
    setup_gpio()
    test_data_pin()
    
    print("\n" + "=" * 70)
    print("📊 Test Results")
    print("=" * 70)
    
    print("\n🤔 Did the LED panel change colors or show ANY patterns during the test?")
    print("\n  Option 1: YES, LED changed → This wire IS the DATA pin! ✅")
    print("            → Proceed to Test 2.1 (Library Installation)")
    print("\n  Option 2: NO, LED stayed red → This wire is NOT data pin ❌")
    print("            → Disconnect this wire")
    print("            → Connect NEXT wire (blue/white/green)")
    print("            → Run this script again")
    
    print("\n💡 Tips:")
    print("  - WS2812B data pin will cause visible LED changes")
    print("  - Non-data wires will show no change at all")
    print("  - Try all three wires systematically")
    
    print("\n" + "=" * 70)

if __name__ == '__main__':
    main()
