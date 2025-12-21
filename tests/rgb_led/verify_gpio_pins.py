#!/usr/bin/env python3
"""
Verify Jetson AGX Orin GPIO Pins for WS2812B Use

Purpose:
  Test which physical pins are available and working on Jetson AGX Orin
  Verify Pin 15 (GPIO27) is safe for WS2812B data signal

Usage:
  python3 verify_gpio_pins.py

Output:
  List of available GPIO pins and their status
"""

import Jetson.GPIO as GPIO
import sys

# Test pins (recommended for WS2812B)
TEST_PINS = {
    15: "GPIO27 (RECOMMENDED for WS2812B data)",
    22: "GPIO17 (currently used for white LED)",
    32: "GPIO09 (alternative)",
    33: "GPIO (alternative)"
}

def verify_jetson_gpio():
    """Verify Jetson.GPIO library and board info"""
    print("=" * 70)
    print("Jetson GPIO Verification")
    print("=" * 70)
    
    try:
        print(f"\n✅ Jetson.GPIO Library Version: {GPIO.VERSION}")
        print(f"✅ Board Model: {GPIO.model}")
        print(f"✅ Board Information: {GPIO.JETSON_INFO}")
    except Exception as e:
        print(f"❌ Error getting board info: {e}")
        return False
    
    return True

def test_gpio_pins():
    """Test each GPIO pin for availability"""
    print("\n" + "=" * 70)
    print("Testing GPIO Pins")
    print("=" * 70)
    
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    
    results = []
    
    for pin, description in TEST_PINS.items():
        print(f"\nTesting Pin {pin}: {description}")
        try:
            # Setup pin as output
            GPIO.setup(pin, GPIO.OUT)
            
            # Test HIGH
            GPIO.output(pin, GPIO.HIGH)
            print(f"  ✅ Set HIGH successfully")
            
            # Test LOW
            GPIO.output(pin, GPIO.LOW)
            print(f"  ✅ Set LOW successfully")
            
            # Cleanup
            GPIO.cleanup(pin)
            
            results.append((pin, "AVAILABLE", description))
            print(f"  ✅ Pin {pin} is AVAILABLE and working")
            
        except Exception as e:
            results.append((pin, "ERROR", str(e)))
            print(f"  ❌ Pin {pin} ERROR: {e}")
    
    return results

def print_summary(results):
    """Print test summary"""
    print("\n" + "=" * 70)
    print("Test Summary")
    print("=" * 70)
    
    available_pins = [pin for pin, status, _ in results if status == "AVAILABLE"]
    error_pins = [pin for pin, status, _ in results if status == "ERROR"]
    
    print(f"\n✅ Available Pins: {len(available_pins)}")
    for pin, status, desc in results:
        if status == "AVAILABLE":
            print(f"   - Pin {pin}: {desc}")
    
    if error_pins:
        print(f"\n❌ Unavailable Pins: {len(error_pins)}")
        for pin, status, desc in results:
            if status == "ERROR":
                print(f"   - Pin {pin}: {desc}")
    
    print("\n" + "=" * 70)
    print("Recommendation")
    print("=" * 70)
    
    if 15 in available_pins:
        print("✅ Pin 15 (GPIO27) is AVAILABLE - RECOMMENDED for WS2812B data signal")
        print("   This pin is PWM-capable and safe for addressable LEDs")
    elif 32 in available_pins:
        print("⚠️  Pin 15 not available, but Pin 32 (GPIO09) is available")
        print("   Use Pin 32 as alternative for WS2812B data signal")
    elif 33 in available_pins:
        print("⚠️  Pin 15 not available, but Pin 33 is available")
        print("   Use Pin 33 as alternative for WS2812B data signal")
    else:
        print("❌ No recommended pins available!")
        print("   Check for GPIO conflicts or permissions issues")

def main():
    try:
        # Verify Jetson.GPIO library
        if not verify_jetson_gpio():
            print("\n❌ Jetson.GPIO library verification failed!")
            sys.exit(1)
        
        # Test GPIO pins
        results = test_gpio_pins()
        
        # Print summary
        print_summary(results)
        
        print("\n" + "=" * 70)
        print("Next Steps")
        print("=" * 70)
        print("1. Note which pin is recommended (Pin 15 preferred)")
        print("2. Connect RGB LED data wire to recommended pin (via 330Ω resistor)")
        print("3. Run test_02_identify_data_pin.py to test data signal")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            GPIO.cleanup()
        except:
            pass

if __name__ == '__main__':
    main()

