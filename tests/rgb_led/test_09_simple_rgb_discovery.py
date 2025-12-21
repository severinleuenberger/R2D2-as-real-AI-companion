#!/usr/bin/env python3
"""
Simple RGB LED Discovery Test
Tests if the LED panel is a simple RGB LED with separate color channels

Your finding: Red wire (5V) + Blue wire (3.3V) = Red + Blue LEDs lit
This suggests: NOT WS2812B, but simple RGB with 3 control lines!

Possible configurations:
1. Common Anode (Red/Green/Blue = GND to activate, Power = 5V/3.3V)
2. Common Cathode (Red/Green/Blue = +V to activate, GND = common)
3. Three independent channels

Author: R2D2 Development Team  
Date: December 18, 2025
"""

import Jetson.GPIO as GPIO
import time
import sys

# Test GPIO pins (using BOARD numbering)
TEST_PINS = {
    'Pin 15': 15,  # GPIO27
    'Pin 16': 16,  # GPIO08
    'Pin 18': 18,  # SPI1_MOSI
    'Pin 22': 22,  # GPIO17
}

def setup_gpio():
    """Initialize GPIO."""
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

def cleanup_gpio():
    """Clean up GPIO."""
    GPIO.cleanup()

def test_pin_as_output(pin_num, pin_name):
    """Test a pin as HIGH/LOW output."""
    try:
        GPIO.setup(pin_num, GPIO.OUT)
        
        print(f"\n  Testing {pin_name} (Physical Pin {pin_num}):")
        
        # Test HIGH
        print(f"    Setting HIGH (3.3V)...", end="", flush=True)
        GPIO.output(pin_num, GPIO.HIGH)
        time.sleep(2)
        print(" done")
        
        # Test LOW
        print(f"    Setting LOW (GND)...", end="", flush=True)
        GPIO.output(pin_num, GPIO.LOW)
        time.sleep(2)
        print(" done")
        
        return True
    except Exception as e:
        print(f"    ❌ Error: {e}")
        return False

def main():
    print("="*70)
    print("Simple RGB LED Discovery Test")
    print("="*70)
    
    print("\n🔍 Your Discovery:")
    print("   Red wire (5V) + Blue wire on Pin 17 (3.3V) = Red + Blue LEDs lit!")
    print("\n💡 This suggests: Simple RGB LED with 3 separate color channels")
    print("   NOT addressable WS2812B!")
    
    print("\n" + "="*70)
    print("Test Plan")
    print("="*70)
    
    print("\nWe'll test different wiring configurations:")
    print("\n📍 Configuration 1: Common Anode (typical)")
    print("   Red wire → 5V (Pin 2) - Power supply")
    print("   Black wire → One color channel (e.g., Pin 15)")
    print("   Blue wire → Another color channel (e.g., Pin 22)")
    print("   (Colors activate when pins go LOW/GND)")
    
    print("\n📍 Configuration 2: Common Cathode")
    print("   Red wire → One color channel")
    print("   Black wire → GND (Pin 6) - Common ground")
    print("   Blue wire → Another color channel")
    print("   (Colors activate when pins go HIGH/3.3V)")
    
    print("\n📍 Configuration 3: Three independent channels")
    print("   Each wire controls different color")
    
    print("\n" + "="*70)
    print("Interactive Discovery Process")
    print("="*70)
    
    # Get current wiring from user
    print("\n❓ Current wiring:")
    print("   Where is RED wire connected? (enter pin number, e.g., 2 for 5V): ", end="")
    red_pin = input().strip()
    print("   Where is BLACK wire connected? (enter pin number, e.g., 6 for GND): ", end="")
    black_pin = input().strip()
    print("   Where is BLUE wire connected? (enter pin number): ", end="")
    blue_pin = input().strip()
    
    print(f"\n📝 Current configuration:")
    print(f"   RED wire → Pin {red_pin}")
    print(f"   BLACK wire → Pin {black_pin}")
    print(f"   BLUE wire → Pin {blue_pin}")
    
    print("\n🧪 Let's test by connecting Blue wire to different GPIO pins")
    print("   and toggling them HIGH/LOW to see which color lights up")
    
    input("\n⚠️  Press ENTER to start GPIO testing...")
    
    setup_gpio()
    
    try:
        print("\n" + "="*70)
        print("GPIO Pin Testing")
        print("="*70)
        
        for pin_name, pin_num in TEST_PINS.items():
            print(f"\n🔌 Connect BLUE wire to {pin_name} (Physical Pin {pin_num})")
            input("   Press ENTER when connected...")
            
            test_pin_as_output(pin_num, pin_name)
            
            print("\n   What happened?")
            print("   a) LED changed (got brighter/dimmer/different color)")
            print("   b) No visible change")
            response = input("   Your answer (a/b): ").strip().lower()
            
            if response == 'a':
                print(f"\n   ✅ {pin_name} affects the LED!")
                color = input("      What color did you see? (red/green/blue/other): ").strip()
                print(f"      Noted: {pin_name} → {color.upper()}")
        
        print("\n" + "="*70)
        print("Summary")
        print("="*70)
        print("\nBased on your observations, we can now determine:")
        print("1. Which wire controls which color")
        print("2. Whether it's common anode or common cathode")
        print("3. How to implement RED/GREEN/BLUE status control")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrupted")
    finally:
        cleanup_gpio()
        print("\n✅ GPIO cleaned up")

if __name__ == "__main__":
    main()

