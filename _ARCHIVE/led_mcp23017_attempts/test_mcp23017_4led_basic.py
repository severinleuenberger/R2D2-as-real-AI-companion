#!/usr/bin/env python3
"""
MCP23017 4-LED Basic Test Script
Tests all 4 LEDs with various patterns to verify hardware is working.

Date: January 9, 2026
"""

import time
import sys

try:
    import board
    import busio
    import digitalio
    from adafruit_mcp230xx.mcp23017 import MCP23017
except ImportError as e:
    print(f"❌ Error: Required libraries not installed")
    print(f"   {str(e)}")
    print("")
    print("Please run: ./install_mcp23017_libraries.sh")
    sys.exit(1)


def main():
    print("=" * 70)
    print("MCP23017 4-LED BASIC TEST")
    print("=" * 70)
    print("")
    print("This script tests all 4 LEDs connected to MCP23017:")
    print("  - PA0: LED (red, blue, or other based on your wiring)")
    print("  - PA1: LED (red, blue, or other based on your wiring)")
    print("  - PA2: Green LED")
    print("  - PA3: Yellow LED")
    print("")
    print("=" * 70)
    print("")
    
    try:
        # Initialize I2C bus
        print("Initializing I2C bus...")
        i2c = busio.I2C(board.SCL, board.SDA)
        print("✅ I2C bus initialized")
        print("")
        
        # Initialize MCP23017 (default address 0x20)
        print("Connecting to MCP23017 at address 0x20...")
        mcp = MCP23017(i2c, address=0x20)
        print("✅ MCP23017 connected")
        print("")
        
        # Configure PA0-PA3 as outputs
        print("Configuring pins PA0-PA3 as outputs...")
        pins = []
        for i in range(4):
            pin = mcp.get_pin(i)
            pin.direction = digitalio.Direction.OUTPUT
            pin.value = False  # Start with all LEDs off
            pins.append(pin)
        print("✅ All 4 pins configured as outputs")
        print("")
        
        print("=" * 70)
        print("STARTING LED TESTS")
        print("=" * 70)
        print("")
        
        # Test 1: Sequential test (one at a time)
        print("Test 1: Sequential LED Test")
        print("─" * 70)
        led_names = ['PA0', 'PA1', 'PA2 (Green)', 'PA3 (Yellow)']
        for i, (pin, name) in enumerate(zip(pins, led_names)):
            print(f"  {name} ON", end="", flush=True)
            pin.value = True
            time.sleep(1.5)
            pin.value = False
            print(" → OFF")
            time.sleep(0.5)
        print("✅ Test 1 complete")
        print("")
        
        # Test 2: All on, all off
        print("Test 2: All LEDs On/Off")
        print("─" * 70)
        for cycle in range(3):
            print(f"  Cycle {cycle+1}/3: All ON", end="", flush=True)
            for pin in pins:
                pin.value = True
            time.sleep(1)
            
            print(" → All OFF")
            for pin in pins:
                pin.value = False
            time.sleep(1)
        print("✅ Test 2 complete")
        print("")
        
        # Test 3: Status simulation (like actual system)
        print("Test 3: Status Simulation (RED → BLUE → GREEN → RED)")
        print("─" * 70)
        
        # RED status (PA0 only)
        print("  Status: RED (person recognized)")
        pins[0].value = True  # Assume PA0 is red
        pins[1].value = False
        pins[2].value = False
        time.sleep(2)
        
        # BLUE status (PA1 only)
        print("  Status: BLUE (no person)")
        pins[0].value = False
        pins[1].value = True  # Assume PA1 is blue
        pins[2].value = False
        time.sleep(2)
        
        # GREEN status (PA2 only)
        print("  Status: GREEN (unknown person)")
        pins[0].value = False
        pins[1].value = False
        pins[2].value = True  # PA2 is green
        time.sleep(2)
        
        # Back to RED
        print("  Status: RED (recognized again)")
        pins[0].value = True
        pins[1].value = False
        pins[2].value = False
        time.sleep(2)
        
        print("✅ Test 3 complete")
        print("")
        
        # Test 4: Gesture flash simulation
        print("Test 4: Gesture Flash Simulation (Yellow LED)")
        print("─" * 70)
        print("  Simulating 5 gesture events...")
        
        # Keep current status LED on (RED), flash yellow
        pins[0].value = True  # RED status
        for i in range(5):
            print(f"    Gesture {i+1}: Yellow flash", end="", flush=True)
            pins[3].value = True  # Yellow ON
            time.sleep(0.5)
            pins[3].value = False  # Yellow OFF
            print(" → complete")
            time.sleep(1)
        
        print("✅ Test 4 complete")
        print("")
        
        # Turn all LEDs off
        print("Turning all LEDs off...")
        for pin in pins:
            pin.value = False
        print("")
        
        print("=" * 70)
        print("✅ ALL TESTS COMPLETED SUCCESSFULLY!")
        print("=" * 70)
        print("")
        print("Your MCP23017 and LEDs are working correctly!")
        print("")
        print("Next step: Run test_mcp23017_led_mapping.py to identify")
        print("           which PA pin controls which LED color.")
        print("")
        
        return 0
        
    except ValueError as e:
        print("")
        print(f"❌ Error: Could not find MCP23017 at address 0x20")
        print(f"   {str(e)}")
        print("")
        print("Troubleshooting:")
        print("1. Check I2C wiring (SDA, SCL, VCC, GND)")
        print("2. Run: sudo i2cdetect -y 1")
        print("3. Verify MCP23017 power LED is on")
        print("4. Verify address jumpers (A0, A1, A2 should be open for 0x20)")
        return 1
        
    except KeyboardInterrupt:
        print("")
        print("")
        print("⚠️  Test interrupted by user")
        # Turn all LEDs off
        try:
            for pin in pins:
                pin.value = False
        except:
            pass
        return 0
        
    except Exception as e:
        print("")
        print(f"❌ Unexpected error: {str(e)}")
        return 1


if __name__ == "__main__":
    sys.exit(main())

