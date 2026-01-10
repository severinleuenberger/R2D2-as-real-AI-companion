#!/usr/bin/env python3
"""
MCP23017 LED Mapping Test Script
Tests which MCP23017 output pin controls which LED color on your board.

This script helps identify if wire colors match LED colors or if they're reversed.

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
    print("MCP23017 LED MAPPING TEST")
    print("=" * 70)
    print("")
    print("This script will test each MCP23017 output pin to help you identify")
    print("which pin controls which LED color on your board.")
    print("")
    print("Watch your LED board and record which COLOR lights up for each test.")
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
        
        # Configure PA0, PA1, PA2, PA3 as outputs
        print("Configuring pins PA0-PA3 as outputs...")
        pins = []
        for i in range(4):
            pin = mcp.get_pin(i)
            pin.direction = digitalio.Direction.OUTPUT
            pin.value = False  # Start with all LEDs off
            pins.append(pin)
        print("✅ All pins configured")
        print("")
        
        print("=" * 70)
        print("STARTING LED MAPPING TESTS")
        print("=" * 70)
        print("")
        
        # Mapping results storage
        mapping = {}
        
        # Test PA0
        print("TEST 1: PA0 (Red wire on Board #1)")
        print("─" * 70)
        print("Turning PA0 ON...")
        pins[0].value = True
        time.sleep(2)
        
        color = input("  What COLOR LED lit up? (type: red/blue/green/yellow): ").strip().lower()
        mapping['PA0'] = color
        
        pins[0].value = False
        time.sleep(1)
        print("")
        
        # Test PA1
        print("TEST 2: PA1 (Blue wire on Board #1)")
        print("─" * 70)
        print("Turning PA1 ON...")
        pins[1].value = True
        time.sleep(2)
        
        color = input("  What COLOR LED lit up? (type: red/blue/green/yellow): ").strip().lower()
        mapping['PA1'] = color
        
        pins[1].value = False
        time.sleep(1)
        print("")
        
        # Test PA2
        print("TEST 3: PA2 (Green wire on Board #1)")
        print("─" * 70)
        print("Turning PA2 ON...")
        pins[2].value = True
        time.sleep(2)
        
        color = input("  What COLOR LED lit up? (type: red/blue/green/yellow): ").strip().lower()
        mapping['PA2'] = color
        
        pins[2].value = False
        time.sleep(1)
        print("")
        
        # Test PA3
        print("TEST 4: PA3 (Yellow LED direct)")
        print("─" * 70)
        print("Turning PA3 ON...")
        pins[3].value = True
        time.sleep(2)
        
        color = input("  What COLOR LED lit up? (type: red/blue/green/yellow): ").strip().lower()
        mapping['PA3'] = color
        
        pins[3].value = False
        time.sleep(1)
        print("")
        
        # Display results
        print("=" * 70)
        print("MAPPING RESULTS")
        print("=" * 70)
        print("")
        print("MCP23017 Pin → LED Color")
        print("─" * 70)
        for pin, color in mapping.items():
            print(f"  {pin:4s} → {color.upper()}")
        print("")
        
        # Determine if wiring is normal or reversed
        print("=" * 70)
        print("CONFIGURATION FOR ROS2 NODE")
        print("=" * 70)
        print("")
        
        # Create reverse mapping (color → pin number)
        color_to_pin = {}
        for pin, color in mapping.items():
            pin_num = int(pin.replace('PA', ''))
            color_to_pin[color] = pin_num
        
        if 'red' in color_to_pin and 'blue' in color_to_pin and 'green' in color_to_pin and 'yellow' in color_to_pin:
            print("Use these pin assignments in mcp23017_status_led_node.py:")
            print("")
            print(f"    self.pin_red = mcp.get_pin({color_to_pin['red']})    # Status: RED (person recognized)")
            print(f"    self.pin_blue = mcp.get_pin({color_to_pin['blue']})   # Status: BLUE (no person)")
            print(f"    self.pin_green = mcp.get_pin({color_to_pin['green']})  # Status: GREEN (unknown)")
            print(f"    self.pin_yellow = mcp.get_pin({color_to_pin['yellow']}) # Gesture flash")
            print("")
            
            if color_to_pin['red'] != 0 or color_to_pin['blue'] != 1:
                print("⚠️  NOTE: Wire colors don't match LED colors (reversed wiring)")
                print("    This is OK - software will handle it correctly!")
            else:
                print("✅ Wire colors match LED colors (normal wiring)")
        else:
            print("⚠️  Warning: Not all colors detected correctly.")
            print("    Please verify your wiring and run test again.")
        
        print("")
        print("=" * 70)
        print("TEST COMPLETE")
        print("=" * 70)
        print("")
        print("Save these pin assignments - you'll need them for configuration!")
        print("")
        
        # Turn all LEDs off
        for pin in pins:
            pin.value = False
        
        return 0
        
    except ValueError as e:
        print("")
        print(f"❌ Error: Could not find MCP23017 at address 0x20")
        print(f"   {str(e)}")
        print("")
        print("Troubleshooting:")
        print("1. Check I2C wiring (SDA, SCL, VCC, GND)")
        print("2. Run: sudo i2cdetect -y 1")
        print("3. Verify MCP23017 address jumpers (should be 0x20)")
        return 1
        
    except KeyboardInterrupt:
        print("")
        print("")
        print("⚠️  Test interrupted by user")
        # Turn all LEDs off
        for pin in pins:
            pin.value = False
        return 0
        
    except Exception as e:
        print("")
        print(f"❌ Unexpected error: {str(e)}")
        return 1


if __name__ == "__main__":
    sys.exit(main())

