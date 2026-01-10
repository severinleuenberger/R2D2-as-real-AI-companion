#!/usr/bin/env python3
"""
Quick LED Test for MCP23017 on I2C Bus 4
Tests each LED individually to verify hardware connections.
"""

import time
import sys

try:
    import board
    import busio
    import digitalio
    from adafruit_mcp230xx.mcp23017 import MCP23017
except ImportError as e:
    print(f"❌ Error: {e}")
    sys.exit(1)

print("=" * 70)
print("MCP23017 LED TEST - Bus 4")
print("=" * 70)
print("")

try:
    # Initialize I2C bus using specific bus number
    print("Connecting to I2C bus 4...")
    from busio import I2C
    i2c = I2C(board.SCL, board.SDA)
    
    # Initialize MCP23017 at 0x20 on bus 4
    print("Connecting to MCP23017 at address 0x20 on bus 4...")
    mcp = MCP23017(i2c, address=0x20)
    print("✅ MCP23017 connected!")
    print("")
    
    # Configure PA0-PA3 as outputs
    print("Setting up 4 LED pins...")
    led_pa0 = mcp.get_pin(0)
    led_pa1 = mcp.get_pin(1)
    led_pa2 = mcp.get_pin(2)
    led_pa3 = mcp.get_pin(3)
    
    for led in [led_pa0, led_pa1, led_pa2, led_pa3]:
        led.direction = digitalio.Direction.OUTPUT
        led.value = False
    print("✅ All pins configured as outputs")
    print("")
    
    print("=" * 70)
    print("TESTING EACH LED")
    print("=" * 70)
    print("")
    
    # Test PA0
    print("TEST 1: PA0 (Board #1 - Red or Blue wire)")
    print("  Turning PA0 ON...")
    led_pa0.value = True
    time.sleep(2)
    color = input("  What COLOR do you see? (red/blue/green/yellow): ").strip()
    led_pa0.value = False
    print(f"  → PA0 controls {color.upper()} LED")
    print("")
    
    # Test PA1
    print("TEST 2: PA1 (Board #1 - Red or Blue wire)")
    print("  Turning PA1 ON...")
    led_pa1.value = True
    time.sleep(2)
    color = input("  What COLOR do you see? (red/blue/green/yellow): ").strip()
    led_pa1.value = False
    print(f"  → PA1 controls {color.upper()} LED")
    print("")
    
    # Test PA2
    print("TEST 3: PA2 (Board #1 - Green wire)")
    print("  Turning PA2 ON...")
    led_pa2.value = True
    time.sleep(2)
    color = input("  What COLOR do you see? (should be GREEN): ").strip()
    led_pa2.value = False
    print(f"  → PA2 controls {color.upper()} LED")
    print("")
    
    # Test PA3
    print("TEST 4: PA3 (Yellow 3mm LED)")
    print("  Turning PA3 ON...")
    led_pa3.value = True
    time.sleep(2)
    color = input("  What COLOR do you see? (should be YELLOW): ").strip()
    led_pa3.value = False
    print(f"  → PA3 controls {color.upper()} LED")
    print("")
    
    # Summary
    print("=" * 70)
    print("TEST COMPLETE!")
    print("=" * 70)
    print("")
    print("✅ All 4 LEDs are working!")
    print("")
    print("Next step: Record your pin mapping for ROS2 configuration")
    print("")
    
except ValueError as e:
    print(f"❌ Error: {e}")
    print("")
    print("MCP23017 not found. Check:")
    print("  1. Run: sudo i2cdetect -y 4")
    print("  2. Verify wiring: SDA, SCL, VCC, GND")
    sys.exit(1)
    
except Exception as e:
    print(f"❌ Error: {e}")
    sys.exit(1)

