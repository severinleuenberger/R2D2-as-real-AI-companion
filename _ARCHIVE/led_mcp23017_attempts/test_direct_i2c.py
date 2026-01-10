#!/usr/bin/env python3
"""
Direct I2C Test - Bypasses board.SCL/SDA auto-detection
Directly opens /dev/i2c-4 device file
"""

import time
import sys

try:
    from adafruit_mcp230xx.mcp23017 import MCP23017
    from busio import I2C
    from board import SCL_1, SDA_1  # Try specific I2C bus pins
except ImportError:
    try:
        # Alternative: use smbus directly
        import smbus2
        print("Using smbus2 library...")
        bus = smbus2.SMBus(4)
        
        # Try to read from MCP23017
        print("Testing I2C bus 4, address 0x20...")
        try:
            # Read IODIRA register (0x00)
            data = bus.read_byte_data(0x20, 0x00)
            print(f"✅ MCP23017 responded! Register 0x00 = 0x{data:02X}")
            print("")
            print("MCP23017 is accessible via smbus2")
            print("But Adafruit library might have issues.")
            print("")
            print("Let's try setting GPIO pins directly via smbus...")
            
            # Set all GPIOA pins as outputs (write 0x00 to IODIRA)
            bus.write_byte_data(0x20, 0x00, 0x00)
            print("✅ GPIOA configured as outputs")
            
            # Turn on PA0
            print("\nTurning PA0 ON...")
            bus.write_byte_data(0x20, 0x12, 0x01)  # GPIOA register, bit 0
            time.sleep(2)
            input("  Press Enter after observing LED...")
            
            # Turn off PA0, turn on PA1
            print("Turning PA1 ON...")
            bus.write_byte_data(0x20, 0x12, 0x02)  # GPIOA register, bit 1
            time.sleep(2)
            input("  Press Enter after observing LED...")
            
            # Turn off PA1, turn on PA2
            print("Turning PA2 ON...")
            bus.write_byte_data(0x20, 0x12, 0x04)  # GPIOA register, bit 2
            time.sleep(2)
            input("  Press Enter after observing LED...")
            
            # Turn off PA2, turn on PA3
            print("Turning PA3 ON...")
            bus.write_byte_data(0x20, 0x12, 0x08)  # GPIOA register, bit 3
            time.sleep(2)
            input("  Press Enter after observing LED...")
            
            # All off
            print("Turning all LEDs OFF...")
            bus.write_byte_data(0x20, 0x12, 0x00)
            
            print("")
            print("=" * 70)
            print("✅ MCP23017 IS WORKING via direct I2C!")
            print("=" * 70)
            print("")
            print("The issue is with the Adafruit library setup,")
            print("but we can work around it.")
            
        except Exception as e:
            print(f"❌ Cannot communicate with device: {e}")
            
    except ImportError:
        print("❌ smbus2 not installed")
        print("Installing smbus2...")
        import subprocess
        subprocess.run(["pip3", "install", "--user", "smbus2"])
        print("Please run this script again")
        sys.exit(1)

