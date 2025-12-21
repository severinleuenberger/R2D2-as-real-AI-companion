#!/usr/bin/env python3
"""
SPI Bus Scanner for WS2812B
Tests different SPI buses to find working configuration

Author: R2D2 Development Team
Date: December 18, 2025
"""

import spidev
import time
import sys

# WS2812B encoding
WS2812B_BIT_1 = 0b1110
WS2812B_BIT_0 = 0b1000

def ws2812b_byte_to_spi(byte_val):
    """Convert byte to WS2812B SPI pattern."""
    spi_bits = []
    for bit_pos in range(7, -1, -1):
        if (byte_val >> bit_pos) & 1:
            spi_bits.append(WS2812B_BIT_1)
        else:
            spi_bits.append(WS2812B_BIT_0)
    
    result = []
    for i in range(0, 8, 2):
        byte = (spi_bits[i] << 4) | spi_bits[i + 1]
        result.append(byte)
    
    return result

def create_simple_frame(r, g, b, num_leds=24):
    """Create LED frame with single color."""
    frame = []
    frame.extend([0x00] * 50)  # Reset
    
    for _ in range(num_leds):
        frame.extend(ws2812b_byte_to_spi(g))  # Green
        frame.extend(ws2812b_byte_to_spi(r))  # Red
        frame.extend(ws2812b_byte_to_spi(b))  # Blue
    
    frame.extend([0x00] * 50)  # Reset
    return frame

def test_spi_bus(bus, device, speed_hz):
    """Test a specific SPI bus configuration."""
    try:
        spi = spidev.SpiDev()
        spi.open(bus, device)
        spi.max_speed_hz = speed_hz
        spi.mode = 0
        spi.bits_per_word = 8
        
        print(f"\n  Testing: SPI{bus}.{device} at {speed_hz/1e6:.1f} MHz")
        
        # Test RED
        frame = create_simple_frame(255, 0, 0)
        spi.writebytes(frame)
        time.sleep(1)
        
        # Test GREEN
        frame = create_simple_frame(0, 255, 0)
        spi.writebytes(frame)
        time.sleep(1)
        
        # Test BLUE
        frame = create_simple_frame(0, 0, 255)
        spi.writebytes(frame)
        time.sleep(1)
        
        # OFF
        frame = create_simple_frame(0, 0, 0)
        spi.writebytes(frame)
        
        spi.close()
        return True
        
    except Exception as e:
        print(f"    ❌ Failed: {e}")
        return False

def main():
    print("="*70)
    print("SPI Bus Scanner for WS2812B")
    print("="*70)
    
    print("\n📍 Current Wiring:")
    print("   LED Red Wire → Pin 2 (5V)")
    print("   LED Black Wire → Pin 6 (GND)")
    print("   LED Blue Wire → Pin 19 (SPI1 MOSI)")
    
    print("\n🔍 We'll test different SPI configurations...")
    print("   Watch the LED for color changes (RED → GREEN → BLUE)")
    
    input("\nPress ENTER to start scanning...")
    
    # Test different SPI buses and speeds
    configs = [
        # (bus, device, speed_hz, description)
        (1, 0, 6400000, "SPI1.0 - Pin 19 (current wiring)"),
        (1, 1, 6400000, "SPI1.1 - Pin 19 (alternate device)"),
        (0, 0, 6400000, "SPI0.0 - Pin 19 (if crosswired)"),
        (1, 0, 3200000, "SPI1.0 - Pin 19 (slower: 3.2 MHz)"),
        (1, 0, 12800000, "SPI1.0 - Pin 19 (faster: 12.8 MHz)"),
        (1, 0, 8000000, "SPI1.0 - Pin 19 (WS2812B ideal: 8 MHz)"),
    ]
    
    print("\n" + "="*70)
    print("Testing SPI Configurations")
    print("="*70)
    
    for bus, device, speed, desc in configs:
        print(f"\n🧪 {desc}")
        success = test_spi_bus(bus, device, speed)
        if success:
            print("    Did you see RED → GREEN → BLUE colors? (y/n): ", end="")
            try:
                response = input().strip().lower()
                if response == 'y':
                    print(f"\n✅ SUCCESS! Working configuration found:")
                    print(f"   Bus: {bus}")
                    print(f"   Device: {device}")
                    print(f"   Speed: {speed/1e6:.1f} MHz")
                    return
            except:
                pass
        
        time.sleep(0.5)
    
    print("\n" + "="*70)
    print("❌ No working configuration found")
    print("="*70)
    print("\nPossible issues:")
    print("1. Data wire not on correct pin (try Pin 21 - SPI1_MOSI alternate)")
    print("2. Need logic level shifter (3.3V → 5V)")
    print("3. LED panel may not be WS2812B compatible")
    print("4. SPI timing doesn't match WS2812B protocol")

if __name__ == "__main__":
    main()

