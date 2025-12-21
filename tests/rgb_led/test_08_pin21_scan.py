#!/usr/bin/env python3
"""
Pin 21 Comprehensive SPI Test
Tests all possible SPI bus configurations for Pin 21

Pin 21 may be on SPI3, SPI1, or another bus depending on Jetson configuration.

Author: R2D2 Development Team
Date: December 18, 2025
"""

import spidev
import time

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

def create_test_frame(r, g, b, num_leds=24):
    """Create test frame with single color."""
    frame = [0x00] * 50  # Reset
    
    for _ in range(num_leds):
        frame.extend(ws2812b_byte_to_spi(g))  # Green
        frame.extend(ws2812b_byte_to_spi(r))  # Red
        frame.extend(ws2812b_byte_to_spi(b))  # Blue
    
    frame.extend([0x00] * 50)  # Reset
    return frame

def test_config(bus, device, speed):
    """Test a specific SPI configuration."""
    try:
        spi = spidev.SpiDev()
        spi.open(bus, device)
        spi.max_speed_hz = speed
        spi.mode = 0
        spi.bits_per_word = 8
        
        print(f"  📡 Opened: /dev/spidev{bus}.{device} @ {speed/1e6:.1f} MHz")
        
        # Quick color cycle
        colors = [
            (255, 0, 0, "RED"),
            (0, 255, 0, "GREEN"),
            (0, 0, 255, "BLUE"),
        ]
        
        for r, g, b, name in colors:
            print(f"     → {name}...", end="", flush=True)
            frame = create_test_frame(r, g, b)
            spi.writebytes(frame)
            time.sleep(0.8)
            print(" ✓")
        
        # OFF
        frame = create_test_frame(0, 0, 0)
        spi.writebytes(frame)
        
        spi.close()
        return True
        
    except Exception as e:
        print(f"  ❌ Failed: {e}")
        return False

def main():
    print("="*70)
    print("Pin 21 Comprehensive SPI Test")
    print("="*70)
    
    print("\n📍 Wiring:")
    print("   LED Red Wire → Pin 2 (5V)")
    print("   LED Black Wire → Pin 6 (GND)")
    print("   LED Blue Wire → Pin 21 (SPI MOSI, NO RESISTOR)")
    
    print("\n🔍 We'll test ALL possible SPI bus/device combinations")
    print("   Watch for RED → GREEN → BLUE color changes")
    
    input("\n⚠️  Press ENTER when Blue wire is on Pin 21...")
    
    # All possible SPI configurations
    configs = [
        # (bus, device, speed, description)
        (0, 0, 6400000, "SPI0.0 @ 6.4 MHz"),
        (0, 1, 6400000, "SPI0.1 @ 6.4 MHz"),
        (1, 0, 6400000, "SPI1.0 @ 6.4 MHz"),
        (1, 1, 6400000, "SPI1.1 @ 6.4 MHz"),
        (0, 0, 8000000, "SPI0.0 @ 8.0 MHz"),
        (1, 0, 8000000, "SPI1.0 @ 8.0 MHz"),
        (0, 0, 3200000, "SPI0.0 @ 3.2 MHz"),
        (1, 0, 3200000, "SPI1.0 @ 3.2 MHz"),
    ]
    
    print("\n" + "="*70)
    print("Testing Configurations")
    print("="*70)
    
    working_configs = []
    
    for i, (bus, device, speed, desc) in enumerate(configs, 1):
        print(f"\n🧪 Test {i}/{len(configs)}: {desc}")
        
        success = test_config(bus, device, speed)
        
        if success:
            response = input("\n     Did you see RED → GREEN → BLUE? (y/n): ").strip().lower()
            if response == 'y':
                working_configs.append((bus, device, speed, desc))
                print(f"\n     ✅ WORKING! Saved configuration.")
        
        time.sleep(0.3)
    
    print("\n" + "="*70)
    print("Results")
    print("="*70)
    
    if working_configs:
        print("\n✅ Working configuration(s) found:")
        for bus, device, speed, desc in working_configs:
            print(f"\n   {desc}")
            print(f"   - Bus: {bus}")
            print(f"   - Device: {device}")
            print(f"   - Speed: {speed} Hz ({speed/1e6:.1f} MHz)")
            print(f"   - Device file: /dev/spidev{bus}.{device}")
    else:
        print("\n❌ No working configuration found on Pin 21")
        print("\nThis suggests:")
        print("1. Pin 21 may not be SPI MOSI on this Jetson model")
        print("2. LED may require 5V logic levels (3.3V too low)")
        print("3. LED protocol may not be standard WS2812B")
        print("\nNext step: Consider logic level shifter or use white LED")

if __name__ == "__main__":
    main()

