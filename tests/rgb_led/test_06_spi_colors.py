#!/usr/bin/env python3
"""
WS2812B RGB LED Control via SPI
Jetson AGX Orin - Color Testing Script

This script controls WS2812B LEDs using SPI interface.
Each WS2812B bit is encoded as multiple SPI bits for precise timing.

Wiring:
  LED Red Wire (Power) → Pin 2 (5V)
  LED Black Wire (GND) → Pin 6 (GND)
  LED Blue Wire (Data) → Pin 19 (SPI1 MOSI)

Author: R2D2 Development Team
Date: December 18, 2025
"""

import spidev
import time
import sys

# ============================================================================
# CONFIGURATION
# ============================================================================

SPI_BUS = 1          # Use SPI1 (tested working)
SPI_DEVICE = 0       # Device 0
SPI_SPEED_HZ = 6400000  # 6.4 MHz (matches WS2812B timing)

NUM_LEDS = 24        # Number of LEDs in the panel (adjust if needed)

# WS2812B bit encoding for SPI
# Each WS2812B bit becomes 4 SPI bits:
# - Bit 1: 0b1110 (HIGH for 3/4 of period)
# - Bit 0: 0b1000 (HIGH for 1/4 of period)
WS2812B_BIT_1 = 0b1110
WS2812B_BIT_0 = 0b1000

# ============================================================================
# WS2812B PROTOCOL FUNCTIONS
# ============================================================================

def ws2812b_byte_to_spi(byte_val):
    """
    Convert a single byte (0-255) to WS2812B SPI bit pattern.
    
    Each bit of the byte is converted to 4 SPI bits.
    Returns 4 bytes (32 SPI bits) for 8 WS2812B bits.
    """
    spi_bits = []
    for bit_pos in range(7, -1, -1):  # MSB first
        if (byte_val >> bit_pos) & 1:
            spi_bits.append(WS2812B_BIT_1)
        else:
            spi_bits.append(WS2812B_BIT_0)
    
    # Pack 4-bit values into bytes
    result = []
    for i in range(0, 8, 2):
        byte = (spi_bits[i] << 4) | spi_bits[i + 1]
        result.append(byte)
    
    return result


def rgb_to_spi(red, green, blue):
    """
    Convert RGB color (0-255 each) to WS2812B SPI data.
    
    WS2812B expects GRB order (Green, Red, Blue).
    Returns list of bytes to send via SPI.
    """
    data = []
    data.extend(ws2812b_byte_to_spi(green))  # Green first
    data.extend(ws2812b_byte_to_spi(red))    # Red second
    data.extend(ws2812b_byte_to_spi(blue))   # Blue third
    return data


def create_led_frame(colors):
    """
    Create a full frame for multiple LEDs.
    
    Args:
        colors: List of (R, G, B) tuples, one per LED
    
    Returns:
        Bytes to send via SPI for the entire LED strip
    """
    frame = []
    
    # Add reset signal (50+ µs of LOW)
    # At 6.4 MHz, need ~400 zero bits = 50 bytes
    frame.extend([0x00] * 50)
    
    # Add each LED's color data
    for r, g, b in colors:
        frame.extend(rgb_to_spi(r, g, b))
    
    # Add trailing reset
    frame.extend([0x00] * 50)
    
    return frame


# ============================================================================
# SPI COMMUNICATION
# ============================================================================

def init_spi():
    """Initialize SPI connection."""
    try:
        spi = spidev.SpiDev()
        spi.open(SPI_BUS, SPI_DEVICE)
        spi.max_speed_hz = SPI_SPEED_HZ
        spi.mode = 0
        spi.bits_per_word = 8
        print(f"✅ SPI initialized: {SPI_SPEED_HZ / 1e6:.1f} MHz")
        return spi
    except Exception as e:
        print(f"❌ Failed to initialize SPI: {e}")
        return None


def send_colors(spi, colors):
    """Send color data to LEDs."""
    frame = create_led_frame(colors)
    spi.writebytes(frame)
    time.sleep(0.001)  # Small delay for latch


def all_leds_off(spi, num_leds):
    """Turn all LEDs off."""
    colors = [(0, 0, 0)] * num_leds
    send_colors(spi, colors)


def all_leds_color(spi, num_leds, r, g, b):
    """Set all LEDs to the same color."""
    colors = [(r, g, b)] * num_leds
    send_colors(spi, colors)


# ============================================================================
# TEST PATTERNS
# ============================================================================

def test_solid_colors(spi, num_leds):
    """Test solid colors: RED, GREEN, BLUE, WHITE."""
    print("\n" + "="*70)
    print("Testing Solid Colors")
    print("="*70)
    
    test_colors = [
        ("RED", 255, 0, 0),
        ("GREEN", 0, 255, 0),
        ("BLUE", 0, 0, 255),
        ("WHITE", 255, 255, 255),
        ("YELLOW", 255, 255, 0),
        ("CYAN", 0, 255, 255),
        ("MAGENTA", 255, 0, 255),
    ]
    
    for name, r, g, b in test_colors:
        print(f"\n🎨 Setting all LEDs to {name} (R={r}, G={g}, B={b})...")
        all_leds_color(spi, num_leds, r, g, b)
        time.sleep(2)
    
    print("\n💡 Turning all LEDs OFF...")
    all_leds_off(spi, num_leds)
    time.sleep(1)


def test_individual_leds(spi, num_leds):
    """Test each LED individually."""
    print("\n" + "="*70)
    print("Testing Individual LEDs")
    print("="*70)
    
    for i in range(num_leds):
        colors = [(0, 0, 0)] * num_leds
        colors[i] = (255, 0, 0)  # Red
        print(f"🔴 Lighting LED {i + 1}/{num_leds}...")
        send_colors(spi, colors)
        time.sleep(0.3)
    
    all_leds_off(spi, num_leds)


def test_status_colors(spi, num_leds):
    """Test R2D2 status colors (RED, GREEN, BLUE)."""
    print("\n" + "="*70)
    print("Testing R2D2 Status Colors")
    print("="*70)
    
    print("\n🔴 RED Status (Person Recognized)...")
    all_leds_color(spi, num_leds, 255, 0, 0)
    time.sleep(3)
    
    print("\n🟢 GREEN Status (Unknown Person)...")
    all_leds_color(spi, num_leds, 0, 255, 0)
    time.sleep(3)
    
    print("\n🔵 BLUE Status (Lost/Idle)...")
    all_leds_color(spi, num_leds, 0, 0, 255)
    time.sleep(3)
    
    print("\n💡 OFF...")
    all_leds_off(spi, num_leds)


# ============================================================================
# MAIN
# ============================================================================

def main():
    print("="*70)
    print("WS2812B Color Test via SPI")
    print("Jetson AGX Orin - RGB LED Control")
    print("="*70)
    
    print(f"\n📋 Configuration:")
    print(f"   SPI Bus: {SPI_BUS}")
    print(f"   SPI Device: {SPI_DEVICE}")
    print(f"   Speed: {SPI_SPEED_HZ / 1e6:.1f} MHz")
    print(f"   Number of LEDs: {NUM_LEDS}")
    
    print(f"\n📍 Wiring Check:")
    print(f"   LED Red Wire (5V) → Pin 2")
    print(f"   LED Black Wire (GND) → Pin 6")
    print(f"   LED Blue Wire (Data) → Pin 19 (SPI1 MOSI)")
    
    input("\n⚠️  Press ENTER when wiring is ready...")
    
    # Initialize SPI
    spi = init_spi()
    if not spi:
        sys.exit(1)
    
    try:
        # Safety: Turn all LEDs off first
        print("\n💡 Turning all LEDs OFF (safety)...")
        all_leds_off(spi, NUM_LEDS)
        time.sleep(1)
        
        # Run tests
        test_solid_colors(spi, NUM_LEDS)
        test_individual_leds(spi, NUM_LEDS)
        test_status_colors(spi, NUM_LEDS)
        
        print("\n" + "="*70)
        print("✅ All tests complete!")
        print("="*70)
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrupted by user")
    except Exception as e:
        print(f"\n\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n💡 Turning all LEDs OFF...")
        all_leds_off(spi, NUM_LEDS)
        spi.close()
        print("✅ SPI closed")


if __name__ == "__main__":
    main()

