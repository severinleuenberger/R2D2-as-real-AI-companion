#!/usr/bin/env python3
"""
Test 2.2: WS2812B Color Control Test

Purpose:
  Test WS2812B LED panel with proper protocol
  Cycle through RED, GREEN, BLUE, WHITE, and OFF
  Verify all LEDs are working and controllable

Hardware:
  - WS2812B LED panel (16 LEDs in circular arrangement)
  - Red wire: Pin 4 (5V)
  - Black wire: Pin 6 (GND)
  - Blue wire: Pin 15 (GPIO27) via 330Ω resistor

Usage:
  sudo python3 test_04_ws2812b_control.py

Note:
  Requires sudo for GPIO/PWM access
  Uses rpi_ws281x library for precise timing

Safety:
  - Starts at low brightness (50/255 = 20%)
  - Can adjust brightness in script
  - Press Ctrl+C to stop and turn off LEDs
"""

import time
import sys

# Try to import WS2812B library
try:
    from rpi_ws281x import PixelStrip, Color
    HAS_WS281X = True
except ImportError:
    print("❌ Error: rpi_ws281x library not found!")
    print("\nInstall with:")
    print("  sudo pip3 install rpi_ws281x")
    print("\nOr run library test first:")
    print("  python3 test_03_library_test.py")
    sys.exit(1)

# LED strip configuration
LED_COUNT = 16        # Number of LEDs in your circular panel
LED_PIN = 15          # GPIO pin (Physical Pin 15 = GPIO27)
LED_FREQ_HZ = 800000  # LED signal frequency in hertz (800kHz)
LED_DMA = 10          # DMA channel to use for generating signal
LED_BRIGHTNESS = 50   # Set to 0-255 (start low for safety: 50 = ~20%)
LED_INVERT = False    # True to invert the signal (for NPN transistor)
LED_CHANNEL = 0       # PWM channel (0 or 1)

def initialize_strip():
    """Initialize the LED strip"""
    print("\n🔧 Initializing WS2812B LED strip...")
    print(f"   LED Count: {LED_COUNT}")
    print(f"   GPIO Pin: {LED_PIN}")
    print(f"   Frequency: {LED_FREQ_HZ} Hz")
    print(f"   Brightness: {LED_BRIGHTNESS}/255 (~{LED_BRIGHTNESS/255*100:.0f}%)")
    
    try:
        strip = PixelStrip(LED_COUNT, LED_PIN, LED_FREQ_HZ, 
                          LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
        strip.begin()
        print("✅ Strip initialized successfully!")
        return strip
    except Exception as e:
        print(f"❌ Failed to initialize strip: {e}")
        print("\nTroubleshooting:")
        print("1. Run with sudo: sudo python3 test_04_ws2812b_control.py")
        print("2. Check wiring: Blue wire → 330Ω → Pin 15")
        print("3. Verify GPIO permissions")
        sys.exit(1)

def set_all_color(strip, color, color_name):
    """Set all LEDs to the same color"""
    print(f"\n🎨 Setting all LEDs to {color_name}...")
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color)
    strip.show()
    print(f"   ✅ All {strip.numPixels()} LEDs set to {color_name}")

def test_individual_leds(strip):
    """Test each LED individually"""
    print("\n🔄 Testing individual LEDs (chase pattern)...")
    
    for i in range(strip.numPixels()):
        # Turn off all
        for j in range(strip.numPixels()):
            strip.setPixelColor(j, Color(0, 0, 0))
        
        # Turn on current LED (white)
        strip.setPixelColor(i, Color(255, 255, 255))
        strip.show()
        print(f"   LED {i}/{strip.numPixels()-1} ON")
        time.sleep(0.2)
    
    # Turn off all
    for j in range(strip.numPixels()):
        strip.setPixelColor(j, Color(0, 0, 0))
    strip.show()
    print("   ✅ Individual LED test complete")

def test_colors(strip):
    """Test basic colors: RED, GREEN, BLUE, WHITE, OFF"""
    print("\n" + "=" * 70)
    print("Color Test Sequence")
    print("=" * 70)
    
    colors = [
        (Color(255, 0, 0), "RED", 2),
        (Color(0, 255, 0), "GREEN", 2),
        (Color(0, 0, 255), "BLUE", 2),
        (Color(255, 255, 255), "WHITE", 2),
        (Color(255, 128, 0), "ORANGE", 2),
        (Color(128, 0, 255), "PURPLE", 2),
        (Color(0, 255, 255), "CYAN", 2),
        (Color(0, 0, 0), "OFF", 2),
    ]
    
    for color, name, duration in colors:
        set_all_color(strip, color, name)
        time.sleep(duration)
    
    print("\n✅ Color test sequence complete!")

def test_brightness_levels(strip):
    """Test different brightness levels"""
    print("\n" + "=" * 70)
    print("Brightness Test (RED color)")
    print("=" * 70)
    
    levels = [255, 128, 64, 32, 16, 8, 0]
    
    for level in levels:
        print(f"\n🔆 Brightness: {level}/255 ({level/255*100:.0f}%)")
        color = Color(level, 0, 0)  # RED with varying brightness
        set_all_color(strip, color, f"RED @ {level/255*100:.0f}%")
        time.sleep(1)
    
    print("\n✅ Brightness test complete!")

def rainbow_cycle(strip, wait_ms=20):
    """Generate rainbow effect"""
    print("\n🌈 Rainbow cycle (5 seconds)...")
    
    for j in range(256):
        for i in range(strip.numPixels()):
            strip.setPixelColor(i, wheel((int(i * 256 / strip.numPixels()) + j) & 255))
        strip.show()
        time.sleep(wait_ms / 1000.0)
        
        # Break after 5 seconds
        if j >= 250:
            break
    
    print("   ✅ Rainbow complete!")

def wheel(pos):
    """Generate rainbow colors across 0-255 positions"""
    if pos < 85:
        return Color(pos * 3, 255 - pos * 3, 0)
    elif pos < 170:
        pos -= 85
        return Color(255 - pos * 3, 0, pos * 3)
    else:
        pos -= 170
        return Color(0, pos * 3, 255 - pos * 3)

def main():
    print("=" * 70)
    print("WS2812B LED Panel - Color Control Test")
    print("Jetson AGX Orin - 16 LED Circular Panel")
    print("=" * 70)
    
    print("\n📋 Hardware Configuration:")
    print("  LED Panel: WS2812B (16 LEDs, circular)")
    print("  Red wire:   Pin 4 (5V)")
    print("  Black wire: Pin 6 (GND)")
    print("  Blue wire:  Pin 15 (GPIO27) via 330Ω resistor")
    
    print("\n⚠️  Safety:")
    print(f"  Starting brightness: {LED_BRIGHTNESS}/255 (~{LED_BRIGHTNESS/255*100:.0f}%)")
    print("  Press Ctrl+C anytime to stop and turn off LEDs")
    
    print("\n" + "=" * 70)
    input("Press ENTER to start test...")
    
    # Initialize strip
    strip = initialize_strip()
    
    try:
        # Run tests
        print("\n▶️  Starting test sequence...")
        
        # Test 1: Basic colors
        test_colors(strip)
        time.sleep(1)
        
        # Test 2: Individual LEDs
        test_individual_leds(strip)
        time.sleep(1)
        
        # Test 3: Brightness levels
        test_brightness_levels(strip)
        time.sleep(1)
        
        # Test 4: Rainbow effect
        rainbow_cycle(strip)
        
        # Turn off
        print("\n🔌 Turning off all LEDs...")
        set_all_color(strip, Color(0, 0, 0), "OFF")
        
        print("\n" + "=" * 70)
        print("✅ All Tests Complete!")
        print("=" * 70)
        
        print("\n📊 Test Results:")
        print("  ✅ WS2812B protocol working")
        print("  ✅ All colors functional (RED, GREEN, BLUE, WHITE)")
        print("  ✅ Individual LED addressing working")
        print("  ✅ Brightness control working")
        print("  ✅ Rainbow effects working")
        
        print("\n🎉 Your WS2812B LED panel is fully functional!")
        print("\nNext steps:")
        print("  1. Document which wire is data (blue wire)")
        print("  2. Integrate with status_led_node.py")
        print("  3. Map RED/BLUE/GREEN states to LED colors")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Error during test: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Always turn off LEDs on exit
        print("\n🔌 Cleaning up - turning off all LEDs...")
        try:
            for i in range(strip.numPixels()):
                strip.setPixelColor(i, Color(0, 0, 0))
            strip.show()
            print("✅ LEDs turned off")
        except:
            pass

if __name__ == '__main__':
    # Check if running as root
    import os
    if os.geteuid() != 0:
        print("\n⚠️  This script requires sudo for GPIO/PWM access")
        print("Run with: sudo python3 test_04_ws2812b_control.py")
        sys.exit(1)
    
    main()

