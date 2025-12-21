#!/usr/bin/env python3
"""
Test 2.1: WS2812B Library Installation and Compatibility Test

Purpose:
  Check which WS2812B libraries are available on Jetson AGX Orin
  Test compatibility before attempting LED control

Libraries to Test:
  1. rpi_ws281x - Most common, may work on Jetson
  2. Adafruit CircuitPython NeoPixel - Alternative
  3. Adafruit Blinka - CircuitPython compatibility layer

Usage:
  python3 test_03_library_test.py

Output:
  - Which libraries are installed
  - Which libraries are working
  - Installation commands if needed
"""

import sys

def test_rpi_ws281x():
    """Test rpi_ws281x library"""
    print("\n" + "=" * 70)
    print("Testing rpi_ws281x Library")
    print("=" * 70)
    
    try:
        from rpi_ws281x import PixelStrip, Color
        print("✅ rpi_ws281x library is INSTALLED")
        print(f"   - PixelStrip class: Available")
        print(f"   - Color class: Available")
        return True
    except ImportError as e:
        print("❌ rpi_ws281x library is NOT installed")
        print(f"   Error: {e}")
        print("\n   Install with:")
        print("   sudo pip3 install rpi_ws281x")
        return False

def test_adafruit_circuitpython():
    """Test Adafruit CircuitPython NeoPixel"""
    print("\n" + "=" * 70)
    print("Testing Adafruit CircuitPython NeoPixel")
    print("=" * 70)
    
    try:
        import board
        import neopixel
        print("✅ Adafruit CircuitPython libraries are INSTALLED")
        print(f"   - board module: Available")
        print(f"   - neopixel module: Available")
        
        # Try to access board pins
        try:
            pins = dir(board)
            gpio_pins = [p for p in pins if p.startswith('D') or p.startswith('GPIO')]
            print(f"   - Available pins: {len(gpio_pins)}")
            return True
        except Exception as e:
            print(f"   ⚠️  Pins not accessible: {e}")
            return False
            
    except ImportError as e:
        print("❌ Adafruit CircuitPython libraries are NOT installed")
        print(f"   Error: {e}")
        print("\n   Install with:")
        print("   sudo pip3 install adafruit-circuitpython-neopixel")
        print("   sudo pip3 install adafruit-blinka")
        return False

def test_jetson_gpio():
    """Test Jetson.GPIO library"""
    print("\n" + "=" * 70)
    print("Testing Jetson.GPIO Library")
    print("=" * 70)
    
    try:
        import Jetson.GPIO as GPIO
        print("✅ Jetson.GPIO library is INSTALLED")
        print(f"   Version: {GPIO.VERSION}")
        print(f"   Board: {GPIO.model}")
        return True
    except ImportError as e:
        print("❌ Jetson.GPIO library is NOT installed")
        print(f"   Error: {e}")
        return False

def print_installation_guide(has_ws281x, has_circuitpython):
    """Print installation instructions based on what's missing"""
    print("\n" + "=" * 70)
    print("Installation Guide")
    print("=" * 70)
    
    if not has_ws281x and not has_circuitpython:
        print("\n❌ No WS2812B libraries found!")
        print("\n📦 Recommended: Install rpi_ws281x (most compatible)")
        print("\nInstallation steps:")
        print("1. Install dependencies:")
        print("   sudo apt-get update")
        print("   sudo apt-get install -y gcc make build-essential python3-dev git scons swig")
        print("\n2. Install rpi_ws281x:")
        print("   sudo pip3 install rpi_ws281x")
        print("\n3. Alternative - Install Adafruit libraries:")
        print("   sudo pip3 install adafruit-circuitpython-neopixel")
        print("   sudo pip3 install adafruit-blinka")
        
    elif has_ws281x:
        print("\n✅ rpi_ws281x is installed - Ready for WS2812B control!")
        print("\nNext step:")
        print("   python3 test_04_ws2812b_control.py")
        
    elif has_circuitpython:
        print("\n✅ Adafruit CircuitPython is installed - Ready for WS2812B control!")
        print("\nNext step:")
        print("   python3 test_04_ws2812b_control.py")

def main():
    print("=" * 70)
    print("WS2812B Library Compatibility Test")
    print("Jetson AGX Orin - Library Detection")
    print("=" * 70)
    
    # Test Jetson.GPIO first (required for both methods)
    has_jetson_gpio = test_jetson_gpio()
    
    if not has_jetson_gpio:
        print("\n❌ Jetson.GPIO is required but not found!")
        print("   Install with: sudo pip3 install Jetson.GPIO")
        sys.exit(1)
    
    # Test WS2812B libraries
    has_ws281x = test_rpi_ws281x()
    has_circuitpython = test_adafruit_circuitpython()
    
    # Print summary
    print("\n" + "=" * 70)
    print("Summary")
    print("=" * 70)
    
    if has_ws281x:
        print("\n✅ READY: rpi_ws281x library is available")
        print("   You can use PixelStrip for WS2812B control")
    elif has_circuitpython:
        print("\n✅ READY: Adafruit CircuitPython is available")
        print("   You can use NeoPixel for WS2812B control")
    else:
        print("\n❌ NOT READY: No WS2812B libraries found")
        print("   Installation required before testing LEDs")
    
    # Print installation guide
    print_installation_guide(has_ws281x, has_circuitpython)
    
    print("\n" + "=" * 70)
    print("Troubleshooting")
    print("=" * 70)
    print("\nIf installation fails:")
    print("1. Check you have sudo privileges")
    print("2. Verify internet connection")
    print("3. Try installing build dependencies first")
    print("4. Check for ARM64 compatibility issues")
    print("\nFor help, see: https://github.com/jgarff/rpi_ws281x_python")

if __name__ == '__main__':
    main()

