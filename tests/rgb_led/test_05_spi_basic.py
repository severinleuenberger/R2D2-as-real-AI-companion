#!/usr/bin/env python3
"""
Test 2.1: SPI Basic Communication Test

Purpose:
  Verify SPI is enabled and working on Jetson AGX Orin
  Test basic SPI communication before WS2812B control

Usage:
  python3 test_05_spi_basic.py

Prerequisites:
  - SPI enabled via jetson-io
  - spidev library installed (sudo pip3 install spidev)

Output:
  - Detects available SPI devices
  - Tests basic SPI read/write
  - Verifies SPI frequency settings
"""

import sys

def check_spi_devices():
    """Check for available SPI devices"""
    print("=" * 70)
    print("Checking for SPI Devices")
    print("=" * 70)
    
    import os
    import glob
    
    spi_devices = glob.glob('/dev/spidev*')
    
    if not spi_devices:
        print("❌ No SPI devices found!")
        print("\nSPI is not enabled. To enable:")
        print("1. Run: sudo /opt/nvidia/jetson-io/jetson-io.py")
        print("2. Select 'Configure Jetson 40-pin header'")
        print("3. Enable SPI1 or SPI3")
        print("4. Save and reboot")
        print("\nOr try: sudo /opt/nvidia/jetson-io/config-by-pin.py")
        return None
    
    print(f"✅ Found {len(spi_devices)} SPI device(s):")
    for dev in spi_devices:
        print(f"   - {dev}")
    
    return spi_devices[0]  # Return first device

def test_spidev_library():
    """Test if spidev library is installed"""
    print("\n" + "=" * 70)
    print("Checking spidev Library")
    print("=" * 70)
    
    try:
        import spidev
        print("✅ spidev library is installed")
        print(f"   Version: {spidev.__version__ if hasattr(spidev, '__version__') else 'Unknown'}")
        return True
    except ImportError:
        print("❌ spidev library is NOT installed")
        print("\nInstall with:")
        print("  sudo pip3 install spidev")
        return False

def test_spi_communication(spi_device):
    """Test basic SPI communication"""
    print("\n" + "=" * 70)
    print("Testing SPI Communication")
    print("=" * 70)
    
    try:
        import spidev
        
        # Extract bus and device number from path
        # /dev/spidev0.0 → bus=0, device=0
        parts = spi_device.split('spidev')[1].split('.')
        bus = int(parts[0])
        device = int(parts[1])
        
        print(f"\n📡 Opening SPI: bus={bus}, device={device}")
        
        # Open SPI
        spi = spidev.SpiDev()
        spi.open(bus, device)
        
        # Configure SPI for WS2812B
        # We'll use 6.4 MHz (6400000 Hz) for WS2812B timing
        spi.max_speed_hz = 6400000
        spi.mode = 0b00  # CPOL=0, CPHA=0
        spi.bits_per_word = 8
        
        print(f"✅ SPI opened successfully")
        print(f"   Speed: {spi.max_speed_hz / 1000000:.1f} MHz")
        print(f"   Mode: {spi.mode}")
        print(f"   Bits per word: {spi.bits_per_word}")
        
        # Test write (send zeros - reset signal for WS2812B)
        print("\n📤 Sending test data...")
        test_data = [0x00] * 100  # 100 bytes of zeros
        spi.writebytes(test_data)
        print("✅ SPI write successful")
        
        # Close SPI
        spi.close()
        print("✅ SPI closed successfully")
        
        return True
        
    except Exception as e:
        print(f"❌ SPI communication failed: {e}")
        print("\nTroubleshooting:")
        print("1. Verify SPI is enabled (check /dev/spidev*)")
        print("2. Check permissions: sudo usermod -a -G spi $USER")
        print("3. Try with sudo: sudo python3 test_05_spi_basic.py")
        return False

def print_next_steps(success):
    """Print next steps based on test results"""
    print("\n" + "=" * 70)
    print("Next Steps")
    print("=" * 70)
    
    if success:
        print("\n✅ SPI is ready for WS2812B control!")
        print("\nNext:")
        print("1. Connect LED data wire to SPI MOSI pin:")
        print("   - SPI1 MOSI: Pin 19 (physical)")
        print("   - SPI3 MOSI: Pin 21 (physical)")
        print("\n2. Run WS2812B color test:")
        print("   python3 test_06_spi_colors.py")
    else:
        print("\n❌ SPI is not ready")
        print("\nRequired actions:")
        print("1. Enable SPI via jetson-io")
        print("2. Install spidev library")
        print("3. Reboot system")
        print("4. Run this test again")

def main():
    print("=" * 70)
    print("SPI Basic Communication Test")
    print("Jetson AGX Orin - SPI Verification")
    print("=" * 70)
    
    success = True
    
    # Check for SPI devices
    spi_device = check_spi_devices()
    if not spi_device:
        success = False
    
    # Check spidev library
    if not test_spidev_library():
        success = False
    
    # Test SPI communication (only if device exists and library is installed)
    if success:
        if not test_spi_communication(spi_device):
            success = False
    
    # Print next steps
    print_next_steps(success)
    
    print("\n" + "=" * 70)
    print("Rollback Information")
    print("=" * 70)
    print("\nIf you want to disable SPI:")
    print("1. sudo /opt/nvidia/jetson-io/jetson-io.py")
    print("2. Uncheck SPI option")
    print("3. Save and reboot")
    print("\nTo remove spidev library:")
    print("  sudo pip3 uninstall -y spidev")

if __name__ == '__main__':
    main()

