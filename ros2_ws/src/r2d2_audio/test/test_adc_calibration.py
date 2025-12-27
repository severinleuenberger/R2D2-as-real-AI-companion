#!/usr/bin/env python3
"""
R2D2 ADC Calibration and Test Script

Tests the B5K potentiometer connected to ADS1115 ADC for volume control.
Provides calibration, monitoring, and verification capabilities.

Hardware:
  - B5K (5kΩ) Linear Potentiometer
  - ADS1115 16-bit ADC (I2C address 0x48)
  - Connected to Jetson I2C bus

Usage:
    python3 test_adc_calibration.py --detect
    python3 test_adc_calibration.py --calibrate
    python3 test_adc_calibration.py --monitor
    python3 test_adc_calibration.py --verify

Author: R2D2 Audio System
Date: December 2025
"""

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Optional, Dict, Any, Tuple

# Calibration file location
CALIBRATION_FILE = Path.home() / '.r2d2' / 'adc_calibration.json'

# Default calibration values (16-bit ADC range)
DEFAULT_ADC_MIN = 0
DEFAULT_ADC_MAX = 26400  # ~80% of 32767 (typical for 3.3V reference)

# Volume mapping based on baseline test results
MAX_VOLUME_CAP = 0.7  # Distortion starts at 0.7-1.0

# Try to import ADC library
try:
    import board
    import busio
    import adafruit_ads1x15.ads1115 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn
    ADC_AVAILABLE = True
except ImportError:
    ADC_AVAILABLE = False
    print("Warning: ADC library not available.")
    print("Install with: pip3 install adafruit-circuitpython-ads1x15")


def detect_adc(i2c_bus: int = 1, address: int = 0x48) -> Tuple[bool, str]:
    """
    Detect if ADS1115 ADC is connected and responding.
    
    Args:
        i2c_bus: I2C bus number
        address: I2C address of ADS1115
    
    Returns:
        Tuple of (success, message)
    """
    if not ADC_AVAILABLE:
        return False, "ADC library not installed. Run: pip3 install adafruit-circuitpython-ads1x15"
    
    try:
        # Try to create I2C connection
        i2c = busio.I2C(board.SCL, board.SDA)
        
        # Try to create ADS1115 instance
        ads = ADS.ADS1115(i2c, address=address)
        
        # Try to read a value
        chan = AnalogIn(ads, ADS.P0)
        value = chan.value
        voltage = chan.voltage
        
        return True, f"ADS1115 detected at address 0x{address:02X}\n  Raw value: {value}\n  Voltage: {voltage:.3f}V"
    
    except ValueError as e:
        return False, f"ADS1115 not found at address 0x{address:02X}: {e}"
    except Exception as e:
        return False, f"Error detecting ADC: {e}"


def read_adc_value() -> Optional[Tuple[int, float]]:
    """
    Read current ADC value and voltage.
    
    Returns:
        Tuple of (raw_value, voltage) or None if error
    """
    if not ADC_AVAILABLE:
        return None
    
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        ads = ADS.ADS1115(i2c)
        chan = AnalogIn(ads, ADS.P0)
        return chan.value, chan.voltage
    except Exception as e:
        print(f"Error reading ADC: {e}")
        return None


def load_calibration() -> Dict[str, Any]:
    """
    Load calibration data from file.
    
    Returns:
        Calibration dictionary
    """
    if CALIBRATION_FILE.exists():
        try:
            with open(CALIBRATION_FILE, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f"Warning: Could not load calibration: {e}")
    
    return {
        'adc_min': DEFAULT_ADC_MIN,
        'adc_max': DEFAULT_ADC_MAX,
        'max_volume': MAX_VOLUME_CAP,
        'calibrated': False
    }


def save_calibration(calibration: Dict[str, Any]) -> bool:
    """
    Save calibration data to file.
    
    Args:
        calibration: Calibration dictionary
    
    Returns:
        True if saved successfully
    """
    try:
        CALIBRATION_FILE.parent.mkdir(parents=True, exist_ok=True)
        with open(CALIBRATION_FILE, 'w') as f:
            json.dump(calibration, f, indent=2)
        print(f"Calibration saved to: {CALIBRATION_FILE}")
        return True
    except Exception as e:
        print(f"Error saving calibration: {e}")
        return False


def map_adc_to_volume(adc_value: int, calibration: Dict[str, Any]) -> float:
    """
    Map ADC value to volume (0.0 - max_volume).
    
    Args:
        adc_value: Raw ADC value
        calibration: Calibration dictionary
    
    Returns:
        Volume value (0.0 - max_volume)
    """
    adc_min = calibration.get('adc_min', DEFAULT_ADC_MIN)
    adc_max = calibration.get('adc_max', DEFAULT_ADC_MAX)
    max_volume = calibration.get('max_volume', MAX_VOLUME_CAP)
    
    # Clamp to calibrated range
    adc_value = max(adc_min, min(adc_max, adc_value))
    
    # Linear mapping
    if adc_max <= adc_min:
        return 0.0
    
    normalized = (adc_value - adc_min) / (adc_max - adc_min)
    volume = normalized * max_volume
    
    return max(0.0, min(max_volume, volume))


def run_detect():
    """Run ADC detection test."""
    print("=" * 60)
    print("ADC Detection Test")
    print("=" * 60)
    
    success, message = detect_adc()
    
    if success:
        print(f"\n✓ {message}")
        return 0
    else:
        print(f"\n✗ {message}")
        return 1


def run_calibrate():
    """Run interactive calibration."""
    print("=" * 60)
    print("ADC Calibration - B5K Potentiometer")
    print("=" * 60)
    
    if not ADC_AVAILABLE:
        print("\n✗ ADC library not available. Install with:")
        print("  pip3 install adafruit-circuitpython-ads1x15")
        return 1
    
    # Check ADC is connected
    success, message = detect_adc()
    if not success:
        print(f"\n✗ {message}")
        return 1
    
    print("\nThis will calibrate the volume knob by recording")
    print("the minimum and maximum ADC values.\n")
    
    calibration = {
        'calibration_date': time.strftime('%Y-%m-%d %H:%M:%S'),
        'max_volume': MAX_VOLUME_CAP,
        'calibrated': False
    }
    
    # Step 1: Calibrate minimum (knob fully counter-clockwise)
    print("-" * 60)
    print("STEP 1: Minimum Position")
    print("-" * 60)
    print("Turn the volume knob fully COUNTER-CLOCKWISE (minimum position).")
    input("Press Enter when ready...")
    
    readings = []
    print("Reading ADC values (3 seconds)...")
    for _ in range(30):
        result = read_adc_value()
        if result:
            readings.append(result[0])
        time.sleep(0.1)
    
    if not readings:
        print("✗ Failed to read ADC values")
        return 1
    
    adc_min = int(sum(readings) / len(readings))
    print(f"  Minimum ADC value: {adc_min}")
    print(f"  (Range: {min(readings)} - {max(readings)})")
    calibration['adc_min'] = adc_min
    
    # Step 2: Calibrate maximum (knob fully clockwise)
    print("\n" + "-" * 60)
    print("STEP 2: Maximum Position")
    print("-" * 60)
    print("Turn the volume knob fully CLOCKWISE (maximum position).")
    input("Press Enter when ready...")
    
    readings = []
    print("Reading ADC values (3 seconds)...")
    for _ in range(30):
        result = read_adc_value()
        if result:
            readings.append(result[0])
        time.sleep(0.1)
    
    if not readings:
        print("✗ Failed to read ADC values")
        return 1
    
    adc_max = int(sum(readings) / len(readings))
    print(f"  Maximum ADC value: {adc_max}")
    print(f"  (Range: {min(readings)} - {max(readings)})")
    calibration['adc_max'] = adc_max
    
    # Validate
    adc_range = adc_max - adc_min
    print("\n" + "-" * 60)
    print("CALIBRATION RESULTS")
    print("-" * 60)
    print(f"  ADC Minimum: {adc_min}")
    print(f"  ADC Maximum: {adc_max}")
    print(f"  ADC Range: {adc_range}")
    print(f"  Max Volume: {MAX_VOLUME_CAP} (capped to prevent distortion)")
    
    if adc_range < 1000:
        print("\n⚠️  WARNING: ADC range is very small!")
        print("Check wiring: VCC, GND, and Wiper connections.")
        print("Expected range: 10000+ for a properly wired potentiometer.")
    elif adc_max <= adc_min:
        print("\n✗ ERROR: Maximum is less than or equal to minimum!")
        print("The potentiometer may be wired backwards.")
        return 1
    else:
        calibration['calibrated'] = True
        print("\n✓ Calibration looks good!")
    
    # Save calibration
    print("\n" + "-" * 60)
    save = input("Save calibration? [Y/n]: ").strip().lower()
    if save != 'n':
        save_calibration(calibration)
    
    # Test mapping
    print("\n" + "-" * 60)
    print("STEP 3: Test Mapping")
    print("-" * 60)
    print("Rotate the knob to test volume mapping.")
    print("Press Ctrl+C to exit.\n")
    
    try:
        while True:
            result = read_adc_value()
            if result:
                adc_value, voltage = result
                volume = map_adc_to_volume(adc_value, calibration)
                bar_len = int(volume / MAX_VOLUME_CAP * 40)
                bar = '█' * bar_len + '░' * (40 - bar_len)
                print(f"\rADC: {adc_value:5d} | Voltage: {voltage:.2f}V | Volume: {volume:.2f} [{bar}]", end='', flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\nCalibration complete!")
    
    return 0


def run_monitor():
    """Run continuous ADC monitoring."""
    print("=" * 60)
    print("ADC Monitor - Live Values")
    print("=" * 60)
    
    if not ADC_AVAILABLE:
        print("\n✗ ADC library not available.")
        return 1
    
    # Load calibration
    calibration = load_calibration()
    if calibration.get('calibrated'):
        print(f"\nUsing calibration: {CALIBRATION_FILE}")
        print(f"  ADC Range: {calibration['adc_min']} - {calibration['adc_max']}")
        print(f"  Max Volume: {calibration['max_volume']}")
    else:
        print("\n⚠️  No calibration found. Using defaults.")
        print("  Run with --calibrate to calibrate.")
    
    print("\nMonitoring ADC values. Press Ctrl+C to exit.\n")
    
    try:
        while True:
            result = read_adc_value()
            if result:
                adc_value, voltage = result
                volume = map_adc_to_volume(adc_value, calibration)
                bar_len = int(volume / MAX_VOLUME_CAP * 40)
                bar = '█' * bar_len + '░' * (40 - bar_len)
                
                # Color based on volume level
                if volume < 0.1:
                    status = "MUTE" if volume < 0.01 else "LOW "
                elif volume < 0.35:
                    status = "MED "
                else:
                    status = "HIGH"
                
                print(f"\r{status} | ADC: {adc_value:5d} | {voltage:.2f}V | Vol: {volume:.2f} [{bar}]", end='', flush=True)
            else:
                print("\r✗ ADC read failed", end='', flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n\nMonitor stopped.")
    
    return 0


def run_verify():
    """Verify calibration is correct."""
    print("=" * 60)
    print("ADC Calibration Verification")
    print("=" * 60)
    
    if not ADC_AVAILABLE:
        print("\n✗ ADC library not available.")
        return 1
    
    # Load calibration
    calibration = load_calibration()
    
    print(f"\nCalibration file: {CALIBRATION_FILE}")
    
    if not calibration.get('calibrated'):
        print("\n✗ No calibration found!")
        print("  Run with --calibrate first.")
        return 1
    
    print(f"\nCalibration data:")
    print(f"  Date: {calibration.get('calibration_date', 'unknown')}")
    print(f"  ADC Min: {calibration['adc_min']}")
    print(f"  ADC Max: {calibration['adc_max']}")
    print(f"  Max Volume: {calibration['max_volume']}")
    
    # Verify by testing positions
    print("\n" + "-" * 60)
    print("Verification Tests")
    print("-" * 60)
    
    tests = [
        ("MINIMUM (fully CCW)", 0.0, 0.05),
        ("MIDDLE (center)", 0.30, 0.40),
        ("MAXIMUM (fully CW)", 0.65, 0.75),
    ]
    
    passed = 0
    for test_name, expected_min, expected_max in tests:
        print(f"\nTest: {test_name}")
        print(f"  Expected volume: {expected_min:.2f} - {expected_max:.2f}")
        input(f"  Turn knob to {test_name} position and press Enter...")
        
        readings = []
        for _ in range(20):
            result = read_adc_value()
            if result:
                volume = map_adc_to_volume(result[0], calibration)
                readings.append(volume)
            time.sleep(0.05)
        
        if readings:
            avg_volume = sum(readings) / len(readings)
            print(f"  Measured volume: {avg_volume:.2f}")
            
            if expected_min <= avg_volume <= expected_max:
                print(f"  ✓ PASS")
                passed += 1
            else:
                print(f"  ✗ FAIL (expected {expected_min:.2f} - {expected_max:.2f})")
        else:
            print(f"  ✗ FAIL (could not read ADC)")
    
    print("\n" + "-" * 60)
    print(f"Results: {passed}/{len(tests)} tests passed")
    print("-" * 60)
    
    return 0 if passed == len(tests) else 1


def main():
    parser = argparse.ArgumentParser(
        description='R2D2 ADC Calibration and Test Script',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  Detect ADC:
    python3 test_adc_calibration.py --detect
    
  Calibrate potentiometer:
    python3 test_adc_calibration.py --calibrate
    
  Monitor live values:
    python3 test_adc_calibration.py --monitor
    
  Verify calibration:
    python3 test_adc_calibration.py --verify
"""
    )
    
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--detect', action='store_true',
                       help='Detect if ADS1115 ADC is connected')
    group.add_argument('--calibrate', action='store_true',
                       help='Run interactive calibration')
    group.add_argument('--monitor', action='store_true',
                       help='Monitor ADC values continuously')
    group.add_argument('--verify', action='store_true',
                       help='Verify calibration is correct')
    
    args = parser.parse_args()
    
    if args.detect:
        return run_detect()
    elif args.calibrate:
        return run_calibrate()
    elif args.monitor:
        return run_monitor()
    elif args.verify:
        return run_verify()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())

