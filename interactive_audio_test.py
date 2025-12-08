#!/usr/bin/env python3
"""
Interactive audio test - plays beeps and waits for user confirmation
"""

import sys
import time
sys.path.insert(0, '/home/severin/dev/r2d2')

from audio_beep import play_beep

def wait_for_yes():
    """Wait for user to confirm 'yes'"""
    while True:
        response = input("Did you hear a beep? (yes/no): ").strip().lower()
        if response in ['yes', 'y']:
            return True
        elif response in ['no', 'n']:
            return False
        else:
            print("Please answer 'yes' or 'no'")

def main():
    print("=" * 70)
    print("R2D2 INTERACTIVE AUDIO TEST")
    print("=" * 70)
    print()
    print("This test will play beeps and ask if you heard them.")
    print("Listen carefully to your speaker!")
    print()
    
    tests_passed = 0
    tests_failed = 0
    
    # Test 1: 1kHz at 50% volume
    print("[TEST 1] Playing 1kHz beep at 50% volume for 1 second...")
    print("         (Should sound like a steady mid-pitch tone)")
    time.sleep(1)
    
    if play_beep(frequency=1000.0, duration=1.0, volume=0.5, device='hw:1,0'):
        print("         Beep sent to speaker")
        if wait_for_yes():
            print("         ✓ TEST 1 PASSED")
            tests_passed += 1
        else:
            print("         ✗ TEST 1 FAILED - No audio heard")
            tests_failed += 1
    else:
        print("         ✗ Error playing beep")
        tests_failed += 1
    
    time.sleep(2)
    
    # Test 2: Lower frequency (500Hz)
    print("\n[TEST 2] Playing 500Hz beep at 50% volume for 1 second...")
    print("         (Should sound LOWER pitched than Test 1)")
    time.sleep(1)
    
    if play_beep(frequency=500.0, duration=1.0, volume=0.5, device='hw:1,0'):
        print("         Beep sent to speaker")
        if wait_for_yes():
            print("         ✓ TEST 2 PASSED")
            tests_passed += 1
        else:
            print("         ✗ TEST 2 FAILED - No audio heard")
            tests_failed += 1
    else:
        print("         ✗ Error playing beep")
        tests_failed += 1
    
    time.sleep(2)
    
    # Test 3: Higher frequency (2000Hz)
    print("\n[TEST 3] Playing 2000Hz beep at 50% volume for 1 second...")
    print("         (Should sound HIGHER pitched than Test 1)")
    time.sleep(1)
    
    if play_beep(frequency=2000.0, duration=1.0, volume=0.5, device='hw:1,0'):
        print("         Beep sent to speaker")
        if wait_for_yes():
            print("         ✓ TEST 3 PASSED")
            tests_passed += 1
        else:
            print("         ✗ TEST 3 FAILED - No audio heard")
            tests_failed += 1
    else:
        print("         ✗ Error playing beep")
        tests_failed += 1
    
    time.sleep(2)
    
    # Test 4: Very quiet (10% volume)
    print("\n[TEST 4] Playing 1kHz beep at 10% volume for 1 second...")
    print("         (Should sound VERY QUIET)")
    time.sleep(1)
    
    if play_beep(frequency=1000.0, duration=1.0, volume=0.1, device='hw:1,0'):
        print("         Beep sent to speaker")
        if wait_for_yes():
            print("         ✓ TEST 4 PASSED")
            tests_passed += 1
        else:
            print("         ✗ TEST 4 FAILED - No audio heard")
            tests_failed += 1
    else:
        print("         ✗ Error playing beep")
        tests_failed += 1
    
    time.sleep(2)
    
    # Test 5: Very loud (90% volume)
    print("\n[TEST 5] Playing 1kHz beep at 90% volume for 1 second...")
    print("         (Should sound VERY LOUD)")
    time.sleep(1)
    
    if play_beep(frequency=1000.0, duration=1.0, volume=0.9, device='hw:1,0'):
        print("         Beep sent to speaker")
        if wait_for_yes():
            print("         ✓ TEST 5 PASSED")
            tests_passed += 1
        else:
            print("         ✗ TEST 5 FAILED - No audio heard")
            tests_failed += 1
    else:
        print("         ✗ Error playing beep")
        tests_failed += 1
    
    # Summary
    print("\n" + "=" * 70)
    print(f"RESULTS: {tests_passed} passed, {tests_failed} failed")
    print("=" * 70)
    
    if tests_failed == 0:
        print("\n✅ ALL TESTS PASSED - Audio system is working!")
        return 0
    elif tests_passed > 0:
        print(f"\n⚠ PARTIAL SUCCESS - {tests_passed}/{tests_passed + tests_failed} tests passed")
        print("\nChecklist for silent audio:")
        print("  1. Check PAM8403 power:")
        print("     $ multimeter on Jetson Pin 2 (should read ~5V)")
        print("     $ multimeter on PAM8403 +5V pin (should read ~5V)")
        print("  2. Check speaker connection:")
        print("     $ Verify PAM8403 L+ → speaker + (red wire)")
        print("     $ Verify PAM8403 L− → speaker − (black wire)")
        print("  3. Check J511 header connection:")
        print("     $ Verify Jetson J511 Pin 9 (HPO_L) → PAM8403 LIN")
        print("     $ Verify Jetson J511 Pin 2 (AGND) → PAM8403 GND")
        print("  4. Check for loose wires or solder bridges")
        return 1
    else:
        print(f"\n❌ NO TESTS PASSED - Audio system not working")
        print("\nChecklist:")
        print("  1. Verify ALSA device: aplay -l (should show Card 1)")
        print("  2. Check audio permissions: ls -la /dev/snd/")
        print("  3. Test direct ALSA: speaker-test -t sine -f 1000 -c 2 -D hw:1,0")
        return 1

if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(1)
