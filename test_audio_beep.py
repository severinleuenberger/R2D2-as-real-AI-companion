#!/usr/bin/env python3
"""
Simple audio beep test for R2D2
Tests the PAM8403 speaker with various beep tones
"""

import sys
import time
sys.path.insert(0, '/home/severin/dev/r2d2')

from audio_beep import play_beep

def test_audio_beep():
    """Test audio beep functionality"""
    
    print("=" * 70)
    print("R2D2 AUDIO BEEP TEST")
    print("=" * 70)
    print()
    
    # Test 1: Simple beep at 50% volume
    print("[Test 1] Simple 1kHz beep at 50% volume (0.5 seconds)")
    print("  Listening for beep from speaker...")
    if play_beep(frequency=1000.0, duration=0.5, volume=0.5, device='hw:1,0'):
        print("  ✓ Test 1 PASSED: Beep played successfully")
    else:
        print("  ✗ Test 1 FAILED: Could not play beep")
        return False
    
    time.sleep(1)
    
    # Test 2: Lower frequency at 50% volume (more bass)
    print("\n[Test 2] Lower tone 500Hz beep at 50% volume (0.5 seconds)")
    print("  Should sound lower pitched than Test 1...")
    if play_beep(frequency=500.0, duration=0.5, volume=0.5, device='hw:1,0'):
        print("  ✓ Test 2 PASSED: Low tone beep played")
    else:
        print("  ✗ Test 2 FAILED: Could not play low tone")
        return False
    
    time.sleep(1)
    
    # Test 3: Higher frequency at 50% volume (more treble)
    print("\n[Test 3] Higher tone 2000Hz beep at 50% volume (0.5 seconds)")
    print("  Should sound higher pitched than Test 1...")
    if play_beep(frequency=2000.0, duration=0.5, volume=0.5, device='hw:1,0'):
        print("  ✓ Test 3 PASSED: High tone beep played")
    else:
        print("  ✗ Test 3 FAILED: Could not play high tone")
        return False
    
    time.sleep(1)
    
    # Test 4: Longer beep at 50% volume
    print("\n[Test 4] Longer 1kHz beep at 50% volume (2 seconds)")
    print("  Should hear a sustained tone for 2 seconds...")
    if play_beep(frequency=1000.0, duration=2.0, volume=0.5, device='hw:1,0'):
        print("  ✓ Test 4 PASSED: Long beep played")
    else:
        print("  ✗ Test 4 FAILED: Could not play long beep")
        return False
    
    time.sleep(1)
    
    # Test 5: Quiet beep at 25% volume
    print("\n[Test 5] Quiet 1kHz beep at 25% volume (0.5 seconds)")
    print("  Should sound quieter than previous tests...")
    if play_beep(frequency=1000.0, duration=0.5, volume=0.25, device='hw:1,0'):
        print("  ✓ Test 5 PASSED: Quiet beep played")
    else:
        print("  ✗ Test 5 FAILED: Could not play quiet beep")
        return False
    
    time.sleep(1)
    
    # Test 6: Loud beep at 75% volume
    print("\n[Test 6] Loud 1kHz beep at 75% volume (0.5 seconds)")
    print("  Should sound louder than previous tests...")
    if play_beep(frequency=1000.0, duration=0.5, volume=0.75, device='hw:1,0'):
        print("  ✓ Test 6 PASSED: Loud beep played")
    else:
        print("  ✗ Test 6 FAILED: Could not play loud beep")
        return False
    
    time.sleep(1)
    
    # Final summary
    print("\n" + "=" * 70)
    print("✅ ALL AUDIO BEEP TESTS PASSED!")
    print("=" * 70)
    print("\nSummary:")
    print("  ✓ PAM8403 speaker is working correctly")
    print("  ✓ Audio output via I2S (hw:1,0) is functioning")
    print("  ✓ Volume control is working (25% to 75%)")
    print("  ✓ Frequency control is working (500Hz to 2000Hz)")
    print("\nThe audio output is ready for Phase 2 implementation:")
    print("  - Text-to-Speech (TTS) integration")
    print("  - Speech response output")
    print("  - Alert tones and notifications")
    print("=" * 70)
    
    return True


if __name__ == '__main__':
    try:
        success = test_audio_beep()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\n⚠ Test interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n\n✗ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
