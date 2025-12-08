#!/usr/bin/env python3
"""
Simple audio test for R2D2 with CORRECTED PAM8403 wiring

Correct Hardware Setup:
- Right channel (R+/R−): Connected to 8Ω speaker
- RIN: Connected to Jetson J511 Pin 5 (HPO_R)
- GND: Connected to Jetson J511 Pin 2 (AGND)
- Power: 5V + GND

Run: python3 test_audio_fixed.py
Expected: Hear 1kHz beep from the 8Ω speaker
"""

import sys
import time
sys.path.insert(0, '/home/severin/dev/r2d2')

from audio_beep import play_beep

def main():
    print("=" * 70)
    print("R2D2 AUDIO TEST - PAM8403 RIGHT CHANNEL (CORRECTED WIRING)")
    print("=" * 70)
    print()
    print("Hardware Setup:")
    print("  - Right channel (R+/R−) → 8Ω speaker")
    print("  - RIN → J511 Pin 5 (HPO_R from Jetson codec)")
    print("  - GND → J511 Pin 2 (AGND)")
    print("  - Power: 5V + GND")
    print()
    print("Playing 1kHz beep at 50% volume for 1 second...")
    print()
    
    # Play a 1kHz beep - should come from the RIGHT channel
    success = play_beep(
        frequency=1000.0,
        duration=1.0,
        volume=0.5,
        device="hw:1,0"  # Jetson I2S output
    )
    
    if success:
        print("✓ Beep played successfully!")
        print()
        print("If you HEARD a beep from the speaker:")
        print("  → Hardware wiring is CORRECT ✓")
        print("  → Audio system is working ✓")
        return 0
    else:
        print("✗ Failed to play beep")
        print()
        print("Possible issues:")
        print("  1. Check PAM8403 solder joints (especially RIN and GND)")
        print("  2. Check speaker wire solder connections")
        print("  3. Verify 5V power is connected to PAM8403")
        print("  4. Check ALSA device: aplay -l")
        return 1

if __name__ == "__main__":
    sys.exit(main())
