#!/usr/bin/env python3
"""Test both left and right audio channels"""

import sys
import time
sys.path.insert(0, '/home/severin/dev/r2d2')
from audio_beep import play_beep

print("=" * 70)
print("R2D2 AUDIO CHANNEL TEST")
print("=" * 70)
print()

print("Your speaker is connected to the RIGHT channel (HPO_R)")
print("We need to route audio to the RIGHT channel instead of LEFT")
print()

# Test: Play on device hw:1,0 which should output both channels
print("[TEST 1] Playing 1kHz beep on hw:1,0 (both channels)")
print("         Listening for sound on RIGHT speaker...")
print()

if play_beep(frequency=1000.0, duration=2.0, volume=0.5, device='hw:1,0'):
    print("✓ Beep sent to device")
    response = input("\nDid you hear a beep from the RIGHT speaker? (yes/no): ").strip().lower()
    if response in ['yes', 'y']:
        print("\n✅ SUCCESS! The audio is working on the RIGHT channel!")
        print("\nNext: We need to configure the mixer to use the RIGHT channel")
        sys.exit(0)
    else:
        print("\n✗ Still no sound")
        print("\nLet me try to route audio to the RIGHT channel explicitly...")
else:
    print("✗ Error playing beep")
    sys.exit(1)
