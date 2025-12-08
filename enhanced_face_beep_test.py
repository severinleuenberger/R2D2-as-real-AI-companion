#!/usr/bin/env python3
"""
Enhanced Face Recognition Test with Loss Detection

Simulates face recognition scenarios to demonstrate:
1. Single beep on recognition (transition from unknown → severin)
2. Jitter tolerance: Brief losses don't trigger loss notification
3. Double beep on confirmed loss (continuous absence > 5 seconds)
4. Status persistence during brief gaps
"""

import subprocess
import time
from pathlib import Path


def play_beep(frequency=1000, duration=0.5, volume=0.7):
    """Play a beep directly"""
    cmd = [
        'python3',
        str(Path.home() / 'dev' / 'r2d2' / 'audio_beep.py'),
        '--frequency', str(frequency),
        '--duration', str(duration),
        '--volume', str(volume),
    ]
    try:
        subprocess.run(cmd, timeout=duration + 1, capture_output=True)
        return True
    except Exception as e:
        print(f"Error: {e}")
        return False


print("")
print("=" * 70)
print("🎬 ENHANCED FACE RECOGNITION TEST")
print("=" * 70)
print("")

print("TEST SCENARIO: Enhanced recognition state management")
print("=" * 70)
print("")

print("PHASE 1: Initial Recognition")
print("-" * 70)
print("Step 1: No face detected (unknown)")
print("         Status: UNKNOWN")
print("         ⏱️  Waiting 2 seconds...")
time.sleep(2)

print("")
print("Step 2: ⭐ YOUR FACE DETECTED (severin)")
print("         Status: RECOGNIZED")
print("         >>> EXPECTING: 1 BEEP (recognition alert) <<<")
play_beep(frequency=1000, duration=0.5, volume=0.7)
time.sleep(2)

print("")
print("PHASE 2: Jitter Tolerance (Brief Gap)")
print("-" * 70)
print("Step 3: Brief interruption (lost for 2 seconds)")
print("         Status: STILL RECOGNIZED (jitter tolerance active)")
print("         ⏱️  Waiting 2 seconds (no beep - within 5s jitter window)...")
time.sleep(2)

print("")
print("Step 4: Face re-appears (still recognized)")
print("         Status: RECOGNIZED")
print("         (No beep - same recognition state, within jitter window)")
print("         ⏱️  Waiting 1 second...")
time.sleep(1)

print("")
print("PHASE 3: Confirmed Loss Detection")
print("-" * 70)
print("Step 5: Continuous loss begins (lost for 6+ seconds)")
print("         Status: RECOGNIZED → Monitoring for loss confirmation...")
print("         ⏱️  Waiting 3 seconds...")
time.sleep(3)

print("")
print("Step 6: Loss confirmation triggered (> 5s continuous absence)")
print("         Status: RECOGNIZED → LOST (CONFIRMED)")
print("         >>> EXPECTING: 2 BEEPS (loss alert) <<<")
print("         Playing double beep at 500 Hz (loss alert frequency)...")
play_beep(frequency=500, duration=0.3, volume=0.7)
time.sleep(0.2)
play_beep(frequency=500, duration=0.3, volume=0.7)
time.sleep(2)

print("")
print("PHASE 4: Re-Recognition After Loss")
print("-" * 70)
print("Step 7: ⭐ YOUR FACE DETECTED AGAIN (severin)")
print("         Status: UNKNOWN → RECOGNIZED")
print("         >>> EXPECTING: 1 BEEP (recognition alert) <<<")
play_beep(frequency=1000, duration=0.5, volume=0.7)
time.sleep(2)

print("")
print("Step 8: Face remains visible")
print("         Status: RECOGNIZED")
print("         (No beep - already in recognized state)")
print("         ⏱️  Waiting 1 second...")
time.sleep(1)

print("")
print("=" * 70)
print("✅ TEST COMPLETE")
print("=" * 70)
print("")
print("Expected beeps:")
print("  1. Step 2: 1 beep @ 1000 Hz (recognition)")
print("  2. Step 6: 2 beeps @ 500 Hz (loss alert)")
print("  3. Step 7: 1 beep @ 1000 Hz (re-recognition)")
print("  Total: 4 beeps (1 + 2 + 1)")
print("")
print("Key behaviors demonstrated:")
print("  ✓ Single beep on transition to recognized")
print("  ✓ Jitter tolerance: Brief gaps don't trigger loss")
print("  ✓ Loss confirmation: Must be lost > 5 seconds")
print("  ✓ Double beep on confirmed loss")
print("  ✓ Re-recognition after loss triggers single beep")
print("")
