#!/usr/bin/env python3
"""
Simple test: Simulates face recognition detection and triggers beeps
No ROS 2 infrastructure - just direct audio testing
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
print("=" * 50)
print("🎬 SIMPLE FACE RECOGNITION TEST")
print("=" * 50)
print("")

print("Step 1: No face detected (unknown)")
print("         Waiting...")
time.sleep(2)

print("")
print("Step 2: ⭐ YOUR FACE DETECTED (severin)")
print("         >>> EXPECTING BEEP NOW <<<")
play_beep()
time.sleep(1)

print("")
print("Step 3: Still recognized (no beep expected)")
print("         Waiting...")
time.sleep(2)

print("")
print("Step 4: Face lost (unknown)")
print("         Waiting...")
time.sleep(2)

print("")
print("Step 5: ⭐ YOUR FACE DETECTED AGAIN (severin)")
print("         >>> EXPECTING BEEP NOW <<<")
play_beep()
time.sleep(1)

print("")
print("=" * 50)
print("✅ TEST COMPLETE")
print("   You should have heard 2 beeps (steps 2 and 5)")
print("=" * 50)
print("")
