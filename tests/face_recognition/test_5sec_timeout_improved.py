#!/usr/bin/env python3
"""
Test script to verify 5-second timeout and display steadiness improvements.

This script monitors the JSON status file and tracks:
1. Recognition detection time
2. Timeout duration 
3. Display stability (no flickering)
4. Re-detection speed
"""

import json
import time
from datetime import datetime
from pathlib import Path

def read_status():
    """Read current status from JSON file."""
    status_file = Path.home() / '.r2d2_face_recognition_status.json'
    try:
        with open(status_file, 'r') as f:
            return json.load(f)
    except:
        return None

def test_5sec_timeout():
    """Test the 5-second timeout mechanism."""
    print("="*70)
    print("TESTING 5-SECOND TIMEOUT & DISPLAY STEADINESS")
    print("="*70)
    print()
    print("INSTRUCTIONS:")
    print("1. Position your face in front of camera and keep it steady")
    print("2. After ~3 seconds, step completely out of frame")
    print("3. Wait 8-10 seconds (watching the display)")
    print("4. Step back in front of camera")
    print()
    print("EXPECTED BEHAVIOR:")
    print("✓ Recognition within 1-2 seconds of entering frame")
    print("✓ Status stays 'RECOGNIZED' for full 5 seconds after leaving")
    print("✓ Display shows smooth transition, no flickering")
    print("✓ Quick re-detection when you reappear")
    print()
    print("Press ENTER to start test (show your face to camera now):")
    input()
    
    print()
    print("-"*70)
    print("LIVE MONITORING (press Ctrl+C to stop)")
    print("-"*70)
    print()
    
    last_status = None
    last_timestamp = None
    start_time = time.time()
    status_changes = []
    
    try:
        while True:
            status = read_status()
            
            if status:
                current_person = status.get('recognized_person')
                timestamp = status.get('timestamp')
                frame_count = status.get('frame_count')
                detection_count = status.get('detection_count', 0)
                
                elapsed = time.time() - start_time
                
                # Check if status changed
                if current_person != last_status:
                    status_changes.append({
                        'time': elapsed,
                        'status': current_person,
                        'timestamp': timestamp,
                    })
                    
                    # Display status change
                    if current_person:
                        marker = f"✅ DETECTED"
                        print(f"[{elapsed:6.1f}s] {marker}: {current_person.upper():15} (detection_count={detection_count})")
                    else:
                        marker = f"❌ LOST"
                        print(f"[{elapsed:6.1f}s] {marker}: Recognition ended                     ")
                    
                    last_status = current_person
                    last_timestamp = timestamp
                else:
                    # Status unchanged - only show persistence info
                    if last_status and current_person:
                        # Calculating how long recognition has been active
                        pass
                
                time.sleep(0.2)
            else:
                print("Waiting for service...")
                time.sleep(1)
    
    except KeyboardInterrupt:
        print()
        print()
        print("-"*70)
        print("TEST RESULTS")
        print("-"*70)
        print()
        
        if len(status_changes) >= 2:
            print("Status transitions detected:")
            for i, change in enumerate(status_changes, 1):
                marker = "✅ DETECT" if change['status'] else "❌ LOSS"
                print(f"  {i}. T={change['time']:6.1f}s: {marker}")
            
            print()
            print("Analysis:")
            
            # Find detection and loss times
            for i in range(len(status_changes)-1):
                if status_changes[i]['status'] and not status_changes[i+1]['status']:
                    detect_time = status_changes[i]['time']
                    loss_time = status_changes[i+1]['time']
                    duration = loss_time - detect_time
                    
                    print(f"  Detection duration: {duration:.1f} seconds")
                    
                    if abs(duration - 5.0) < 1.0:
                        print(f"  ✅ TIMEOUT WORKING: ~5 second persistence confirmed!")
                    else:
                        print(f"  ⚠️  TIMEOUT UNUSUAL: Expected ~5s, got {duration:.1f}s")
            
            # Check for rapid flickering
            rapid_changes = 0
            for i in range(len(status_changes)-1):
                time_diff = status_changes[i+1]['time'] - status_changes[i]['time']
                if time_diff < 0.5:
                    rapid_changes += 1
            
            print()
            if rapid_changes == 0:
                print(f"  ✅ DISPLAY STABILITY: No flickering detected!")
            else:
                print(f"  ⚠️  DISPLAY UNSTABLE: {rapid_changes} rapid changes detected")
        
        else:
            print("Not enough data. Make sure to:")
            print("1. Show face to camera")
            print("2. Step away completely")
            print("3. Let service detect and timeout")
            print("4. Step back in front")

if __name__ == "__main__":
    test_5sec_timeout()
