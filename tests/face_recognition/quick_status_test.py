#!/usr/bin/env python3
"""
Simple visual test of IMMEDIATE recognition + 5-second loss persistence.

Run this while:
1. Standing in front of camera
2. Stepping away completely  
3. Stepping back in front

Watch the display to verify behavior.
"""

import json
from pathlib import Path
import time

def get_status():
    try:
        with open(Path.home() / '.r2d2_face_recognition_status.json') as f:
            return json.load(f)
    except:
        return None

def format_status(status):
    if not status:
        return "⏳ Waiting for status..."
    
    person = status.get('recognized_person')
    if person:
        return f"✅ RECOGNIZED: {person.upper()}"
    else:
        return "❌ No one recognized"

print("="*70)
print(" LIVE RECOGNITION STATUS")
print("="*70)
print()
print("Instructions:")
print("  1. Show your face → should see ✅ RECOGNIZED instantly")
print("  2. Step away → should see ✅ for ~5 seconds, then ❌")
print("  3. Step back → should see ✅ instantly again")
print()
print("-"*70)
print()

last_status = None
last_change = time.time()
persistence_start = None

try:
    while True:
        status = get_status()
        current_status = status.get('recognized_person') if status else None
        
        if current_status != last_status:
            # Status changed
            elapsed_since_change = time.time() - last_change
            
            # Show timing info if this is a loss
            if last_status and not current_status and persistence_start:
                persistence_duration = elapsed_since_change
                print(f"⏱️  Persistence lasted: {persistence_duration:.1f} seconds")
            
            # Show the new status
            display = format_status(status)
            print(display)
            
            # Track timing
            if current_status and not last_status:
                print("  (Recognized immediately ✓)")
            
            if not current_status and last_status:
                persistence_start = time.time()
                print("  (Starting 5-second persistence...)")
            
            last_status = current_status
            last_change = time.time()
        
        time.sleep(0.1)

except KeyboardInterrupt:
    print()
    print()
    print("-"*70)
    print("✅ Test completed. Status transitions verified above.")
    print()
