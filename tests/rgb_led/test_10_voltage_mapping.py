#!/usr/bin/env python3
"""
Complete RGB LED Voltage Mapping Test
Tests all 3 wires on different voltages to determine LED type

Strategy: Test each wire on 5V, 3.3V, and GND to map out behavior

Author: R2D2 Development Team
Date: December 18, 2025
"""

print("="*70)
print("Complete RGB LED Voltage Mapping Test")
print("="*70)

print("\n🎯 Goal: Determine which wire controls which color")
print("\n📋 Available voltages on Jetson 40-pin header:")
print("   - 5V: Pin 2, Pin 4")
print("   - 3.3V: Pin 1, Pin 17")
print("   - GND: Pin 6, Pin 9, Pin 14, Pin 20, Pin 25, Pin 30, Pin 34, Pin 39")

print("\n" + "="*70)
print("Test Matrix")
print("="*70)

tests = [
    # (Red, Black, Blue, Expected Result)
    ("5V Pin 2", "GND Pin 6", "Disconnected", "Baseline: What glows?"),
    ("5V Pin 2", "GND Pin 6", "3.3V Pin 17", "Your discovery: Red + Blue glow"),
    ("5V Pin 2", "3.3V Pin 17", "GND Pin 6", "Swap: Black=3.3V, Blue=GND"),
    ("3.3V Pin 17", "GND Pin 6", "5V Pin 2", "Swap: Red=3.3V, Blue=5V"),
    ("GND Pin 6", "5V Pin 2", "3.3V Pin 17", "Reverse: Red=GND, Black=5V"),
    ("3.3V Pin 17", "GND Pin 6", "Disconnected", "Just Red+Black"),
    ("5V Pin 2", "Disconnected", "3.3V Pin 17", "Just Red+Blue"),
    ("Disconnected", "GND Pin 6", "3.3V Pin 17", "Just Black+Blue"),
]

print("\nWe'll test 8 different wiring combinations.")
print("For each, you'll tell me what colors you see.\n")

input("Press ENTER to start testing...")

results = []

for i, (red, black, blue, description) in enumerate(tests, 1):
    print("\n" + "="*70)
    print(f"Test {i}/8: {description}")
    print("="*70)
    print(f"   RED wire   → {red}")
    print(f"   BLACK wire → {black}")
    print(f"   BLUE wire  → {blue}")
    
    input("\n   Connect wires as shown above, then press ENTER...")
    
    print("\n   What do you see on the LED panel?")
    print("   Options:")
    print("     1) All off / nothing")
    print("     2) Red only")
    print("     3) Green only")
    print("     4) Blue only")
    print("     5) Red + Green (yellow/orange)")
    print("     6) Red + Blue (magenta/purple)")
    print("     7) Green + Blue (cyan)")
    print("     8) All colors (white)")
    print("     9) Other (you'll describe)")
    
    response = input("\n   Your answer (1-9): ").strip()
    
    color_map = {
        '1': 'OFF',
        '2': 'RED',
        '3': 'GREEN',
        '4': 'BLUE',
        '5': 'RED+GREEN',
        '6': 'RED+BLUE',
        '7': 'GREEN+BLUE',
        '8': 'WHITE',
        '9': 'OTHER'
    }
    
    color = color_map.get(response, 'UNKNOWN')
    
    if response == '9':
        color = input("   Please describe what you see: ").strip()
    
    results.append({
        'test': i,
        'description': description,
        'red': red,
        'black': black,
        'blue': blue,
        'observed': color
    })
    
    print(f"\n   ✅ Recorded: {color}")

print("\n" + "="*70)
print("Test Results Summary")
print("="*70)

for r in results:
    print(f"\nTest {r['test']}: {r['description']}")
    print(f"  Red={r['red']}, Black={r['black']}, Blue={r['blue']}")
    print(f"  Result: {r['observed']}")

print("\n" + "="*70)
print("Analysis")
print("="*70)

print("\n📊 Based on these results, we can determine:")
print("1. Which wire connects to which LED color channel")
print("2. Whether it's common anode (shares 5V) or common cathode (shares GND)")
print("3. What voltage each color needs")

print("\n💡 Pattern to look for:")
print("   - If a wire on 3.3V lights a color → that wire controls that color")
print("   - If removing a wire turns off a color → that wire is power/ground")
print("   - Common anode: Colors need GND to light (one wire is +V)")
print("   - Common cathode: Colors need +V to light (one wire is GND)")

print("\n" + "="*70)
print("Next Steps")
print("="*70)
print("\nOnce we understand the wiring, we'll:")
print("1. Map RED/BLACK/BLUE wires to R/G/B LED channels")
print("2. Create GPIO control script for RED/GREEN/BLUE status")
print("3. Integrate into R2D2 status_led_node")
print("\n✅ Test complete!")

