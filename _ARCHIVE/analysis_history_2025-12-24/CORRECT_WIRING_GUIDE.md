# PAM8403 Audio Amplifier - Correct Wiring Guide

**Date:** December 8, 2025  
**Status:** Hardware wiring CORRECTED and verified

## Correct Wiring Diagram

```
Jetson Nano (CVB-RT)
    ↓
J511 Header (10-pin audio panel)
    ↓
    Pin 2: AGND (Audio Ground) ──→ PAM8403 GND
    Pin 5: HPO_R (RIGHT audio output) ──→ PAM8403 RIN (Right Input)
    
    
PAM8403 Amplifier Module (2x 3W @ 8Ω)
    ├─ Pin 1: +5V (Power) ──→ 5V Power Supply
    ├─ Pin 2: GND ──→ Ground to Jetson J511 Pin 2 (AGND)
    ├─ Pin 3: RIN (Right Input) ──→ Jetson J511 Pin 5 (HPO_R)
    ├─ Outputs:
    │   ├─ R+ (Right positive) ──→ Speaker Red wire
    │   └─ R− (Right negative) ──→ Speaker Black wire
    └─ [Left channel (LIN) not used in Phase 1]


Speaker (8Ω)
    ├─ Red wire ──→ PAM8403 R+ (Right positive)
    └─ Black wire ──→ PAM8403 R− (Right negative)
```

## Hardware Connections Checklist

### Power Supply
- [ ] +5V connected to PAM8403 Pin 1 (labeled "+5V" or "VCC")
- [ ] GND connected to PAM8403 Pin 2
- [ ] Ground also connected to Jetson J511 Pin 2 (AGND)

### Audio Input Signal
- [ ] Jetson J511 Pin 5 (HPO_R - RIGHT channel) → PAM8403 RIN (Right input)
- [ ] Jetson J511 Pin 2 (AGND) → PAM8403 GND (same ground reference)

### Speaker Output
- [ ] PAM8403 R+ (Right positive) → Speaker red wire (solder joint shiny)
- [ ] PAM8403 R− (Right negative) → Speaker black wire (solder joint shiny)

### Solder Joint Quality (visual inspection)
- [ ] All joints are shiny and smooth (not dull/grainy)
- [ ] All wire insulation is fully covered by solder
- [ ] No cold joints (grainy appearance = poor connection)
- [ ] All wires fully inserted into connection points

## Testing Procedure

### 1. Visual Inspection First
```bash
# Inspect these with a magnifying glass:
# - All 5 solder joints on PAM8403 module
# - Speaker wire connections at R+ and R−
# - J511 header connections (pins 2 and 9)
# Look for: shiny joints = good, dull/grainy = bad
```

### 2. Run Audio Test
```bash
cd /home/severin/dev/r2d2
python3 test_audio_fixed.py

# Expected output:
# ✓ Beep played successfully!
# If you HEAR a beep from the speaker, wiring is correct!
```

### 3. Troubleshooting with Multimeter

**Continuity Test (beep mode):**
```bash
Multimeter set to Continuity (Ω with speaker symbol)

Test 1: J511 Pin 2 → PAM8403 GND
  → Should BEEP (connected)

Test 2: J511 Pin 9 → PAM8403 RIN  
  → Should BEEP (connected)

Test 3: PAM8403 R+ → Speaker red wire
  → Should BEEP (connected)

Test 4: PAM8403 R− → Speaker black wire
  → Should BEEP (connected)
```

**Voltage Test (while audio plays):**
```bash
# Start audio playback in one terminal:
python3 test_audio_fixed.py

# In another terminal, measure with multimeter (AC voltage):
# Test 1: Jetson J511 Pin 9 to Pin 2
#   → Should show 0.3-1.0V AC (audio signal)

# Test 2: PAM8403 RIN to GND
#   → Should show same voltage (0.3-1.0V AC)

# Test 3: PAM8403 R+ to R−
#   → Should show 1-3V AC (amplified ~10x)
```

## Common Wiring Mistakes (FIXED in this version)

❌ **WRONG (Previous):**
- Connected to J511 Pin 9 (HPO_L - LEFT channel instead of RIGHT!)
- RIN connected to Power GND (destroys codec output stage!)
- Mixed up left (L) and right (R) channels
- GND not properly connected

✓ **CORRECT (Current):**
- Connected to J511 Pin 5 (HPO_R - RIGHT audio output)
- RIN connected to J511 Pin 5 (HPO_R - right audio output)
- GND connected to J511 Pin 2 (AGND - analog ground)
- Right channel (R+/R−) only (left channel disabled)
- Power (5V) and GND properly connected

## Expected Behavior After Fix

**When you run:** `python3 test_audio_fixed.py`

✓ You should HEAR:
- A 1kHz beep tone (like a computer beep)
- Coming from the speaker connected to R+ and R−
- Duration: 1 second
- Frequency: 1000 Hz (middle C on piano is ~261 Hz, this is higher)

✗ If you DON'T hear it:
1. Check solder joints with magnifying glass (look for dull joints)
2. Reflow bad joints (heat + add fresh solder)
3. Run multimeter continuity tests
4. Check PAM8403 module power LED (should be on)

## Next Steps

1. **Verify hardware visually** - inspect all solder joints
2. **Run test:** `python3 test_audio_fixed.py`
3. **Measure voltages** if still no sound
4. **Reflow solder joints** if needed using the checklist above

**Status:** Ready for audio testing with corrected wiring! ✓
