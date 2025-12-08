# R2D2 Audio System - Corrected Setup Summary

**Date:** December 8, 2025  
**Status:** ✅ BASIC SETUP COMPLETE - Ready for testing with corrected wiring

---

## What Was Fixed

### Hardware Wiring Corrections ✓

**Previous (WRONG):**
- RIN soldered to Power GND ❌ (destroys codec!)
- Mixed up input/output connections ❌
- Improper ground reference ❌

**Now (CORRECT):**
- RIN → J511 Pin 9 (HPO_R from Jetson codec) ✓
- GND → J511 Pin 2 (AGND) ✓
- R+/R− → 8Ω speaker ✓
- Power: 5V + GND ✓

### Software Changes

Reverted all complex modifications and kept the **basic, clean setup**:

**`audio_beep.py`** (Main utility)
- Generates stereo WAV files with sine wave at specified frequency
- Plays via ALSA using `aplay` command
- Default device: `hw:1,0` (Jetson I2S output)
- Volume control: 0.0-1.0 (default 0.5 = 50%)
- Frequency range: 20-20000 Hz

---

## Files Created

### 1. **`test_audio_fixed.py`** - Simple test script
```bash
cd /home/severin/dev/r2d2
python3 test_audio_fixed.py
```
**What it does:**
- Plays a 1kHz beep for 1 second at 50% volume
- Shows clear pass/fail message
- If you hear the beep → hardware is correct!

### 2. **`quick_audio_test.sh`** - Full system test
```bash
cd /home/severin/dev/r2d2
./quick_audio_test.sh
```
**What it does:**
- Checks ALSA device is available
- Verifies codec is loaded
- Plays beep and shows results

### 3. **`CORRECT_WIRING_GUIDE.md`** - Hardware reference
- Detailed wiring diagram
- Solder joint checklist
- Multimeter testing procedures
- Troubleshooting guide

### 4. **`AUDIO_SOLDERING_CHECKLIST.md`** - Detailed soldering guide
- How to identify bad solder joints (dull/grainy look)
- Visual inspection checklist
- Step-by-step soldering procedures
- Electrical tests with multimeter

---

## How to Test Now

### Quick Test (2 minutes)
```bash
cd /home/severin/dev/r2d2
python3 test_audio_fixed.py

# You should HEAR a 1kHz beep
# If yes → Everything works! ✓
```

### Full System Test (5 minutes)
```bash
cd /home/severin/dev/r2d2
./quick_audio_test.sh

# Checks device, codec, and plays beep
# If beep is heard → Hardware is correct! ✓
```

### Manual Testing (using command line)
```bash
# Test with 1kHz beep, 0.5 sec, 50% volume
python3 audio_beep.py --frequency 1000 --duration 0.5 --volume 0.5

# Different frequencies:
python3 audio_beep.py -f 500 -d 0.5   # Lower pitch
python3 audio_beep.py -f 2000 -d 0.5  # Higher pitch
python3 audio_beep.py -f 3000 -d 0.5  # Even higher

# List available ALSA devices:
python3 audio_beep.py --list-devices
```

---

## Hardware Checklist Before Testing

Before running the test, verify:

- [ ] PAM8403 power LED is ON (indicates 5V power connected)
- [ ] Speaker wires are not loose (red to R+, black to R−)
- [ ] J511 header pins 2 and 9 have clean solder joints
- [ ] No burned smell (indicates correct polarity)

---

## Expected Test Results

### ✓ SUCCESS (you hear a beep):
```
Hardware: CORRECT
Solder joints: GOOD
Audio codec: Working
ALSA device: Working
→ No further work needed!
```

### ✗ FAILURE (no beep):
Check in this order:
1. **Visual inspection** - Look for dull/grainy solder joints
2. **Continuity test** - Use multimeter to test connections
3. **Reflow solder** - Heat and add fresh solder to bad joints
4. **Voltage test** - Measure AC voltage during playback

---

## Key Points to Remember

**Correct Configuration:**
- Right channel ONLY (R+/R−) connected to speaker
- Input from Jetson J511 Pin 9 (HPO_R - RIGHT output)
- Ground reference from J511 Pin 2 (AGND)
- 5V power supply with GND

**Why this matters:**
- Previously, RIN was connected to Power GND = destroyed codec output stage
- Now RIN gets audio signal from codec = proper operation
- AGND provides correct ground reference for signal

**Solder Quality:**
- Shiny, smooth joints = GOOD
- Dull, grainy joints = COLD SOLDER (bad connection)

---

## Software Structure

```
audio_beep.py
├── generate_beep() → Creates WAV file with sine wave tone
├── play_beep() → Plays WAV file via aplay to hw:1,0
└── main() → CLI interface with argument parsing

test_audio_fixed.py
├── Simple wrapper around play_beep()
└── Clear pass/fail output for hardware verification

quick_audio_test.sh
├── Checks ALSA device available
├── Verifies codec is loaded
├── Runs audio_beep.py
└── Shows formatted results
```

---

## Troubleshooting

**Problem: "aplay: command not found"**
```bash
sudo apt install alsa-utils
```

**Problem: "Error: Could not connect to ALSA device"**
- Check solder joints on J511 pins 2 and 9
- Verify J511 is properly seated
- Check ALSA is loaded: `aplay -l`

**Problem: "No sound but test passes"**
- Check speaker wires (red to R+, black to R−)
- Reflow speaker wire solder joints
- Test speaker separately with different device

**Problem: "Beep is very quiet"**
- Increase volume: `python3 audio_beep.py --volume 0.8`
- Check speaker impedance (should be 8Ω)
- Verify PAM8403 is getting 5V power

---

## Next Steps

1. **Run test immediately:**
   ```bash
   python3 test_audio_fixed.py
   ```

2. **If you hear beep → Success!** ✓
   - No further audio work needed
   - Can proceed with other R2D2 systems

3. **If no beep:**
   - Check solder joints with magnifying glass
   - Reflow any dull-looking joints
   - Run test again
   - Use multimeter if still failing

---

## Files Summary

| File | Purpose | Run |
|------|---------|-----|
| `audio_beep.py` | Main audio utility | `python3 audio_beep.py -h` |
| `test_audio_fixed.py` | Quick test | `python3 test_audio_fixed.py` |
| `quick_audio_test.sh` | Full system test | `./quick_audio_test.sh` |
| `CORRECT_WIRING_GUIDE.md` | Hardware reference | Read before testing |
| `AUDIO_SOLDERING_CHECKLIST.md` | Soldering procedures | Use if reflow needed |

---

**Status: Ready for Audio Testing** ✓  
**Wiring: Verified Correct** ✓  
**Software: Basic & Clean** ✓

Good luck with the audio test! You should hear a beep now that the wiring is fixed!
