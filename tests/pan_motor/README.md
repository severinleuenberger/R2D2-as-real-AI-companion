
# Pan Motor Testing Guide

**Date:** January 6, 2026  
**Status:** Ready for Manual Testing  
**Hardware:** Pololu G2 High-Power Motor Driver 24v21 + DeAgostini DC Motor

---

## 📋 Pre-Testing Checklist

Before running any tests, verify:

- [ ] **Battery connected and charged** (14.8V 4S LiPo)
- [ ] **All wiring completed** as per `002_HARDWARE_REFERENCE.md` Section 4.5
- [ ] **Motor can rotate freely** (no obstructions, dome can spin)
- [ ] **SLP pin tied HIGH** (Pin 1 or 17 - 3.3V)
- [ ] **Common ground** between battery and Jetson verified
- [ ] **Encoder pull-down resistors** installed (4.7kΩ to GND on each channel)
- [ ] **Emergency stop ready** (know where Ctrl+C is!)

---

## 🧪 Test Scripts Available

### 1. Quick GPIO Test (Recommended First)
**File:** `test_quick_gpio.py`  
**Motor Movement:** ❌ No (safe test)  
**Duration:** ~30 seconds

**What it tests:**
- Can read all GPIO pins
- Encoder signals (manual rotation)
- Home sensor response

**Run:**
```bash
cd /home/severin/dev/r2d2/tests/pan_motor
sudo python3 test_quick_gpio.py
```

**Expected Results:**
- All pins should be readable
- Rotating motor shaft by hand should show encoder changes
- Moving dome to home position should trigger sensor

---

### 2. Full Motor Test
**File:** `test_pan_motor_basic.py`  
**Motor Movement:** ✅ Yes (motor will run)  
**Duration:** ~2 minutes

**What it tests:**
- GPIO state reading
- Encoder signal detection
- Home sensor monitoring
- Motor direction control (CW/CCW at 30% speed)
- Speed control (20%, 40%, 60%, 80%)
- Encoder accuracy at different speeds

**Run:**
```bash
cd /home/severin/dev/r2d2/tests/pan_motor
sudo python3 test_pan_motor_basic.py
```

**Expected Results:**
- Motor should rotate smoothly in both directions
- Encoder should count pulses during rotation
- Higher speeds should show proportionally more encoder pulses
- No unusual noises or overheating

---

## 🚀 Testing Sequence (Step by Step)

### Step 1: Verify Wiring (Visual Inspection)

**Power Side (Pololu G2 Driver):**
```
Battery +14.8V  →  Pololu "VM"
Battery GND     →  Pololu "GND"
Motor Red       →  Pololu "MOTOR A"
Motor Black     →  Pololu "MOTOR B"
```

**Logic Side (Pololu G2 to Jetson):**
```
Pololu "PWM"    →  Jetson Pin 13 (GPIO32/SPI2_SCK)
Pololu "DIR"    →  Jetson Pin 29 (CAN0_DIN/GPIO01)
Pololu "SLP"    →  Jetson Pin 1 or 17 (3.3V) - tie HIGH
Pololu "GND"    →  Jetson Pin 6/9/14/20/25/30/34/39 (GND)
```

**Encoder (to Jetson):**
```
Encoder VCC (Red)     →  Jetson Pin 2 or 4 (5V)
Encoder GND (Black)   →  Jetson Pin 6/9/14/etc (GND)
Encoder Ch A (Yellow) →  [4.7kΩ to GND] → Jetson Pin 16 (GPIO23)
Encoder Ch B (Green)  →  [4.7kΩ to GND] → Jetson Pin 18 (GPIO24)
```

**Home Sensor (to Jetson):**
```
Sensor VCC (Red)    →  Jetson Pin 2 or 4 (5V)
Sensor GND (Black)  →  Jetson Pin 6/9/14/etc (GND)
Sensor Signal (Blue)→  Jetson Pin 23 (GPIO11)
```

### Step 2: Run Quick GPIO Test

```bash
cd /home/severin/dev/r2d2/tests/pan_motor
sudo python3 test_quick_gpio.py
```

**What to do:**
1. Script will ask you to press ENTER - do so
2. It will read all GPIO pins (should show 0 or 1 for each)
3. **Encoder test:** Manually rotate motor shaft slowly - you should see "Change #X: A=X, B=X" messages
4. **Home sensor test:** Move dome to home position - should see state changes

**Troubleshooting:**
- **No encoder changes?** Check encoder wiring and pull-down resistors
- **Home sensor stuck?** Check sensor wiring and 5V power
- **GPIO read error?** Verify pin numbers match Jetson AGX Orin (not Raspberry Pi!)

### Step 3: Run Full Motor Test

⚠️ **WARNING:** Motor will run during this test!

```bash
cd /home/severin/dev/r2d2/tests/pan_motor
sudo python3 test_pan_motor_basic.py
```

**What to do:**
1. Verify pre-flight checklist (battery, wiring, clear rotation)
2. Type 'y' and press ENTER
3. Watch motor carefully during each test phase
4. Press Ctrl+C at any time to emergency stop

**Test Phases:**
1. **GPIO Reading** - No movement, just reads pins
2. **Encoder Detection** - Manually rotate shaft for 15 seconds
3. **Home Sensor** - Manually trigger sensor for 10 seconds
4. **Direction Test** - Motor runs CW and CCW at 30% for 2 seconds each
5. **Speed Test** - Motor runs at 20%, 40%, 60%, 80% for 1 second each
6. **Encoder Accuracy** - Motor runs at different speeds, counts pulses

**What to Watch For:**
- ✅ Motor rotates smoothly without jerking
- ✅ No unusual noises (grinding, clicking)
- ✅ Motor doesn't overheat (warm is OK, hot is not)
- ✅ Encoder counts increase with motor rotation
- ✅ Higher speeds → more encoder pulses
- ❌ Sparking, smoke, burning smell = STOP IMMEDIATELY

---

## 📊 Expected Results

### Encoder Pulse Counts (Approximate)

DeAgostini motors typically have ~12 pulses per revolution (PPR).

**At 30% speed for 2 seconds:**
- Expected: ~50-200 pulses (depends on motor speed and gearing)

**At different speeds:**
- 20% speed: Baseline count
- 40% speed: ~2× baseline
- 60% speed: ~3× baseline

**If encoder counts are 0:**
- Check encoder power (5V)
- Check pull-down resistors (4.7kΩ to GND)
- Verify encoder wiring (channels A and B)
- Test encoder with multimeter (should show voltage changes when rotating)

---

## 🔧 Troubleshooting

### Motor doesn't run
1. Check battery voltage (should be 14.8V-16.8V)
2. Verify VM and GND connections on Pololu driver
3. Check PWM signal on Pin 13 (should be active when motor runs)
4. Verify SLP pin is tied HIGH (3.3V)

### Motor runs but wrong direction
1. Swap MOTOR A and MOTOR B wires on Pololu driver
2. Or invert direction signal in software

### Motor runs but no encoder pulses
1. Check encoder power (5V on red wire)
2. Verify ground connection (black wire)
3. Check pull-down resistors (4.7kΩ each channel)
4. Test encoder with multimeter while rotating shaft

### Home sensor not responding
1. Check sensor power (5V)
2. Verify sensor type (active LOW or active HIGH)
3. Test sensor with multimeter
4. Verify trigger mechanism (reflective strip or magnet)

### Motor overheating
1. Reduce PWM duty cycle (lower speed)
2. Check for mechanical binding (dome should spin freely)
3. Verify battery voltage (low voltage = high current)
4. Check motor current draw (should be < 2A continuous)

---

## 📝 Test Results Template

After testing, document your results:

```
Test Date: _______________
Battery Voltage: _____ V

✅ Quick GPIO Test:
   - Pin reading: [ ] Pass [ ] Fail
   - Encoder detection: [ ] Pass [ ] Fail
   - Home sensor: [ ] Pass [ ] Fail

✅ Full Motor Test:
   - Direction control: [ ] Pass [ ] Fail
   - Speed control: [ ] Pass [ ] Fail
   - Encoder accuracy: [ ] Pass [ ] Fail
   - Encoder pulse count at 30% CW: _____ pulses
   - Encoder pulse count at 30% CCW: _____ pulses

Notes:
_________________________________
_________________________________
_________________________________
```

---

## 🎯 Success Criteria

Tests are successful if:
- ✅ All GPIO pins readable
- ✅ Encoder pulses detected during rotation
- ✅ Home sensor responds to trigger
- ✅ Motor runs smoothly in both directions
- ✅ Speed control works (higher speed = faster rotation)
- ✅ Encoder counts increase proportionally with speed
- ✅ No overheating, noise, or mechanical issues

**If all tests pass:** Ready for ROS 2 integration! 🚀

**If tests fail:** Review troubleshooting section and verify wiring

---

## 🔗 References

- **Hardware Reference:** `/home/severin/dev/r2d2/002_HARDWARE_REFERENCE.md` Section 4.5
- **Pololu G2 Datasheet:** https://www.pololu.com/product/2995
- **Jetson GPIO Library:** https://github.com/NVIDIA/jetson-gpio

---

**Last Updated:** January 6, 2026  
**Author:** R2D2 Development Team

