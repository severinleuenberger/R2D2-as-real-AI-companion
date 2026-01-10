# MCP23017 LED System - Build and Test Guide

**Date:** January 9, 2026  
**Purpose:** Step-by-step instructions for building and testing the LED system

---

## Overview

This guide walks you through the complete hardware and software setup for the MCP23017 4-LED status display system.

**Estimated Time:** 2-3 hours

---

## Phase 1: Hardware Preparation (You Do This First!)

### Step 1: Verify LED Board #1 with Lab Power Supply

**Equipment needed:**
- Lab power supply set to 3.0V
- LED Board #1 (circular, 3 wires: Black, Red, Blue)

**Test 1 - Red Wire:**
```
Power Supply Connections:
  Negative (GND) → Black wire
  Positive (+3V) → Red wire

WATCH: Which color LEDs light up?
Write down: Red wire controls ________ LEDs (red or blue)
```

**Test 2 - Blue Wire:**
```
Power Supply Connections:
  Negative (GND) → Black wire
  Positive (+3V) → Blue wire

WATCH: Which color LEDs light up?
Write down: Blue wire controls ________ LEDs (red or blue)
```

**Record your results - you'll need them later!**

### Step 2: Prepare Green LED

**Inspect Board #2:**
1. Find green SMD LEDs
2. Look for tiny black resistor near green LED (labeled "220" or "221")
3. Take photo for reference

**Desolder:**
- If resistor exists: Desolder both green LED + resistor
- If no resistor: Desolder only green LED (you'll add 220Ω when soldering to Board #1)

**Solder to Board #1:**
1. Choose location on Board #1 for green LED
2. Solder green LED (+ resistor if applicable)
3. Add 4th wire (green) to connector
4. Test with power supply: Black=GND, Green=3V → Green LEDs should light

### Step 3: Prepare Yellow LED

**Equipment:**
- Yellow 3mm LED
- 1× 220Ω resistor

**Assembly:**
1. Identify yellow LED legs:
   - Longer leg = Voltage side
   - Shorter leg = Ground side
2. Connect 220Ω resistor to shorter leg (solder or twist)
3. Test with power supply:
   - Positive → Longer leg
   - Negative → Resistor end
   - Should glow yellow

### Step 4: Connect MCP23017 to Jetson

**⚠️ SHUTDOWN JETSON FIRST:**
```bash
sudo shutdown -h now
# Wait 30 seconds for complete shutdown
```

**I2C Bus Wiring (4 wires):**

| Jetson Pin | Signal | MCP23017 Header | Wire Color |
|------------|--------|-----------------|------------|
| Pin 3 | SDA | SDA | Yellow/White |
| Pin 5 | SCL | SCL | Blue/Green |
| Pin 1 | 3.3V | VCC | Red |
| Pin 6 | GND | GND | Black |

**Use the bottom headers on MCP23017** labeled VCC, GND, SDA, SCL.

### Step 5: Connect LEDs to MCP23017

**LED Board #1 (now with 4 wires):**

| Board Wire | MCP23017 Header |
|------------|-----------------|
| Black | GND |
| Red | PA0 or PA1 (we'll test which) |
| Blue | PA1 or PA0 (we'll test which) |
| Green | PA2 |

**Yellow LED:**

| Yellow LED Connection | MCP23017 Header |
|-----------------------|-----------------|
| Longer leg (voltage) | PA3 |
| Resistor end (from shorter leg) | GND (can share with Board #1) |

**Tip:** Use common ground wire - connect Board #1 black wire + yellow resistor together, then single wire to MCP23017 GND.

### Step 6: Power On Jetson

```bash
# Press power button or reconnect power
# Wait for boot (~30 seconds)
```

---

## Phase 2: Software Setup (I Help You With This!)

### Step 1: Verify I2C Detection

```bash
sudo i2cdetect -y 1
```

**Expected:** Should show "20" at address 0x20

**If not detected:** Check wiring, try `sudo i2cdetect -y 0` or `-y 8`

### Step 2: Install Python Libraries

```bash
cd ~/dev/r2d2/tests/led_expansion
./install_mcp23017_libraries.sh
```

**Verify:**
```bash
python3 -c "import board, busio; from adafruit_mcp230xx.mcp23017 import MCP23017; print('✅ Libraries OK')"
```

### Step 3: Test All LEDs

```bash
cd ~/dev/r2d2/tests/led_expansion
python3 test_mcp23017_4led_basic.py
```

**What should happen:**
- PA0 LED turns on/off
- PA1 LED turns on/off
- PA2 (green) LED turns on/off
- PA3 (yellow) LED turns on/off
- Sequential test runs
- All on/off test runs
- Status simulation works
- Gesture flash works

**If fails:** Check LED polarity, resistors, wiring

### Step 4: Identify Wire Mapping

```bash
python3 test_mcp23017_led_mapping.py
```

**Interactive test - watch LEDs and answer questions:**
- PA0 turns on → What color do you see?
- PA1 turns on → What color do you see?
- PA2 turns on → What color do you see? (should be green)
- PA3 turns on → What color do you see? (should be yellow)

**Write down the mapping:**
- PA0 → _____ LED
- PA1 → _____ LED
- PA2 → _____ LED
- PA3 → _____ LED

### Step 5: Configure ROS2 Node

**If mapping is NOT standard (wire colors don't match LED colors):**

Edit: `ros2_ws/src/r2d2_audio/launch/all_audio_services.launch.py`

Find the mcp23017_status_led_node section and update:
```python
'pa0_controls': 'blue',    # Example: if red wire controls blue LEDs
'pa1_controls': 'red',     # Example: if blue wire controls red LEDs
'pa2_controls': 'green',
'pa3_controls': 'yellow',
```

**If mapping IS standard (red wire = red LEDs):**
- No changes needed! Default configuration is correct.

### Step 6: Build ROS2 Package

```bash
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_audio
source install/setup.bash
```

### Step 7: Test ROS2 Node (Standalone)

**Terminal 1 - Start LED node:**
```bash
cd ~/dev/r2d2/ros2_ws
source install/setup.bash
ros2 launch r2d2_audio mcp23017_status_led.launch.py
```

**Expected output:**
```
[INFO] [mcp23017_status_led_node]: MCP23017 Status LED Node initializing...
[INFO] [mcp23017_status_led_node]: Connecting to MCP23017 at 0x20...
[INFO] [mcp23017_status_led_node]:   PA0 → RED LED
[INFO] [mcp23017_status_led_node]:   PA1 → BLUE LED
[INFO] [mcp23017_status_led_node]:   PA2 → GREEN LED
[INFO] [mcp23017_status_led_node]:   PA3 → YELLOW LED
[INFO] [mcp23017_status_led_node]: ✅ MCP23017 Status LED Node ready
```

**Terminal 2 - Test manually:**
```bash
cd ~/dev/r2d2/ros2_ws && source install/setup.bash

# Test RED status
ros2 topic pub --once /r2d2/audio/person_status std_msgs/String \
  'data: "{\"status\": \"red\", \"person_identity\": \"test\"}"'
# → Red LED should turn ON, others OFF

# Test BLUE status
ros2 topic pub --once /r2d2/audio/person_status std_msgs/String \
  'data: "{\"status\": \"blue\", \"person_identity\": \"no_person\"}"'
# → Blue LED should turn ON, others OFF

# Test GREEN status
ros2 topic pub --once /r2d2/audio/person_status std_msgs/String \
  'data: "{\"status\": \"green\", \"person_identity\": \"unknown\"}"'
# → Green LED should turn ON, others OFF

# Test gesture flash
ros2 topic pub --once /r2d2/perception/gesture_event std_msgs/String \
  "data: 'fist'"
# → Yellow LED should flash 500ms (independent of status LED)
```

---

## Phase 3: Full System Integration

### Step 1: Restart Audio Notification Service

```bash
sudo systemctl restart r2d2-audio-notification.service
```

The service uses `all_audio_services.launch.py` which now includes the MCP23017 LED node.

### Step 2: Monitor with minimal_monitor.py

```bash
python3 ~/dev/r2d2/tools/minimal_monitor.py
```

### Step 3: Verification Matrix

Stand in front of camera and verify:

| minimal_monitor.py Shows | Physical LED State | ✓ |
|-------------------------|-------------------|---|
| 🔴 RED (your name) | Red LED ON, blue/green OFF | [ ] |
| 🔵 BLUE (no_person) | Blue LED ON, red/green OFF | [ ] |
| 🟢 GREEN (unknown) | Green LED ON, red/blue OFF | [ ] |
| ☝️ gesture | Yellow LED flash 500ms | [ ] |
| ✊ gesture | Yellow LED flash 500ms | [ ] |
| 🖐️ gesture | Yellow LED flash 500ms | [ ] |

**All boxes must be checked!**

### Step 4: Interaction Test

Complete walkthrough:

1. **Start in BLUE:** No one in front of camera
   - Verify: Blue LED ON
   
2. **Approach camera:** Face detected
   - Transition: BLUE → RED
   - Verify: Red LED turns ON, blue turns OFF
   - Hear: "Hello!" beep
   
3. **Make gesture:** Index finger up when RED
   - Verify: Yellow LED flashes (red stays on)
   
4. **Walk away:** Leave camera view
   - Transition: RED → BLUE (after 15s timer + 5s absence)
   - Verify: Blue LED turns ON, red turns OFF
   - Hear: "Lost you!" beep

5. **Unknown person test** (if possible):
   - Have untrained person stand in front
   - Transition: BLUE → GREEN
   - Verify: Green LED turns ON

---

## Phase 4: Production Deployment

### Step 1: Verify Auto-Start

```bash
# Check service is enabled
sudo systemctl is-enabled r2d2-audio-notification.service
# Should show: enabled
```

### Step 2: Reboot Test

```bash
sudo reboot
```

**After reboot (~60 seconds):**

```bash
# Check LED node is running
ros2 node list | grep mcp23017

# Monitor status
python3 ~/dev/r2d2/tools/minimal_monitor.py

# Walk in front of camera
# Verify: LEDs work automatically without manual intervention
```

### Step 3: Final Verification

All systems operational checklist:

- [ ] I2C detected at boot: `sudo i2cdetect -y 1` shows 0x20
- [ ] LED node auto-starts: Visible in `ros2 node list`
- [ ] RED status: Red LED on
- [ ] BLUE status: Blue LED on
- [ ] GREEN status: Green LED on
- [ ] Gestures: Yellow flash works
- [ ] minimal_monitor.py: LED colors match display
- [ ] Audio switch: Still functional (Bluetooth/Speaker toggle)
- [ ] No console errors in systemd logs

---

## Troubleshooting Quick Reference

| Issue | Quick Fix |
|-------|-----------|
| I2C not detected | Check 4 wires: Pin 3→SDA, Pin 5→SCL, Pin 1→VCC, Pin 6→GND |
| LED not lighting | Check polarity (reverse LED if wrong), check resistor |
| Wrong LED colors | Update pa0_controls/pa1_controls in launch file |
| Yellow not flashing | Check gestures work: `ros2 topic echo /r2d2/perception/gesture_event` |
| Node not starting | Rebuild: `colcon build --packages-select r2d2_audio` |

**For complete troubleshooting, see:** [270_LED_INSTALLATION.md](../../270_LED_INSTALLATION.md#troubleshooting)

---

## Success Criteria

When complete, you should have:

✅ **Hardware:**
- MCP23017 detected at I2C address 0x20
- All 4 LEDs wired correctly (red, blue, green, yellow)
- Resistors installed (yellow LED, green if needed)
- No short circuits or wiring errors

✅ **Software:**
- Python libraries installed
- ROS2 package built successfully
- Node auto-starts on boot
- LEDs respond to status/gesture topics

✅ **Integration:**
- LED colors match minimal_monitor.py exactly
- Status transitions work (RED/BLUE/GREEN)
- Gesture flash works (yellow)
- System works after reboot
- No errors in logs

---

**Next Steps After Completion:**

1. Review [270_LED_INSTALLATION.md](../../270_LED_INSTALLATION.md) for complete system reference
2. Consider adding more LEDs (12 outputs still available on MCP23017)
3. Customize gesture flash duration if desired
4. Take photos of your installation for documentation

---

**Document Status:** Ready for use  
**Last Updated:** January 9, 2026

