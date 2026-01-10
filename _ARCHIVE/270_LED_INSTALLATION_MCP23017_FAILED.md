# R2D2 Status LED System - Complete Reference
## MCP23017 I2C 4-LED Status Display

**Date:** January 9, 2026  
**Status:** ✅ Production Implementation Guide  
**Platform:** NVIDIA Jetson AGX Orin 64GB + ROS 2 Humble  
**Hardware:** MCP23017 I2C GPIO Expander + 4 Individual LEDs

---

## Executive Summary

The R2D2 status LED system provides real-time visual feedback for person recognition status and gesture detection using 4 individual LEDs controlled via an MCP23017 I2C GPIO expander. This solution uses only 2 GPIO pins (I2C) while supporting 4 independent LED indicators with potential for 12 more LEDs using the same board.

**Key Features:**
- **4-Color Status Display:** RED (recognized), BLUE (no person), GREEN (unknown), YELLOW (gesture flash)
- **Minimal GPIO Usage:** Only 2 pins (I2C SDA/SCL) for all LEDs
- **Matches Monitoring Display:** LED colors exactly match `minimal_monitor.py` status indicators
- **Scalable:** Can add up to 12 more LEDs on same board
- **Chainable:** Can add up to 7 more MCP23017 boards (128 LEDs total)

**System Integration:**
- Subscribes to `/r2d2/audio/person_status` for RED/BLUE/GREEN status
- Subscribes to `/r2d2/perception/gesture_event` for yellow flash
- Replaces old white LED (GPIO 17) with multi-color status display
- Frees up Pin 22 for audio switch exclusively

---

## Table of Contents

1. [Hardware Overview](#hardware-overview)
2. [Bill of Materials](#bill-of-materials)
3. [Understanding LED Basics](#understanding-led-basics)
4. [Wiring Guide](#wiring-guide)
5. [Software Installation](#software-installation)
6. [LED Behavior Reference](#led-behavior-reference)
7. [Testing Procedures](#testing-procedures)
8. [ROS2 Integration](#ros2-integration)
9. [Troubleshooting](#troubleshooting)
10. [Configuration](#configuration)
11. [Maintenance](#maintenance)

---

## Hardware Overview

### MCP23017 I2C GPIO Expander

**Product:** Waveshare MCP23017 I/O Expansion Board  
**Link:** https://www.berrybase.ch/waveshare-mcp23017-io-erweiterungsplatine-16-digitale-io-ph2.0-oder-loetpads-i2c-3-3v-5v

**Key Specifications:**
- **16 GPIO outputs:** 8 on Port A (PA0-PA7), 8 on Port B (PB0-PB7)
- **I2C interface:** Uses only 2 GPIO pins (SDA, SCL)
- **Voltage:** 3.3V or 5V compatible (Jetson 3.3V ✅)
- **Current per pin:** 25mA max (our LEDs: ~6mA ✅)
- **I2C address:** 0x20 default (configurable 0x20-0x27)
- **Extra features:** INTA/INTB interrupt pins (not used for LED output)

**Board Headers:**
- **Bottom:** VCC, GND, SDA, SCL (I2C connection to Jetson)
- **Sides:** PA0-PA7, PB0-PB7 (LED outputs)
- **Top:** INTA, INTB, PWR (interrupt pins - not needed)

### LED Hardware

**R2D2 System Uses 4 LEDs:**

1. **Red LED** - Person recognized status
2. **Blue LED** - No person status  
3. **Green LED** - Unknown person status
4. **Yellow LED** - Gesture detected flash

**LED Types in This Project:**

| LED | Type | Source | Resistor Needed |
|-----|------|--------|-----------------|
| **Red** | Pre-assembled board (Board #1) | 3-wire module | ❌ No (built-in) |
| **Blue** | Pre-assembled board (Board #1) | Same 3-wire module | ❌ No (built-in) |
| **Green** | SMD LED from Board #2 | Desolder + solder to Board #1 | ❓ Check Board #2 |
| **Yellow** | Standard 3mm LED | Separate component | ✅ Yes (220Ω) |

---

## Bill of Materials

### Required Components

| Item | Quantity | Specification | Purpose | Source |
|------|----------|---------------|---------|--------|
| **MCP23017 Board** | 1 | Waveshare I/O Expansion Board | GPIO expander | BerryBase |
| **LED Board #1** | 1 | Pre-assembled (Red+Blue, 3-wire) | Status LEDs | Your kit |
| **LED Board #2** | 1 | Pre-assembled (Yellow+Green) | Source for green LED | Your kit |
| **Yellow LED** | 1 | Standard 3mm LED | Gesture indicator | Your kit |
| **220Ω Resistors** | 1-2 | 1/4W metal film | Current limiting | Standard electronics |
| **Jumper Wires** | 8-10 | Female-to-Female Dupont | Connections | Standard electronics |
| **Wire** | 1m | 22-26 AWG, various colors | Green LED 4th wire | Standard electronics |
| **Soldering Kit** | - | Iron, solder, flux | Green LED installation | Standard tools |

**Total Cost:** ~$8-12 USD (MCP23017 board only, assuming you have LEDs and wires)

### Tools Required

- Soldering iron + solder (for green LED)
- Wire strippers
- Lab power supply (3V, for testing)
- Multimeter (optional, for verification)
- Small screwdriver (for board installation)

---

## Understanding LED Basics

### LED Polarity Explained (For Beginners)

LEDs have **two legs** that must be connected correctly:

**Terminology Translation:**
| Technical Term | Simple Term | Where It Connects |
|----------------|-------------|-------------------|
| **Anode (+)** | **Voltage side** | MCP23017 output pin (PA0-PA3) |
| **Cathode (-)** | **Ground side** | Resistor → GND |

**How to Identify LED Legs:**

```
         Longer leg = VOLTAGE side (connect to MCP23017)
              |
              |
         ┌────┴────┐
         │   LED   │ ← Round plastic lens
         │  Body   │
         └────┬────┘
              |
              |
         Shorter leg = GROUND side (connect to resistor → GND)
```

**Alternative: Flat Edge Method**

Look at LED body from top - one side has a **flat edge**:
- **Flat edge side** = Ground (cathode)
- **Round side** = Voltage (anode)

**If You Connect It Backwards:**
- LED won't light up
- ✅ No damage! Just reverse the legs and try again
- LEDs are polarity-sensitive but not fragile

### Why Current-Limiting Resistors Are Required

**The Physics:**

An LED is like a one-way valve for electricity that emits light. Without a resistor:

```
MCP23017 Output (3.3V)
    ↓
LED (uses ~2.0V, has almost no resistance)
    ↓
Almost all voltage drops across LED
    ↓
Result: 50-100mA flows (WAY TOO MUCH!)
    ↓
LED overheats and burns out within seconds
MCP23017 chip also damaged (max 25mA per pin)
```

**With 220Ω Resistor:**

```
MCP23017 Output: 3.3V
    ↓
LED: Uses 2.0V (leaves 1.3V)
    ↓
220Ω Resistor: Uses remaining 1.3V
    ↓
Current = 1.3V / 220Ω = 5.9mA ✅ SAFE!
    ↓
GND (0V)
```

**The Math:**
- Current = (Voltage_supply - Voltage_LED) / Resistance
- Current = (3.3V - 2.0V) / 220Ω = 5.9mA
- This is well below the 25mA limit of MCP23017
- LED glows nicely without overheating

**Why 220Ω Specifically:**
- Standard value (easy to find)
- Provides safe current (5-6mA)
- Bright enough to see clearly
- Works for all LED colors (red, green, blue, yellow)
- Alternative values: 150Ω (brighter), 330Ω (dimmer)

---

## Wiring Guide

### Phase 1: I2C Bus Connection (Jetson ↔ MCP23017)

**⚠️ POWER OFF JETSON BEFORE WIRING!**

```bash
sudo shutdown -h now
# Wait for complete shutdown (~30 seconds)
```

**Connections (4 wires):**

| Jetson 40-Pin Header | Signal | MCP23017 Board Header | Wire Color (Suggested) |
|----------------------|--------|----------------------|------------------------|
| **Pin 3** | I2C5_DAT (SDA) | **SDA** | Yellow or White |
| **Pin 5** | I2C5_CLK (SCL) | **SCL** | Blue or Green |
| **Pin 1** | 3.3V Power | **VCC** | Red |
| **Pin 6** | GND | **GND** | Black |

**Jetson Pin Location Reference:**

Refer to the Jetson pinout image (NVIDIA official):
- **Pin 1** is marked with white triangle on board (top-left when viewing from above)
- **Pin 3** is in same column as Pin 1, second position down (left side)
- **Pin 5** is in same column as Pin 1, third position down (left side)
- **Pin 6** is in right column, third position down (right side)

**MCP23017 Header Location:**

The I2C connection headers are at the **bottom** of the MCP23017 board:
- Labeled: **VCC**, **GND**, **SDA**, **SCL**
- Usually 4-pin single-row header or 4 separate holes for wiring

**Wiring Photo Reference:**
(Your board has colored wires already connected - see photos for reference)

### Phase 2: LED Board #1 Connection (Red + Blue LEDs)

**Your Board #1 Characteristics:**
- Pre-assembled circular PCB with SMD LEDs
- 3-wire connector: Black, Red, Blue
- Built-in current-limiting resistors
- Tested working at 3V

**⚠️ IMPORTANT: Wire colors might NOT match LED colors!**

Before connecting to MCP23017, verify with lab power supply:

**Verification Test:**
```
Lab Power Supply (3V):
  Negative → Black wire
  Positive → Red wire
  → Observe: Do RED or BLUE LEDs light up?

Lab Power Supply (3V):
  Negative → Black wire
  Positive → Blue wire
  → Observe: Do RED or BLUE LEDs light up?
```

Record results in `tests/led_expansion/HARDWARE_VERIFICATION_GUIDE.md`.

**Connection to MCP23017:**

| Board #1 Wire | Color | MCP23017 Header | Notes |
|---------------|-------|-----------------|-------|
| **Black** | Black | **GND** | Common ground |
| **Red** | Red | **PA0** or **PA1** | Test to verify which color this controls |
| **Blue** | Blue | **PA1** or **PA0** | Test to verify which color this controls |

**After Green LED Added:**

| Board #1 Wire | Color | MCP23017 Header | Controls |
|---------------|-------|-----------------|----------|
| Black | Black | **GND** | Common ground |
| Red | Red | **PA0** | Red LEDs (or blue if reversed) |
| Blue | Blue | **PA1** | Blue LEDs (or red if reversed) |
| **Green** | Green | **PA2** | Green LEDs (after you add green LED to board) |

### Phase 3: Green LED Addition (Soldering Required)

**Source:** Board #2 (Yellow + Green LED module)

**Step 1: Inspect Board #2**
1. Locate green SMD LEDs on board
2. Look for nearby resistor (small black component, might be labeled "220" or "221")
3. Take photo before desoldering (for reference)

**Step 2: Desolder Components**

**If resistor exists near green LED:**
- Desolder both green LED + resistor
- Keep them together (you'll solder both to Board #1)

**If no resistor near green LED:**
- Desolder only green LED
- You'll need to add 220Ω resistor when soldering to Board #1

**Step 3: Solder to Board #1**

1. Choose location on Board #1 PCB for green LED
2. Solder green LED (+ resistor if needed) to board
3. Wire from green LED → New 4th wire on connector
4. Test with lab power supply: Black=GND, Green=3V → Green LEDs should light

**Step 4: Add 4th Wire to Connector**

Options:
- Solder new wire directly to PCB pad
- Use connector with 4 positions (JST PH 2.0 or similar)
- Extend existing 3-wire connector

### Phase 4: Yellow LED Connection (Simple LED)

**Hardware:** Standard 3mm yellow LED (2 legs)

**Required:** 1× 220Ω resistor (mandatory!)

**Wiring:**

```
Yellow LED Assembly:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
1. Identify LED legs:
   - Longer leg = Voltage side (anode)
   - Shorter leg = Ground side (cathode)

2. Connect resistor to cathode:
   - Solder or twist 220Ω resistor to shorter LED leg
   - Or use breadboard/wire connector

3. Final connections:
   - LED longer leg (anode) → MCP23017 PA3 header
   - Resistor other end → MCP23017 GND header
   
   OR use a common ground wire:
   - LED longer leg → PA3
   - LED shorter leg → 220Ω resistor → Common GND wire
   - Common GND wire → MCP23017 GND (shared with Board #1)
```

**Visual Diagram:**

```
MCP23017 PA3 Header
        ↓
   Yellow LED longer leg (voltage side)
        ↓
   Yellow LED shorter leg (ground side)
        ↓
   220Ω Resistor
        ↓
   GND wire (can share with Board #1 black wire)
        ↓
MCP23017 GND Header
```

### Phase 5: Complete System Wiring Diagram

**Full Connection Map:**

```
═══════════════════════════════════════════════════════════════════════
JETSON AGX ORIN 40-PIN HEADER → MCP23017 I2C CONNECTION
═══════════════════════════════════════════════════════════════════════

Jetson Side:                      MCP23017 Side:
┌─────────────┐                   ┌──────────────┐
│ Pin 3 (SDA) │────────────────→  │ SDA header   │
│ Pin 5 (SCL) │────────────────→  │ SCL header   │
│ Pin 1 (3.3V)│────────────────→  │ VCC header   │
│ Pin 6 (GND) │────────────────→  │ GND header   │
└─────────────┘                   └──────────────┘

4 wires total (standard Dupont female-to-female jumpers)


═══════════════════════════════════════════════════════════════════════
MCP23017 OUTPUT HEADERS → LED CONNECTIONS
═══════════════════════════════════════════════════════════════════════

LED Board #1 (Pre-assembled with resistors):
┌────────────────┐                   ┌──────────────────┐
│ Black wire     │────────────────→  │ GND header       │
│ Red wire       │────────────────→  │ PA0 or PA1       │ (test to verify)
│ Blue wire      │────────────────→  │ PA1 or PA0       │ (test to verify)
│ Green wire     │────────────────→  │ PA2 header       │ (after soldering)
└────────────────┘                   └──────────────────┘

NO external resistors needed (built into board!)


Yellow LED (Separate 3mm LED + resistor):
┌─────────────────────────────┐    ┌──────────────────┐
│ Longer leg (voltage/anode)  │───→│ PA3 header       │
│ Shorter leg (ground/cathode)│───→│ 220Ω resistor    │
│                              │    │      ↓           │
│                              │    │ GND header       │
└─────────────────────────────┘    └──────────────────┘

Needs 220Ω resistor! (no built-in resistor)


═══════════════════════════════════════════════════════════════════════
COMMON GROUND WIRING TIP
═══════════════════════════════════════════════════════════════════════

You can use ONE ground wire for all components:

Board #1 Black wire ───┐
Yellow LED resistor ───┼─── Common GND wire ─── MCP23017 GND header
                       │
                       └─── (Can add more LEDs here)

This reduces wiring complexity - only one GND wire to MCP23017!
```

**Safety Checklist:**
- [ ] Verify 3.3V goes to VCC (NOT 5V - could work but 3.3V is safer)
- [ ] Verify GND connected to GND (not swapped with VCC!)
- [ ] Verify SDA/SCL are not swapped
- [ ] Check no short circuits between pins (use multimeter)
- [ ] Verify LED polarity correct (longer leg to PA headers)
- [ ] Verify resistors installed on simple LEDs (yellow, and green if needed)

---

## Software Installation

### Step 1: Install MCP23017 Python Libraries

```bash
cd ~/dev/r2d2/tests/led_expansion
./install_mcp23017_libraries.sh
```

**What this installs:**
- `adafruit-blinka` - CircuitPython compatibility for Jetson
- `adafruit-circuitpython-mcp230xx` - MCP23017 control library
- `adafruit-circuitpython-busdevice` - I2C communication

**Verify installation:**
```bash
python3 << 'EOF'
import board, busio
from adafruit_mcp230xx.mcp23017 import MCP23017
print("✅ Libraries installed correctly!")
EOF
```

### Step 2: Verify I2C Detection

**After MCP23017 is wired to Jetson:**

```bash
# Scan I2C bus 1 (40-pin header I2C bus)
sudo i2cdetect -y 1
```

**Expected output:**
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: 20 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
...
```

You should see **"20"** at address 0x20 (row 20, column 0).

**If not detected:**
- Check 4 I2C wires: VCC, GND, SDA, SCL
- Verify MCP23017 has power (check for power LED on board)
- Try other I2C buses: `sudo i2cdetect -y 0`, `sudo i2cdetect -y 8`
- Check address jumpers (A0, A1, A2 should be open for 0x20)

### Step 3: Build ROS2 Package

```bash
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_audio
source install/setup.bash
```

---

## LED Behavior Reference

### Status LED Mapping

The LED colors **exactly match** the color-coded status in `minimal_monitor.py`:

| Person Status | Active LED | Color | Meaning | Other LEDs |
|---------------|-----------|-------|---------|------------|
| **RED** | Red LED | 🔴 | Person recognized (active engagement) | Blue/Green OFF |
| **BLUE** | Blue LED | 🔵 | No person (idle/waiting) | Red/Green OFF |
| **GREEN** | Green LED | 🟢 | Unknown person (caution) | Red/Blue OFF |

**Key Behavior:**
- **Mutually exclusive:** Only ONE status LED on at any time
- **Instant switching:** Status change → LED change within 100ms
- **Synchronized:** LED changes synchronized with audio beeps
- **Persistent:** LED stays on until status changes (not blinking)

### Gesture Flash Indicator

| Event | LED | Behavior | Duration | Independent |
|-------|-----|----------|----------|-------------|
| **Any gesture** | Yellow LED | Flash ON then OFF | 500ms | ✅ Yes |

**Supported gestures:**
- ☝️ Index finger up (start conversation)
- ✊ Fist (stop conversation)
- 🖐️ Open hand (intelligent mode)

**Key Behavior:**
- **Independent:** Yellow can flash while status LED is on
- **Example:** RED status (red LED on) + gesture → Red + Yellow both on for 500ms
- **Non-blocking:** Flash uses timer, doesn't block status updates
- **Overlap handling:** New gesture cancels previous flash timer

### Example State Transitions

**Sequence 1: Person Recognition**
```
t=0s:     BLUE status → Blue LED ON
t=0.5s:   Person appears → Face detected
t=1.0s:   Person recognized → RED status
          → Red LED ON, Blue LED OFF
          → Audio beep plays
t=15s:    Person still visible → Red LED stays ON
t=20s:    Person walks away → BLUE status
          → Blue LED ON, Red LED OFF
          → "Lost you" audio plays
```

**Sequence 2: Gesture Detection**
```
State: RED status (red LED ON)
t=0s:     User makes index finger gesture
t=0.1s:   Gesture detected → Yellow flash starts
          → Red LED ON, Yellow LED ON (both!)
t=0.6s:   Flash complete → Yellow LED OFF
          → Red LED stays ON (status unchanged)
```

### Visual Reference: Matching minimal_monitor.py

When you run `python3 ~/dev/r2d2/tools/minimal_monitor.py`, the STATUS column shows:

```
STATUS Column    Physical LEDs
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
🔴 RED    →      Red LED ON, Blue/Green OFF
🔵 BLUE   →      Blue LED ON, Red/Green OFF
🟢 GREEN  →      Green LED ON, Red/Blue OFF

GESTURE Column   Physical LEDs
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
☝️ ✊ 🖐️   →      Yellow LED flashes 500ms
```

The physical LEDs **must exactly match** the monitor display colors!

---

## Testing Procedures

### Test 1: Hardware Verification (Before ROS2)

**Location:** `tests/led_expansion/`

**Step 1: Verify Board #1 wiring**
```bash
# Read the guide and test with lab power supply
cat tests/led_expansion/HARDWARE_VERIFICATION_GUIDE.md
```

**Step 2: Test I2C detection**
```bash
sudo i2cdetect -y 1
# Should show "20" at address 0x20
```

**Step 3: Test all 4 LEDs**
```bash
cd ~/dev/r2d2/tests/led_expansion
python3 test_mcp23017_4led_basic.py
```

**Expected output:**
- All 4 LEDs turn on/off individually
- Sequential test passes
- All on/off test passes
- Status simulation works (RED/BLUE/GREEN)
- Gesture flash works (yellow)

**Step 4: Identify wire mapping**
```bash
python3 test_mcp23017_led_mapping.py
```

Follow prompts and record which PA pin controls which LED color.

**Save the mapping!** You'll need it for configuration:
- PA0 controls: _____ LED
- PA1 controls: _____ LED
- PA2 controls: _____ LED (should be green)
- PA3 controls: _____ LED (should be yellow)

### Test 2: ROS2 Integration Test

**Start the LED node:**
```bash
cd ~/dev/r2d2/ros2_ws
source install/setup.bash

# Launch with default mapping
ros2 launch r2d2_audio mcp23017_status_led.launch.py

# OR launch with reversed mapping (if needed)
ros2 launch r2d2_audio mcp23017_status_led.launch.py \
  pa0_controls:=blue pa1_controls:=red
```

**Test status changes manually:**

Open new terminal:
```bash
cd ~/dev/r2d2/ros2_ws && source install/setup.bash

# Test RED status
ros2 topic pub --once /r2d2/audio/person_status std_msgs/String \
  'data: "{\"status\": \"red\", \"person_identity\": \"test\"}"'
# → Red LED should turn ON

# Test BLUE status  
ros2 topic pub --once /r2d2/audio/person_status std_msgs/String \
  'data: "{\"status\": \"blue\", \"person_identity\": \"no_person\"}"'
# → Blue LED should turn ON, red OFF

# Test GREEN status
ros2 topic pub --once /r2d2/audio/person_status std_msgs/String \
  'data: "{\"status\": \"green\", \"person_identity\": \"unknown\"}"'
# → Green LED should turn ON, blue OFF

# Test gesture flash
ros2 topic pub --once /r2d2/perception/gesture_event std_msgs/String \
  "data: 'index_finger_up'"
# → Yellow LED should flash 500ms
```

### Test 3: Full System Integration

**Terminal 1: Start complete audio system**
```bash
cd ~/dev/r2d2/ros2_ws
source install/setup.bash
ros2 launch r2d2_audio all_audio_services.launch.py
```

**Terminal 2: Run monitoring**
```bash
python3 ~/dev/r2d2/tools/minimal_monitor.py
```

**Verification Matrix:**

| Monitor Display | Physical LED State | Status |
|-----------------|-------------------|--------|
| 🔴 RED (severin) | Red LED ON, others OFF | [ ] Verified |
| 🔵 BLUE (no_person) | Blue LED ON, others OFF | [ ] Verified |
| 🟢 GREEN (unknown) | Green LED ON, others OFF | [ ] Verified |
| ☝️ or ✊ gesture | Yellow LED flash 500ms | [ ] Verified |

**Interaction test:**
1. Stand in front of camera (BLUE → RED transition)
2. Verify red LED turns on when recognized
3. Make gesture (index finger or fist)
4. Verify yellow LED flashes
5. Walk away (RED → BLUE transition)
6. Verify blue LED turns on when lost

**All LEDs must match the monitoring display EXACTLY!**

---

## ROS2 Integration

### Node Architecture

**Node Name:** `mcp23017_status_led_node`  
**Package:** `r2d2_audio`  
**Type:** Standard ROS2 node

**Subscriptions:**

| Topic | Type | Rate | Purpose |
|-------|------|------|---------|
| `/r2d2/audio/person_status` | String (JSON) | 10 Hz | Status updates (RED/BLUE/GREEN) |
| `/r2d2/perception/gesture_event` | String | Event | Gesture flash trigger |

**Published Topics:** None (pure consumer node)

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `i2c_bus` | int | 1 | I2C bus number (usually 1 for 40-pin header) |
| `i2c_address` | int | 32 | MCP23017 address (32=0x20 decimal) |
| `pa0_controls` | string | 'red' | Which LED color PA0 controls |
| `pa1_controls` | string | 'blue' | Which LED color PA1 controls |
| `pa2_controls` | string | 'green' | Which LED color PA2 controls |
| `pa3_controls` | string | 'yellow' | Which LED color PA3 controls |
| `gesture_flash_duration_ms` | int | 500 | Yellow LED flash duration (ms) |
| `enabled` | bool | true | Enable/disable LED control |
| `simulate` | bool | false | Force simulation mode |

### Launch Files

**Dedicated launch (for testing):**
```bash
ros2 launch r2d2_audio mcp23017_status_led.launch.py
```

**Integrated launch (production):**
```bash
ros2 launch r2d2_audio all_audio_services.launch.py
```

The integrated launch starts:
1. `audio_notification_node` - Status state machine
2. `mcp23017_status_led_node` - LED control (this node)
3. `database_logger_node` - Event logging

### Systemd Service Integration

**Service:** `r2d2-audio-notification.service`

The LED node is automatically started as part of the audio notification system. No separate service needed.

**Check status:**
```bash
sudo systemctl status r2d2-audio-notification.service
```

**View logs:**
```bash
sudo journalctl -u r2d2-audio-notification.service -f | grep mcp23017
```

---

## Troubleshooting

### Issue 1: I2C Device Not Detected

**Symptom:**
```bash
sudo i2cdetect -y 1
# Shows no device at 0x20
```

**Solutions:**

**1. Check physical wiring:**
```bash
# Verify 4 I2C connections:
Jetson Pin 3 → MCP23017 SDA ✅
Jetson Pin 5 → MCP23017 SCL ✅
Jetson Pin 1 → MCP23017 VCC ✅
Jetson Pin 6 → MCP23017 GND ✅
```

**2. Check power:**
- Look for power LED on MCP23017 board (should be lit)
- Measure voltage at MCP23017 VCC pin (should be 3.3V ±0.1V)
- If 0V, check connection to Jetson Pin 1

**3. Try different I2C bus:**
```bash
sudo i2cdetect -y 0   # Try bus 0
sudo i2cdetect -y 8   # Try bus 8 (some Jetson configs)
```

**4. Check address jumpers:**
- A0, A1, A2 jumpers on MCP23017 board
- Default: all open (not soldered) = address 0x20
- If soldered, address changes (0x21, 0x22, etc.)

### Issue 2: LEDs Not Responding

**Symptom:** MCP23017 detected, but LEDs don't light up

**Solutions:**

**1. Check LED polarity (for simple LEDs):**
```
Correct:   PA header → Longer leg → LED → Shorter leg → Resistor → GND
Wrong:     PA header → Shorter leg → LED → ... (LED won't light)

Fix: Reverse the LED (swap the legs)
```

**2. Check resistor (for simple LEDs only):**
- Yellow LED needs 220Ω resistor between cathode and GND
- Green LED needs resistor if not built into Board #1
- Board #1 red/blue: NO resistor needed (built-in)

**3. Test with standalone script:**
```bash
python3 tests/led_expansion/test_mcp23017_4led_basic.py
# Should light each LED individually
```

**4. Check voltage at LED pins:**
```bash
# Use multimeter to measure voltage
# When LED should be ON: PA pin should read ~3.3V
# When LED should be OFF: PA pin should read ~0V
```

**5. Verify LED isn't burned out:**
```bash
# Test LED directly with lab power supply (3V + resistor)
# If LED doesn't light with any polarity, LED is dead (replace)
```

### Issue 3: Wrong LED Colors

**Symptom:** RED status turns on blue LED instead of red LED

**Cause:** Wire colors don't match LED colors (Board #1 wiring)

**Solution:** Update pin mapping in launch file

```bash
# Edit launch parameters
nano ~/dev/r2d2/ros2_ws/src/r2d2_audio/launch/all_audio_services.launch.py

# Change from:
'pa0_controls': 'red',
'pa1_controls': 'blue',

# To (if reversed):
'pa0_controls': 'blue',    # Red wire actually controls blue LEDs
'pa1_controls': 'red',     # Blue wire actually controls red LEDs

# Rebuild
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_audio
sudo systemctl restart r2d2-audio-notification.service
```

### Issue 4: Yellow LED Doesn't Flash

**Symptom:** Status LEDs work, but yellow LED doesn't flash on gestures

**Solutions:**

**1. Check gesture recognition is working:**
```bash
# Monitor gesture events
ros2 topic echo /r2d2/perception/gesture_event

# Make gestures in front of camera when recognized (RED status)
# Should see: data: 'index_finger_up' or 'fist'
```

**2. Check gesture recognition is enabled:**
```bash
# Check if gesture training is complete
ls ~/dev/r2d2/data/gesture_recognition/models/

# Should have: {person}_gesture_classifier.pkl
```

**3. Test yellow LED directly:**
```bash
# Publish test gesture event
ros2 topic pub --once /r2d2/perception/gesture_event std_msgs/String \
  "data: 'fist'"

# Yellow LED should flash
```

**4. Check node is receiving gestures:**
```bash
# View node logs
sudo journalctl -u r2d2-audio-notification.service -f | grep "Gesture detected"
```

### Issue 5: ROS2 Node Not Starting

**Symptom:**
```bash
ros2 run r2d2_audio mcp23017_status_led_node
# Returns: Package not found or executable not found
```

**Solutions:**

**1. Rebuild package:**
```bash
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_audio --cmake-clean-cache
source install/setup.bash
```

**2. Check entry point in setup.py:**
```bash
grep mcp23017 ~/dev/r2d2/ros2_ws/src/r2d2_audio/setup.py
# Should show:
# 'mcp23017_status_led_node=r2d2_audio.mcp23017_status_led_node:main',
```

**3. Test node directly:**
```bash
python3 ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/mcp23017_status_led_node.py
# Should start in simulation mode if libraries available
```

**4. Check file permissions:**
```bash
chmod +x ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/mcp23017_status_led_node.py
```

### Issue 6: Simulation Mode (No Hardware)

**Symptom:** Node logs show "Running in simulation mode"

**This is OK for development!** The node falls back gracefully when:
- MCP23017 libraries not installed
- MCP23017 hardware not detected at I2C address
- `simulate` parameter set to true

**To run with hardware:**
```bash
# Ensure libraries installed
pip3 list | grep adafruit

# Ensure hardware detected
sudo i2cdetect -y 1

# Launch without simulation
ros2 launch r2d2_audio mcp23017_status_led.launch.py simulate:=false
```

---

## Configuration

### Pin Mapping Configuration

If your LED board has reversed wiring (wire colors don't match LED colors):

**Option 1: Launch argument (temporary):**
```bash
ros2 launch r2d2_audio all_audio_services.launch.py \
  pa0_controls:=blue pa1_controls:=red
```

**Option 2: Update launch file (permanent):**

Edit: `ros2_ws/src/r2d2_audio/launch/all_audio_services.launch.py`

Find the mcp23017_status_led_node parameters and update:
```python
'pa0_controls': 'blue',    # If red wire controls blue LEDs
'pa1_controls': 'red',     # If blue wire controls red LEDs
'pa2_controls': 'green',
'pa3_controls': 'yellow',
```

Then rebuild:
```bash
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_audio
sudo systemctl restart r2d2-audio-notification.service
```

### Gesture Flash Duration

**Default:** 500ms (half second)

**To change:**
```bash
# Shorter flash (300ms)
ros2 launch r2d2_audio mcp23017_status_led.launch.py \
  gesture_flash_duration_ms:=300

# Longer flash (1000ms = 1 second)
ros2 launch r2d2_audio mcp23017_status_led.launch.py \
  gesture_flash_duration_ms:=1000
```

### I2C Address Configuration

**Default:** 0x20 (decimal 32)

**If using multiple MCP23017 boards or address conflict:**

**Hardware:** Set address jumpers on MCP23017 board

| A2 | A1 | A0 | Address (Hex) | Address (Decimal) |
|----|----|----|---------------|-------------------|
| 0  | 0  | 0  | 0x20 | 32 |
| 0  | 0  | 1  | 0x21 | 33 |
| 0  | 1  | 0  | 0x22 | 34 |
| ... | ... | ... | ... | ... |

**Software:** Update launch parameter:
```bash
ros2 launch r2d2_audio mcp23017_status_led.launch.py \
  i2c_address:=33  # For 0x21
```

---

## Maintenance

### LED Replacement

**If an LED fails:**

**Board #1 (Red/Blue/Green SMD LEDs):**
1. Identify failed LED on board
2. Desolder failed SMD LED
3. Solder new SMD LED (same size, check polarity)
4. Or replace entire Board #1 module

**Yellow LED (Simple 3mm LED):**
1. Disconnect from PA3 and GND
2. Check polarity of replacement LED (long leg = voltage)
3. Connect new LED (long leg → PA3, short leg → 220Ω → GND)
4. Test: `python3 tests/led_expansion/test_mcp23017_4led_basic.py`

### Testing After Hardware Changes

**Always run full test sequence:**

```bash
# 1. I2C detection
sudo i2cdetect -y 1

# 2. Basic LED test
cd ~/dev/r2d2/tests/led_expansion
python3 test_mcp23017_4led_basic.py

# 3. ROS2 integration test
cd ~/dev/r2d2/ros2_ws
source install/setup.bash
ros2 launch r2d2_audio mcp23017_status_led.launch.py

# 4. Full system test with minimal_monitor.py
python3 ~/dev/r2d2/tools/minimal_monitor.py
```

### Adding More LEDs

The MCP23017 has 12 unused outputs (PA4-PA7, PB0-PB7) available for expansion:

**Example: Add notification LED on PA4:**

```python
# In mcp23017_status_led_node.py
self.notification_led = mcp.get_pin(4)  # PA4
self.notification_led.direction = digitalio.Direction.OUTPUT

# Control it:
self.notification_led.value = True   # ON
self.notification_led.value = False  # OFF
```

**Future expansion ideas:**
- Battery level LEDs (3 LEDs for low/medium/high)
- WiFi status LED
- Speech active LED
- Camera active LED
- Additional gesture indicators

---

## System Integration

### Replaces Old White LED System

**Old system (before MCP23017):**
- Simple white LED on GPIO 17 (Pin 22)
- Binary state: ON=recognized, OFF=lost/unknown
- Could not show difference between BLUE and GREEN states
- Used 1 GPIO pin

**New system (MCP23017):**
- 4 independent colored LEDs via I2C
- Full status display: RED/BLUE/GREEN (mutually exclusive)
- Gesture flash indicator (yellow, independent)
- Uses 2 GPIO pins (I2C bus, shared with future I2C devices)
- Frees up Pin 22 for audio switch exclusively

### Pin 22 (GPIO 17) Status

**BEFORE:** White LED control + Audio switch (shared, complex)  
**AFTER:** Audio switch only (dedicated, simple)

The audio switch remains on Pin 22 with no changes:
- Pin 1 (3.3V) → 2.2kΩ resistor → Pin 22
- Pin 22 → Switch terminal 1
- Pin 9 (GND) → Switch terminal 2

**Switch function:** Toggle between Bluetooth and PAM8403 speaker

### Integration with Person Recognition

The LED system is tightly integrated with the person recognition state machine:

```
r2d2_perception/image_listener
    ├─ Face detection (Haar Cascade)
    └─ Face recognition (LBPH)
         ↓
r2d2_audio/audio_notification_node
    ├─ Rolling window filter (4 matches in 1.5s)
    ├─ State machine (RED/BLUE/GREEN)
    ├─ RED timer (15s, resets on match)
    └─ /r2d2/audio/person_status (JSON, 10 Hz)
         ↓
r2d2_audio/mcp23017_status_led_node
    ├─ RED status → Red LED ON
    ├─ BLUE status → Blue LED ON
    └─ GREEN status → Green LED ON

r2d2_perception/image_listener
    └─ Gesture recognition (MediaPipe + SVM)
         ↓
/r2d2/perception/gesture_event
         ↓
r2d2_audio/mcp23017_status_led_node
    └─ Any gesture → Yellow LED flash 500ms
```

**For complete status system details, see:** [100_PERCEPTION_STATUS_REFERENCE.md](100_PERCEPTION_STATUS_REFERENCE.md)

---

## Quick Reference Commands

### Hardware Testing
```bash
# Check I2C detection
sudo i2cdetect -y 1

# Test all LEDs
cd ~/dev/r2d2/tests/led_expansion
python3 test_mcp23017_4led_basic.py

# Identify wire mapping
python3 test_mcp23017_led_mapping.py
```

### ROS2 Operations
```bash
# Launch LED node (standalone)
ros2 launch r2d2_audio mcp23017_status_led.launch.py

# Launch full audio system (includes LED node)
ros2 launch r2d2_audio all_audio_services.launch.py

# Monitor with color display
python3 ~/dev/r2d2/tools/minimal_monitor.py

# Test status manually
ros2 topic pub --once /r2d2/audio/person_status std_msgs/String \
  'data: "{\"status\": \"red\", \"person_identity\": \"test\"}"'

# Test gesture manually
ros2 topic pub --once /r2d2/perception/gesture_event std_msgs/String \
  "data: 'fist'"
```

### System Control
```bash
# Rebuild package after changes
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_audio
source install/setup.bash

# Restart service
sudo systemctl restart r2d2-audio-notification.service

# Check status
sudo systemctl status r2d2-audio-notification.service

# View logs
sudo journalctl -u r2d2-audio-notification.service -f
```

---

## Technical Specifications

### MCP23017 Specifications

| Specification | Value | Notes |
|---------------|-------|-------|
| Supply Voltage | 1.8V - 5.5V | Using 3.3V from Jetson |
| Max Current/Pin | 25mA | Our LEDs: ~6mA (safe) |
| Max Total Current | 125mA | Our 4 LEDs: ~24mA (safe) |
| I2C Speed | 100kHz (standard) or 400kHz (fast) | Default works fine |
| I2C Address Range | 0x20 - 0x27 | Default: 0x20 |
| GPIO Pins | 16 total (PA0-PA7, PB0-PB7) | Using 4 (PA0-PA3) |

### LED Current Calculations

**Board #1 (Red + Blue - built-in resistors):**
- Current limiting handled by onboard resistors
- Estimated: 5-10mA per LED
- No external resistors needed

**Green LED (added to Board #1):**
- If resistor from Board #2: Same as Board #1 (5-10mA)
- If no resistor: Add 220Ω → ~5.9mA

**Yellow LED (3mm with 220Ω resistor):**
- Voltage drop: ~2.0V (typical yellow LED)
- Resistor voltage: 3.3V - 2.0V = 1.3V
- Current: 1.3V / 220Ω = 5.9mA
- Safe for MCP23017 (well under 25mA limit)

**Total System Current:**
- All 4 LEDs on: ~24-40mA total
- Well within MCP23017 125mA limit
- Powered from Jetson 3.3V pin via MCP23017 VCC

### Performance Metrics

| Metric | Value | Notes |
|--------|-------|-------|
| Status Update Rate | 10 Hz | Matches person_status topic rate |
| LED Response Time | <100ms | Status change to LED update |
| Gesture Flash Duration | 500ms | Configurable (300-1000ms range) |
| I2C Communication | <5ms | Per LED update |
| CPU Usage | <0.5% | Minimal overhead |
| Memory Usage | ~30MB | Node footprint |

---

## Related Documentation

**System Architecture:**
- [001_ARCHITECTURE_OVERVIEW.md](001_ARCHITECTURE_OVERVIEW.md) - Overall system architecture
- [000_UX_AND_FUNCTIONS.md](000_UX_AND_FUNCTIONS.md) - User experience and capabilities

**Person Recognition:**
- [100_PERCEPTION_STATUS_REFERENCE.md](100_PERCEPTION_STATUS_REFERENCE.md) - Status system reference
- [006_SYSTEM_STATUS_AND_MONITORING.md](006_SYSTEM_STATUS_AND_MONITORING.md) - Monitoring guide

**Hardware:**
- [002_HARDWARE_REFERENCE.md](002_HARDWARE_REFERENCE.md) - Hardware specifications
- [HARDWARE_WHITE_LED_WIRING.md](HARDWARE_WHITE_LED_WIRING.md) - Old LED system (archived)

**Archived Documentation:**
- [_ARCHIVE/265_LED_Installation.md](_ARCHIVE/265_LED_Installation.md) - Old generic guide
- [_ARCHIVE/270_LED_EXPANSION_BOARD_GUIDE.md](_ARCHIVE/270_LED_EXPANSION_BOARD_GUIDE.md) - Old planning doc

---

## Appendix A: Complete Wiring Checklist

Use this checklist during installation:

**I2C Bus (4 wires):**
- [ ] Jetson Pin 3 → MCP23017 SDA
- [ ] Jetson Pin 5 → MCP23017 SCL
- [ ] Jetson Pin 1 → MCP23017 VCC
- [ ] Jetson Pin 6 → MCP23017 GND

**LED Board #1 (3-4 wires):**
- [ ] Black wire → MCP23017 GND
- [ ] Red wire → MCP23017 PA0 or PA1 (verify with test)
- [ ] Blue wire → MCP23017 PA1 or PA0 (verify with test)
- [ ] Green wire → MCP23017 PA2 (after soldering green LED)

**Yellow LED:**
- [ ] Longer leg → MCP23017 PA3
- [ ] Shorter leg → 220Ω resistor
- [ ] Resistor other end → GND (can share with Board #1 black wire)

**Verification:**
- [ ] No short circuits (measure with multimeter)
- [ ] All connections secure
- [ ] LED polarity correct (test with power supply)
- [ ] Resistors installed where needed

---

## Appendix B: Expansion Possibilities

The MCP23017 has 12 unused GPIO pins available:

**Available Outputs:**
- PA4, PA5, PA6, PA7 (4 pins on Port A)
- PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7 (8 pins on Port B)

**Future LED Ideas:**
1. **Battery status LEDs** (3 LEDs: low/medium/high)
2. **System mode LEDs** (fast mode, intelligent mode indicators)
3. **Network status** (connected, disconnected)
4. **Camera active** (recording indicator)
5. **Additional gesture types** (more gesture-specific colors)
6. **Ambient lighting** (decorative LEDs)

**Adding more LEDs is easy:**
1. Connect LED to available PA/PB pin
2. Add pin configuration in node
3. Subscribe to relevant topic
4. Update LED based on topic data

**Multiple MCP23017 Boards:**

Can chain up to 8 boards (128 LEDs!) on same I2C bus:
- Board 1: Address 0x20 (A0=0, A1=0, A2=0)
- Board 2: Address 0x21 (A0=1, A1=0, A2=0)
- Board 3: Address 0x22 (A0=0, A1=1, A2=0)
- ...
- Board 8: Address 0x27 (A0=1, A1=1, A2=1)

---

**Document Status:** ✅ Complete Implementation Guide  
**Last Updated:** January 9, 2026  
**Hardware:** MCP23017 + Pre-assembled Board #1 + Simple LEDs  
**Software:** ROS2 Humble, mcp23017_status_led_node  
**Maintainer:** Severin Leuenberger

---

**End of LED Installation Guide**

