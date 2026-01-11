# R2D2 Status LED System - GPIO + Transistor Implementation
## Direct GPIO Control with NPN Transistors

**Date:** January 11, 2026  
**Status:** ✅ Working Production Implementation  
**Platform:** NVIDIA Jetson AGX Orin 64GB + ROS 2 Humble  
**Hardware:** 3 LEDs + NPN Transistors (2N2222) + Resistors

---

## Executive Summary

The R2D2 status LED system provides real-time visual feedback using **3 LEDs controlled directly via GPIO pins with NPN transistors**. This solution bypasses the I2C hardware issues on Jetson AGX Orin's 40-pin header.

**Working Configuration:**
- **RED LED (Pin 7):** Person recognized status
- **BLUE LED (Pin 11):** No person / idle status  
- **YELLOW LED (Pin 13):** Gesture flash indicator

**Why We Use Transistors:**
- Jetson GPIO can only source ~2-4mA (too dim for LEDs)
- NPN transistors amplify the 3.3V GPIO signal to switch full 5V power
- LEDs get bright 15-20mA current from 5V supply
- GPIO just controls the transistor (low current requirement)

---

## Table of Contents

1. [What We Tried and Why It Failed](#what-we-tried-and-why-it-failed)
2. [The Working Solution](#the-working-solution)
3. [Bill of Materials](#bill-of-materials)
4. [Transistor Basics](#transistor-basics)
5. [Complete Wiring Guide](#complete-wiring-guide)
6. [Perfboard Layout](#perfboard-layout)
7. [Software Installation](#software-installation)
8. [Testing Procedures](#testing-procedures)
9. [ROS2 Integration](#ros2-integration)
10. [Troubleshooting](#troubleshooting)
11. [Future: USB-to-I2C for Servos/Motors](#future-usb-to-i2c-for-servos-motors)

---

## What We Tried and Why It Failed

### Failed Approach: MCP23017 I2C GPIO Expander

**Original Plan:**
- Use MCP23017 I2C chip to control 4 LEDs with just 2 GPIO pins (SDA/SCL)
- Connect via Pins 3/5 (I2C8) on Jetson 40-pin header
- Scale to 16 LEDs with one chip, chain multiple chips if needed

**What We Tried:**

1. **Hardware I2C on Pins 3/5 (I2C8)** - `i2cdetect` showed no external devices
2. **Hardware I2C on Pins 27/28 (I2C2)** - Same result, no detection  
3. **Scanned all I2C buses (0-9)** - None detected MCP23017 or PCA9685
4. **Added FDTOVERLAYS to extlinux.conf** - UEFI bootloader ignored it
5. **Added OVERLAYS + FDT to extlinux.conf** - Still ignored
6. **Used jetson-io tool** - No I2C options available for AGX Orin 40-pin header
7. **Software I2C bit-banging** - Showed 112 phantom devices (floating lines)
8. **Tested boards on Raspberry Pi** - Both MCP23017 and PCA9685 work perfectly

**Root Cause:**
The Jetson AGX Orin's 40-pin header I2C pins are **NOT properly muxed** at the firmware level. The UEFI bootloader in JetPack 6 does not support runtime device tree overlays. The `jetson-io` configuration tool doesn't expose I2C as an option for the 40-pin header.

**Conclusion:**
Hardware I2C on the Jetson AGX Orin 40-pin header is **not usable** without flashing custom firmware (risky and complex). For I2C devices like servo controllers (PCA9685), a **USB-to-I2C adapter** is the recommended solution.

---

## The Working Solution

### GPIO + NPN Transistor Switching

**How It Works:**

```
GPIO Pin (3.3V, ~3mA) → 1kΩ resistor → Transistor BASE
                                             ↓
                                      Transistor turns ON
                                             ↓
                              5V → LED → 220Ω → Transistor COLLECTOR
                                                      ↓
                                              Transistor EMITTER → GND
                                             
Result: LED gets full 15-20mA from 5V (bright!)
        GPIO only provides tiny 3mA signal (safe!)
```

**Key Benefits:**
- ✅ No I2C required - direct GPIO control
- ✅ Bright LEDs - full 5V power
- ✅ Low GPIO current - safe for Jetson
- ✅ Simple circuit - easy to build
- ✅ Proven working - tested and verified

**Limitations:**
- ❌ Uses 1 GPIO pin per LED (vs. 2 pins for 16 LEDs with I2C)
- ❌ Not easily expandable to many LEDs
- ✅ Perfect for 3-4 status indicators (our use case!)

---

## Bill of Materials

### Required Components

| Item | Quantity | Specification | Purpose | Estimated Cost |
|------|----------|---------------|---------|----------------|
| **NPN Transistors** | 3 | 2N2222 or BC337 | Electronic switches | $0.30 |
| **1kΩ Resistors** | 3 | 1/4W, brown-black-red | Base current limiting | $0.15 |
| **220Ω Resistors** | 3 | 1/4W, red-red-brown | LED current limiting | $0.15 |
| **Red LED** | 1 | 5mm or 3mm, any type | Person recognized | $0.10 |
| **Blue LED** | 1 | 5mm or 3mm, any type | No person / idle | $0.10 |
| **Yellow LED** | 1 | 5mm or 3mm, any type | Gesture flash | $0.10 |
| **Perfboard** | 1 | Small (5x7cm) | Circuit assembly | $1.00 |
| **Jumper Wires** | 10 | Female-to-female Dupont | Connections | $2.00 |
| **Wire** | 1m | 22-26 AWG, various colors | Perfboard wiring | $1.00 |

**Total Cost:** ~$5.00 USD (basic electronics components)

### Tools Required

- Soldering iron + solder
- Wire strippers
- Multimeter (for testing)
- Small screwdriver

---

## Transistor Basics

### What is a Transistor?

A transistor is an **electronic switch** controlled by a small signal. Think of it like a water faucet:

```
        FAUCET ANALOGY              TRANSISTOR REALITY
        
    Water pipe (IN)        →        5V through LED (COLLECTOR)
          │                                   │
    ┌─────┴─────┐                      ┌─────┴─────┐
    │  Faucet   │          →           │ Transistor│
    │  Handle   │                      │   (BASE)  │
    └─────┬─────┘                      └─────┬─────┘
          │                                   │
    Water out (drain)      →         Current to GND (EMITTER)
          
    Turn handle = small force    →   GPIO signal = small current
    Water flows = big flow       →   LED current = big current
```

### The Three Pins of 2N2222

Looking at the transistor with the **flat side facing you** and **legs pointing down**:

```
        ┌─────────┐
        │  2N2222 │  ← flat side with markings
        │         │
        └────┬────┘
           ┌┴┐
           │││
           EBC
           │││
           ││└── Right pin = COLLECTOR (C) - current flows IN from LED
           │└─── Middle pin = BASE (B) - GPIO control signal
           └──── Left pin = EMITTER (E) - current flows OUT to ground
```

### How the Transistor Switch Works

**When GPIO is HIGH (3.3V):**
1. Small current (~3mA) flows into BASE through 1kΩ resistor
2. Transistor "opens the valve" between COLLECTOR and EMITTER
3. Large current (~15mA) flows: 5V → LED → Resistor → COLLECTOR → EMITTER → GND
4. **LED lights up bright!**

**When GPIO is LOW (0V):**
1. No current into BASE
2. Transistor "closes the valve"
3. No current flows through LED
4. **LED is off**

---

## Complete Wiring Guide

### Circuit Schematic (Per LED)

```
         Pin 2 or 4 (5V Power)
                │
                │
             [220Ω] ← LED current limiting resistor
                │
                │
              ┌─┴─┐
              │LED│  (long leg = anode = +)
              │ ▼ │  (short leg = cathode = -)
              └─┬─┘
                │ (cathode / short leg)
                │
                ▼
          ┌─────┤ COLLECTOR (C) - right pin
          │     │
          │   ──┼── 2N2222 NPN transistor
          │     │
GPIO Pin ──[1kΩ]──┤ BASE (B) - middle pin
          │     │
          │     ┤ EMITTER (E) - left pin
          │     │
          └─────┴───── Pin 6 (GND)
```

### Pin-by-Pin Wiring Tables

#### RED LED Circuit (Person Recognized)

| Connection | From | To | Notes |
|------------|------|---|-------|
| 1. Power | Jetson Pin 2 (5V) | 220Ω resistor | Common 5V rail |
| 2. LED Anode | 220Ω resistor | RED LED long leg | Positive side |
| 3. LED Cathode | RED LED short leg | Transistor C (right) | Through transistor |
| 4. Base Control | Jetson Pin 7 (GPIO) | 1kΩ resistor | Control signal |
| 5. Base | 1kΩ resistor | Transistor B (middle) | Switch control |
| 6. Emitter | Transistor E (left) | Jetson Pin 6 (GND) | Common ground |

#### BLUE LED Circuit (No Person / Idle)

| Connection | From | To | Notes |
|------------|------|---|-------|
| 1. Power | Jetson Pin 2 (5V) | 220Ω resistor | Common 5V rail |
| 2. LED Anode | 220Ω resistor | BLUE LED long leg | Positive side |
| 3. LED Cathode | BLUE LED short leg | Transistor C (right) | Through transistor |
| 4. Base Control | Jetson Pin 11 (GPIO) | 1kΩ resistor | Control signal |
| 5. Base | 1kΩ resistor | Transistor B (middle) | Switch control |
| 6. Emitter | Transistor E (left) | Jetson Pin 6 (GND) | Common ground |

#### YELLOW LED Circuit (Gesture Flash)

| Connection | From | To | Notes |
|------------|------|---|-------|
| 1. Power | Jetson Pin 2 (5V) | 220Ω resistor | Common 5V rail |
| 2. LED Anode | 220Ω resistor | YELLOW LED long leg | Positive side |
| 3. LED Cathode | YELLOW LED short leg | Transistor C (right) | Through transistor |
| 4. Base Control | Jetson Pin 13 (GPIO) | 1kΩ resistor | Control signal |
| 5. Base | 1kΩ resistor | Transistor B (middle) | Switch control |
| 6. Emitter | Transistor E (left) | Jetson Pin 6 (GND) | Common ground |

---

## Perfboard Layout

### Complete System on Perfboard

```
    PERFBOARD LAYOUT (top view)
    ═══════════════════════════════════════════════════
    
    5V RAIL ═══════════════════════════ (red wire from Pin 2)
        │       │       │
        ▼       ▼       ▼
      [220]   [220]   [220]    ← LED resistors
        │       │       │
        ▼       ▼       ▼
      RED     BLUE    YELLOW    ← LEDs (standing up)
      LED     LED     LED
        │       │       │
        ▼       ▼       ▼
     ┌──C    ┌──C    ┌──C       ← Transistor collectors
     │  B    │  B    │  B       ← Transistors (flat on board)
     │  E    │  E    │  E
     │  │    │  │    │  │
     │  ▲    │  ▲    │  ▲       ← 1kΩ base resistors
     │[1k]   │[1k]   │[1k]
     │  │    │  │    │  │
     │  │    │  │    │  │
    GND RAIL ═══════════════════ (black wire to Pin 6)
        │       │       │
        ▼       ▼       ▼
      Pin 7   Pin 11  Pin 13    ← GPIO control wires
      (RED)   (BLUE)  (YELLOW)
```

### Physical Wiring from Board to Jetson

| Wire | From Perfboard | To Jetson Pin | Function |
|------|---------------|---------------|----------|
| Red (thick) | 5V rail | Pin 2 (5V) | Power for all LEDs |
| Black (thick) | GND rail | Pin 6 (GND) | Common ground |
| Red (thin) | 1kΩ resistor #1 | Pin 7 | RED LED control |
| Blue (thin) | 1kΩ resistor #2 | Pin 11 | BLUE LED control |
| Yellow (thin) | 1kΩ resistor #3 | Pin 13 | YELLOW LED control |

**Total: 5 wires** from perfboard to Jetson

---

## Software Installation

### Step 1: Build ROS2 Package

```bash
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_audio
source install/setup.bash
```

### Step 2: Verify GPIO Access

```bash
# Check if user is in gpio group
groups | grep gpio

# If not, add user to gpio group
sudo usermod -aG gpio $USER
# Then log out and back in
```

---

## Testing Procedures

### Test 1: Quick Single LED Test

Test each LED individually:

```bash
cd ~/dev/r2d2/tests/led_expansion

# Test RED LED
sudo python3 test_single_led.py red

# Test BLUE LED  
sudo python3 test_single_led.py blue

# Test YELLOW LED
sudo python3 test_single_led.py yellow

# Test all LEDs
sudo python3 test_single_led.py all
```

Press Ctrl+C to turn off and exit.

### Test 2: Full LED Test Suite

Run comprehensive tests:

```bash
sudo python3 test_gpio_transistor_leds.py
```

This will:
1. Test each LED individually
2. Test all LEDs together
3. Run sequence pattern
4. Run blink test
5. Preview status patterns

### Test 3: ROS2 Integration Test

Test the LED node with ROS2:

```bash
# Terminal 1: Start LED node
ros2 run r2d2_audio gpio_status_led_node

# Terminal 2: Send test commands
cd ~/dev/r2d2/ros2_ws && source install/setup.bash

# Test RED status (person recognized)
ros2 topic pub --once /r2d2/audio/person_status std_msgs/String \
  'data: "{\"status\": \"red\", \"person_identity\": \"severin\"}"'

# Test BLUE status (no person)
ros2 topic pub --once /r2d2/audio/person_status std_msgs/String \
  'data: "{\"status\": \"blue\", \"person_identity\": \"no_person\"}"'

# Test gesture flash
ros2 topic pub --once /r2d2/perception/gesture_event std_msgs/String \
  "data: 'fist'"
```

### Test 4: Auto-Start Verification

Verify the service auto-starts on boot:

```bash
# Restart the audio service (includes LED node)
sudo systemctl restart r2d2-audio-notification.service

# Check status
sudo systemctl status r2d2-audio-notification.service

# Look for gpio_status_led_node in output
# Should show "active (running)"
```

---

## ROS2 Integration

### Node Architecture

**Node Name:** `gpio_status_led_node`  
**Package:** `r2d2_audio`  
**File:** `ros2_ws/src/r2d2_audio/r2d2_audio/gpio_status_led_node.py`

**Subscriptions:**

| Topic | Type | Rate | Purpose |
|-------|------|------|---------|
| `/r2d2/audio/person_status` | String (JSON) | 10 Hz | Status updates (RED/BLUE) |
| `/r2d2/perception/gesture_event` | String | Event | Gesture flash trigger (YELLOW) |

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enabled` | bool | true | Enable/disable LED control |
| `simulate` | bool | false | Run in simulation mode (no GPIO) |
| `gesture_flash_duration_ms` | int | 500 | Yellow LED flash duration (ms) |

### Launch Files

**Standalone LED node (for testing):**
```bash
ros2 launch r2d2_audio gpio_status_led.launch.py
```

**Full audio system (production):**
```bash
ros2 launch r2d2_audio all_audio_services.launch.py
```

The full launch file starts:
1. `audio_notification_node` - Status state machine
2. `gpio_status_led_node` - LED control (auto-included)
3. `database_logger_node` - Event logging

### Systemd Service

The LED node is automatically started as part of the audio notification service:

**Service:** `r2d2-audio-notification.service`  
**Status:** Auto-starts on boot

```bash
# Check service status
sudo systemctl status r2d2-audio-notification.service

# View logs
sudo journalctl -u r2d2-audio-notification.service -f

# Filter for LED node
sudo journalctl -u r2d2-audio-notification.service -f | grep gpio_status_led
```

---

## Troubleshooting

### Issue 1: LEDs Not Responding

**Symptom:** Test script runs but LEDs don't light up

**Solutions:**

**1. Check LED polarity:**
```
Correct:   5V → 220Ω → Longer leg → LED → Shorter leg → Transistor C
Wrong:     5V → 220Ω → Shorter leg → LED → (won't light)

Fix: Reverse the LED legs
```

**2. Check transistor orientation:**
- Flat side of 2N2222 should face you
- E-B-C from left to right
- Verify with multimeter in diode mode

**3. Check resistor values:**
```bash
# Use multimeter to verify:
Base resistor: Should read ~1000Ω (1kΩ)
LED resistor: Should read ~220Ω
```

**4. Test with multimeter:**
```bash
# When LED should be ON:
# Measure GPIO pin voltage: should be ~3.3V
# Measure transistor base: should be ~2.6V
# Measure transistor collector: should be ~2.0V (LED forward voltage)
```

### Issue 2: Dim LEDs

**Symptom:** LEDs light up but very dim

**Cause:** Likely using 3.3V instead of 5V

**Solution:**
```bash
# Verify 5V connection:
# Use multimeter on perfboard 5V rail
# Should read 4.8-5.2V
# If reading 3.3V, you're connected to Pin 1 instead of Pin 2/4
```

### Issue 3: GPIO Permission Denied

**Symptom:**
```
RuntimeError: Please run as root (sudo) or add user to gpio group
```

**Solution:**
```bash
# Add user to gpio group
sudo usermod -aG gpio $USER

# Log out and back in (or reboot)
sudo reboot
```

### Issue 4: ROS2 Node Not Starting

**Symptom:**
```
Package 'r2d2_audio' not found
```

**Solution:**
```bash
# Rebuild package
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_audio --cmake-clean-cache
source install/setup.bash

# Verify node is installed
ros2 pkg executables r2d2_audio | grep gpio_status_led_node
```

### Issue 5: Wrong LED Behavior

**Symptom:** RED status turns on BLUE LED or vice versa

**Cause:** Wires connected to wrong GPIO pins

**Solution:**
Check your wiring against the plan:
- Pin 7 should control RED LED
- Pin 11 should control BLUE LED
- Pin 13 should control YELLOW LED

---

## Future: USB-to-I2C for Servos/Motors

### Why USB-to-I2C?

Since the Jetson AGX Orin's 40-pin header I2C is broken, use a **USB-to-I2C adapter** for I2C devices like:
- PCA9685 (16-channel PWM for servos)
- Other I2C sensors and controllers

### Recommended Adapters

| Adapter | Price | Features | Linux Support |
|---------|-------|----------|---------------|
| **Adafruit FT232H** | ~$15 | USB to I2C/SPI/GPIO | Excellent |
| **CJMCU FT232H** | ~$8 | Clone of Adafruit | Good |
| **CH341A** | ~$5 | Basic I2C/SPI | Fair |

**Recommended:** Adafruit FT232H (best Linux support)

### Quick Setup (FT232H Example)

```bash
# Install libraries
pip3 install pyftdi adafruit-circuitpython-pca9685

# Test I2C scan
python3 << 'EOF'
from pyftdi.i2c import I2cController
i2c = I2cController()
i2c.configure('ftdi://ftdi:232h/1')
devices = i2c.scan()
print(f"Found {len(devices)} I2C device(s): {[hex(addr) for addr in devices]}")
EOF

# Should detect PCA9685 at 0x40
```

This provides a **reliable I2C bus via USB**, completely bypassing the Jetson's 40-pin header issues.

---

## Appendix: What We Learned

### I2C Issues on Jetson AGX Orin

**Key Findings:**
1. The 40-pin header I2C pins (3/5 and 27/28) are NOT properly muxed
2. UEFI bootloader in JetPack 6 doesn't support runtime device tree overlays
3. `jetson-io` tool doesn't expose I2C configuration for AGX Orin
4. Software I2C bit-banging shows floating lines (112 phantom devices)
5. Same I2C boards work perfectly on Raspberry Pi (hardware not defective)

**For Future Reference:**
- Don't rely on 40-pin header I2C on Jetson AGX Orin
- Use USB-to-I2C adapters for I2C peripherals  
- Direct GPIO works fine for simple on/off control
- Consider this when designing hardware interfaces

### Files Created/Modified

**ROS2 Nodes:**
- `ros2_ws/src/r2d2_audio/r2d2_audio/gpio_status_led_node.py` - GPIO LED controller
- `ros2_ws/src/r2d2_audio/launch/gpio_status_led.launch.py` - Dedicated launch file
- `ros2_ws/src/r2d2_audio/launch/all_audio_services.launch.py` - Updated to use GPIO node

**Test Scripts:**
- `tests/led_expansion/test_gpio_transistor_leds.py` - Main LED test suite
- `tests/led_expansion/test_single_led.py` - Quick single LED test
- `tests/led_expansion/test_gpio_toggle.py` - GPIO voltage verification

**Archived (I2C attempts that failed):**
- `_ARCHIVE/270_LED_INSTALLATION_MCP23017_FAILED.md` - Old MCP23017 documentation
- `tests/led_expansion/test_software_i2c.py` - Software I2C attempt
- `tests/led_expansion/test_mcp23017_*.py` - MCP23017 test scripts

---

## Quick Reference

### Hardware Specs

| Component | Value | Purpose |
|-----------|-------|---------|
| RED LED | Pin 7 | Person recognized |
| BLUE LED | Pin 11 | No person / idle |
| YELLOW LED | Pin 13 | Gesture flash |
| Base Resistors | 1kΩ | Current limiting to transistor base |
| LED Resistors | 220Ω | LED current limiting |
| Transistors | 2N2222 NPN | Electronic switches |
| Power | 5V (Pin 2/4) | LED power supply |
| Ground | GND (Pin 6) | Common ground |

### Common Commands

```bash
# Test hardware
sudo python3 ~/dev/r2d2/tests/led_expansion/test_gpio_transistor_leds.py

# Start LED node
ros2 run r2d2_audio gpio_status_led_node

# Start full system
ros2 launch r2d2_audio all_audio_services.launch.py

# Check service status
sudo systemctl status r2d2-audio-notification.service

# View logs
sudo journalctl -u r2d2-audio-notification.service -f
```

---

**Document Status:** ✅ Complete Working Implementation  
**Last Updated:** January 10, 2026  
**Hardware:** 3 LEDs + NPN Transistors (2N2222)  
**Software:** ROS2 Humble, gpio_status_led_node  
**Maintainer:** Severin Leuenberger

---

**End of LED Installation Guide**


