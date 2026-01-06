# R2D2 Camera Pan/Tilt System - Final Implementation Plan

**Date:** January 6, 2026  
**Last Updated:** January 6, 2026  
**Status:** BLOCKED - Awaiting PCA9685 Hardware Delivery  
**Branch:** Working on `main`  
**Author:** AI Agent (Claude) + Severin Leuenberger

---

## Executive Summary

**Critical Discovery:** During implementation testing on January 6, 2026, we discovered that **most Jetson AGX Orin GPIO pins cannot output voltage** due to pinmux configuration in JetPack R36.4.7. Only Pin 13 (PWM01) reliably outputs voltage for motor speed control. Direct GPIO control for motor direction is not viable.

**Solution:** Use the **PCA9685 I2C PWM controller** (already ordered from BerryBase, 2 units for CHF 10.92) to provide both PWM and direction signals via the I2C bus. This solves the GPIO limitation and provides a cleaner, more scalable solution for all three motors (pan, left wheel, right wheel) plus the tilt servo.

**Goal:** When a human face is detected, R2D2 actively tracks that face by adjusting head pan (via dome motor) and camera tilt (via servo) to keep the face centered. Movement should feel intentional, calm, responsive, and "alive."

**Current Status:** Hardware testing complete, PCA9685 solution identified, awaiting hardware delivery.

---

## Table of Contents

1. [Hardware Discovery & Debugging Session](#1-hardware-discovery--debugging-session)
2. [GPIO Pinmux Limitation (Critical Finding)](#2-gpio-pinmux-limitation-critical-finding)
3. [Solution: PCA9685 I2C PWM Controller](#3-solution-pca9685-i2c-pwm-controller)
4. [Current Hardware Status](#4-current-hardware-status)
5. [PCA9685 Wiring Reference](#5-pca9685-wiring-reference)
6. [Implementation Plan (When PCA9685 Arrives)](#6-implementation-plan-when-pca9685-arrives)
7. [Working Code to Preserve](#7-working-code-to-preserve)
8. [Lessons Learned](#8-lessons-learned)
9. [ROS 2 Integration](#9-ros-2-integration)
10. [Rollback Strategy](#10-rollback-strategy)

---

## 1. Hardware Discovery & Debugging Session

### 1.1 What We Tested (January 6, 2026)

**Hardware Setup:**
- Pololu G2 High-Power Motor Driver 24v21 (wired to Jetson and battery)
- DeAgostini DC motor with Hall encoder (pan/dome motor)
- Jetson AGX Orin 40-pin GPIO header
- 14.8V 4S LiPo battery

**Testing Sequence:**
1. ✅ **Motor Power Test:** Connected motor to Pololu driver, verified motor spins at variable speeds
2. ✅ **PWM Control (Pin 13):** Successfully controlled motor speed via GPIO PWM
3. ❌ **Direction Control (Pin 29, 31, 15, 16, 18):** Failed - no voltage output on DIR pins

### 1.2 What Works

| Component | Status | Evidence |
|-----------|--------|----------|
| **Pin 13 (GPIO32/PWM01)** | ✅ WORKS | Motor spins at variable speeds (20%, 40%, 60%, 80%) |
| **Motor speed control** | ✅ WORKS | Confirmed via multimeter and motor rotation |
| **Motor bi-directional** | ✅ WORKS | Tested manually with power supply (clockwise and counterclockwise) |
| **Pololu G2 driver (power side)** | ✅ WORKS | Motor runs smoothly, no overheating, clean operation |
| **Common ground** | ✅ VERIFIED | Battery GND and Jetson GND connected (measured with multimeter) |
| **Battery power** | ✅ VERIFIED | 14.8V nominal, sufficient current for motor |

### 1.3 What Does NOT Work

| Component | Attempted Pins | Measured Voltage | Root Cause |
|-----------|---------------|------------------|------------|
| **Direction control** | Pin 29 (CAN0_DIN) | 0V (always) | Special function pin, not GPIO capable |
| **Direction control** | Pin 31 (CAN0_DOUT) | 3.3V (stuck HIGH) | Special function pin, not GPIO capable |
| **Direction control** | Pin 15 (GPIO27) | 0V (no toggle) | Pinmux: tristate/input-only mode |
| **Direction control** | Pin 16 (GPIO08) | 0V (initially worked, then stopped) | Pinmux: tristate/input-only mode |
| **Direction control** | Pin 18 (GPIO35) | 0V (no toggle) | Pinmux: tristate/input-only mode |

**Key Observation:** Software commands (`GPIO.output(pin, HIGH)`) reported success and read back as HIGH, but multimeter showed **0V on the physical pin**. This indicates a hardware/firmware-level issue, not a software bug.

---

## 2. GPIO Pinmux Limitation (Critical Finding)

### 2.1 Root Cause Analysis

After extensive debugging and system diagnostics, we identified the root cause:

**JetPack R36.4.7 Pinmux Configuration:**
- Uses character device GPIO (`/dev/gpiochip*`) instead of legacy sysfs (`/sys/class/gpio/`)
- Pin 13 works because it's configured as **PWM01** (output-enabled by default in device tree)
- Most other pins are in **tristate or input-only mode** at the pinmux/pad level
- Even when configured as GPIO outputs in software, the physical output buffer is not enabled

**Technical Details:**

```
Pin Muxing Configuration (from NVIDIA jetson-io):
┌────────────────────────────────────────────────┐
│ sfio | input_en | tristate | pin mode          │
├────────────────────────────────────────────────┤
│    0 |      0/1 |      0/1 | disable           │
│    1 |      0/1 |        0 | enable (output)   │
│    1 |        1 |        1 | enable (input)    │
│    1 |        0 |        1 | disable           │
└────────────────────────────────────────────────┘

Pin 13 (PWM01):  sfio=1, tristate=0 → OUTPUT ENABLED ✅
Pin 16 (GPIO08): sfio=1, tristate=1 → OUTPUT DISABLED ❌
Pin 18 (GPIO35): sfio=1, tristate=1 → OUTPUT DISABLED ❌
```

**GPIO Chip Mapping:**
- Pin 13 (working): `gpiochip0` (main GPIO controller)
- Pin 16 (broken): `gpiochip1` (AON - Always-On chip)
- Pin 18 (broken): `gpiochip0` (main GPIO controller, but output disabled)

### 2.2 Why Pin 13 Works

Pin 13 is special because:
1. It's configured as `PWM01` (hardware PWM) in the default device tree
2. Output buffer is enabled by default for PWM functionality
3. Even when used as GPIO, the output path remains active
4. This is why motor speed control works perfectly

### 2.3 Attempted Fixes (All Failed)

We attempted several approaches to enable other GPIO pins:

1. ❌ **Different pins:** Tried 29, 31, 15, 16, 18 - none worked
2. ❌ **Software GPIO modes:** BCM vs BOARD mode - no difference
3. ❌ **Library switching:** `Jetson.GPIO` vs `RPi.GPIO` - no difference
4. ❌ **Pin cleanup and re-initialization:** No effect on output
5. ⚠️ **NVIDIA jetson-io tool:** Could reconfigure pinmux, but requires:
   - Device tree modification
   - System reboot
   - Risk of breaking other functions
   - Complex and error-prone

### 2.4 Why Direct GPIO Control is Not Viable

**Conclusion:** Direct GPIO motor control (PWM + DIR pins) is **not feasible** on Jetson AGX Orin with current JetPack configuration because:

1. Only 1 confirmed working output pin (Pin 13 for PWM)
2. Direction control requires at least 1 additional output pin per motor
3. Three motors (pan + 2 wheels) would require 6 GPIO outputs (3× PWM + 3× DIR)
4. Encoders need 6 GPIO inputs (working, but unrelated to output issue)
5. Reconfiguring pinmux is risky and may break existing functionality

**We need a different approach.**

---

## 3. Solution: PCA9685 I2C PWM Controller

### 3.1 Why PCA9685 Solves the Problem

The **PCA9685** is a 16-channel, 12-bit PWM controller that communicates via I2C bus. This solves our GPIO limitation perfectly:

**Advantages:**
- ✅ **Only uses 2 pins:** SDA (Pin 3) and SCL (Pin 5) on I2C bus
- ✅ **I2C is functional:** No pinmux issues with I2C pins (already used for LED expansion board plan)
- ✅ **16 PWM channels:** Enough for all 3 motors (6 channels) + tilt servo (1 channel) + 9 spare
- ✅ **Hardware PWM:** Very precise timing, no jitter or CPU load
- ✅ **5V logic output:** Can drive servo directly (tilt servo requires 5V logic)
- ✅ **Industry standard:** Well-supported by Adafruit libraries and ROS 2
- ✅ **Scalable:** Can chain up to 62 PCA9685 boards on one I2C bus (if needed)

**Why This is Better Than Direct GPIO:**
1. **Simpler wiring:** 2 I2C wires instead of 6+ GPIO wires
2. **More reliable:** No pinmux configuration issues
3. **Future-proof:** Easy to add more motors or servos
4. **Proven solution:** Same chip used in hundreds of robotics projects
5. **Same approach as LED expansion:** Consistent architecture (I2C expanders for everything)

### 3.2 Order Status

**Ordered:** January 5, 2026 (before discovering the GPIO issue - lucky!)  
**Vendor:** BerryBase (berrybase.ch)  
**Product:** BerryBase 16-Kanal PWM Servo Treiber Board PCA9685  
**Link:** https://www.berrybase.ch/berrybase-16-kanal-pwm-servo-treiber-board-pca9685-i2c-12bit-1-6khz-3-3-5v  
**Quantity:** 2 units (one for motors, one spare/future expansion)  
**Total Cost:** CHF 10.92  
**Invoice Reference:** 966980  
**Expected Delivery:** ~5-7 days

**Original Purpose:** This was originally ordered for the tilt servo (to solve the 3.3V→5V logic level issue). Now it will also solve the GPIO output limitation for motor direction control - a perfect dual-purpose solution!

### 3.3 PCA9685 Specifications

**Key Features:**
- 16 independently controllable PWM outputs
- 12-bit resolution (4096 steps)
- Adjustable PWM frequency: 24 Hz to 1526 Hz (default 200 Hz for servos)
- I2C interface (address: 0x40 to 0x7F, default 0x40)
- 3.3V or 5V logic compatible (Jetson 3.3V I2C works)
- External 5V power supply for servo/motor V+ rail (separate from logic power)
- Low power consumption: ~10mA (logic), servos/motors powered externally

**Perfect for R2D2:**
- Tilt servo: 5V logic PWM signal required ✅
- Motor DIR signals: Can use PWM channels as digital outputs (0% = LOW, 100% = HIGH) ✅
- Motor PWM signals: Variable PWM for speed control ✅

---

## 4. Current Hardware Status

### 4.1 Pan Motor System (Dome Rotation)

**Pololu G2 24v21 Driver:**
- ✅ **Wired and tested:** Power side (VM, GND, MOTOR A/B) fully functional
- ✅ **Motor runs smoothly:** No noise, no overheating, clean operation
- ⚠️ **Logic side partially working:** PWM works (Pin 13), DIR does not work (GPIO limitation)
- 🔌 **Current wiring (to be modified when PCA9685 arrives):**
  - VM → Battery +14.8V
  - GND → Battery GND (common with Jetson GND)
  - PWM → Jetson Pin 13 (currently works, will move to PCA9685 CH0)
  - DIR → Currently not connected (will connect to PCA9685 CH1)
  - SLP → Jetson Pin 1 (3.3V) - tied HIGH
  - MOTOR A → Motor Red wire
  - MOTOR B → Motor Black wire

**Motor:**
- ✅ **Tested and working:** Spins at variable speeds (20%, 40%, 60%, 80%)
- ✅ **Bi-directional confirmed:** Manually tested with power supply (CW and CCW)
- ✅ **Smooth operation:** No mechanical binding, dome rotates freely
- ✅ **Encoder installed:** 6-pin connector (VCC, GND, Ch A, Ch B, possibly Index)

**Encoder:**
- 🔌 **Wired but untested:**
  - VCC (Red) → Jetson Pin 2 or 4 (5V)
  - GND (Black) → Jetson Pin 6 (GND)
  - Ch A (Yellow) → [4.7kΩ pull-down] → Jetson Pin 16 (GPIO23)
  - Ch B (Green) → [4.7kΩ pull-down] → Jetson Pin 18 (GPIO24)
- ⚠️ **Not yet tested:** Encoder inputs should work (GPIO inputs are functional), but not verified

**Home Sensor (IR Reflective):**
- 🔌 **Wired but untested:**
  - VCC (Red) → Jetson Pin 2 or 4 (5V)
  - GND (Black) → Jetson Pin 6 (GND)
  - Signal (Blue) → Jetson Pin 23 (GPIO11)
- ⚠️ **Not yet tested:** Sensor input should work (GPIO inputs are functional), but not verified

### 4.2 Tilt Servo System

**Servo:**
- ✅ **Confirmed working:** Tested with external servo tester (5V logic)
- ❌ **Cannot use Jetson GPIO directly:** Jetson outputs 3.3V, servo requires 5V logic
- ⏳ **Waiting for PCA9685:** Will provide 5V logic PWM signal

**Power:**
- ✅ **5V BEC available:** External 5V power supply for servo
- ✅ **Common ground verified:** Jetson GND and servo power GND connected

### 4.3 Wheel Motors (Future Phase 3)

**Status:** Hardware available (2× DeAgostini motors + 2× Pololu G2 24v21 drivers), not yet wired

**Plan:** Use PCA9685 for motor control when implementing wheel motors:
- Left wheel: PCA9685 CH2 (PWM), CH3 (DIR)
- Right wheel: PCA9685 CH4 (PWM), CH5 (DIR)
- See `400_WHEEL_MOTORS_SYSTEM_REFERENCE.md` for full specification

---

## 5. PCA9685 Wiring Reference

### 5.1 PCA9685 to Jetson Connection

```
PCA9685 Board             Jetson AGX Orin 40-Pin Header
═══════════════           ═══════════════════════════════
VCC  ─────────────────►   Pin 1 or 17 (3.3V logic power)
GND  ─────────────────►   Pin 6, 9, 14, 20, 25, 30, 34, or 39 (GND)
SDA  ─────────────────►   Pin 3 (I2C5_DAT)
SCL  ─────────────────►   Pin 5 (I2C5_CLK)
V+   ─────────────────►   External 5V BEC (servo/motor power rail)

Important Notes:
- VCC is logic power (3.3V from Jetson)
- V+ is servo/motor power (5V from external BEC)
- V+ and VCC are separate power rails
- GND must be common between Jetson, PCA9685, and 5V BEC
```

### 5.2 PCA9685 Channel Allocation

| Channel | Function | Connects To | Notes |
|---------|----------|-------------|-------|
| **CH0** | Pan Motor PWM | Pololu G2 "PWM" pin | Variable duty cycle (0-100%) for speed |
| **CH1** | Pan Motor DIR | Pololu G2 "DIR" pin | 0% = LOW (reverse), 100% = HIGH (forward) |
| **CH2** | Tilt Servo PWM | Servo signal wire (orange) | 1-2ms pulse width (servo control) |
| **CH3** | Left Wheel PWM | Pololu G2 #2 "PWM" pin | Phase 3 - Future |
| **CH4** | Left Wheel DIR | Pololu G2 #2 "DIR" pin | Phase 3 - Future |
| **CH5** | Right Wheel PWM | Pololu G2 #3 "PWM" pin | Phase 3 - Future |
| **CH6** | Right Wheel DIR | Pololu G2 #3 "DIR" pin | Phase 3 - Future |
| **CH7-15** | Reserved | Future expansion | 9 spare channels |

### 5.3 Complete System Wiring (Pan Motor + Tilt Servo)

```
═══════════════════════════════════════════════════════════════
COMPLETE PAN/TILT WIRING WITH PCA9685
═══════════════════════════════════════════════════════════════

JETSON 40-PIN HEADER:
    │
    ├──Pin 3 (I2C5_DAT)───────────────► PCA9685 "SDA"
    ├──Pin 5 (I2C5_CLK)───────────────► PCA9685 "SCL"
    ├──Pin 1 or 17 (3.3V)─────────────► PCA9685 "VCC" (logic power)
    ├──Pin 6/9/14/etc (GND)───────────► PCA9685 "GND"
    │
    ├──Pin 16 (GPIO23/SPI1_MISO)──────► Encoder Ch A (via 4.7kΩ pull-down)
    ├──Pin 18 (GPIO24/SPI1_MOSI)──────► Encoder Ch B (via 4.7kΩ pull-down)
    ├──Pin 23 (GPIO11/UART1_RTS)──────► Home Sensor "L" signal
    │
    ├──Pin 2 or 4 (5V)────────────────► Encoder VCC + Home Sensor VCC
    └──Pin 6/9/14/etc (GND)───────────► Encoder GND + Home Sensor GND

EXTERNAL 5V BEC:
    │
    ├──5V OUT ────────────────────────► PCA9685 "V+" (servo power rail)
    └──GND ───────────────────────────► PCA9685 "GND" (common ground)

PCA9685 BOARD:
    │
    ├──CH0 (Pan Motor PWM)────────────► Pololu G2 "PWM" pin
    ├──CH1 (Pan Motor DIR)────────────► Pololu G2 "DIR" pin
    └──CH2 (Tilt Servo PWM)───────────► Servo signal wire (orange)

POLOLU G2 DRIVER:
    │
    ├──VM ────────────────────────────► Battery +14.8V (via fuse/switch)
    ├──GND ───────────────────────────► Battery GND (common with Jetson GND)
    ├──PWM ───────────────────────────► PCA9685 CH0 (from above)
    ├──DIR ───────────────────────────► PCA9685 CH1 (from above)
    ├──SLP ───────────────────────────► Jetson Pin 1 or 17 (3.3V) - tie HIGH
    │
    ├──MOTOR A ───────────────────────► Pan Motor Red wire (+)
    └──MOTOR B ───────────────────────► Pan Motor Black wire (-)

TILT SERVO:
    │
    ├──Signal (Orange)────────────────► PCA9685 CH2 (from above)
    ├──V+ (Red)───────────────────────► 5V BEC (from PCA9685 V+ rail)
    └──GND (Brown/Black)──────────────► GND (common)

═══════════════════════════════════════════════════════════════
CRITICAL NOTES:
═══════════════════════════════════════════════════════════════
✅ I2C bus is functional (no pinmux issues)
✅ PCA9685 provides 5V logic output (perfect for servo)
✅ Battery and Jetson MUST share common ground
✅ Encoder and home sensor inputs should work (GPIO inputs functional)
✅ PCA9685 V+ is separate from VCC (servo power vs logic power)
⚠️  Test I2C communication first (i2cdetect -y 1 or -y 8)
⚠️  PCA9685 default address: 0x40 (can be changed via solder jumpers)
═══════════════════════════════════════════════════════════════
```

---

## 6. Implementation Plan (When PCA9685 Arrives)

### Phase 1: PCA9685 Hardware Setup & Testing

**Duration:** ~30 minutes  
**Prerequisites:** PCA9685 boards arrived, basic tools ready

**Steps:**

1. **Install Python Libraries:**
   ```bash
   pip3 install adafruit-circuitpython-pca9685 adafruit-circuitpython-servokit
   ```

2. **Wire PCA9685 to Jetson:**
   - VCC → Pin 1 or 17 (3.3V)
   - GND → Pin 6 (GND)
   - SDA → Pin 3 (I2C5_DAT)
   - SCL → Pin 5 (I2C5_CLK)
   - V+ → External 5V BEC positive
   - V+ GND → Common ground

3. **Test I2C Communication:**
   ```bash
   # Find I2C bus (usually 1 or 8 on Jetson AGX Orin)
   ls /dev/i2c-*
   
   # Scan for PCA9685 (should show 0x40)
   i2cdetect -y 1   # or i2cdetect -y 8
   ```

4. **Test Basic PCA9685 Output:**
   ```python
   from adafruit_pca9685 import PCA9685
   import board
   import busio
   
   i2c = busio.I2C(board.SCL, board.SDA)
   pca = PCA9685(i2c, address=0x40)
   pca.frequency = 1000  # 1kHz for motor PWM
   
   # Test channel 0 (should output PWM)
   pca.channels[0].duty_cycle = 0x7FFF  # 50% duty cycle
   
   # Test channel 1 as digital output (DIR signal)
   pca.channels[1].duty_cycle = 0xFFFF  # 100% = HIGH
   pca.channels[1].duty_cycle = 0x0000  # 0% = LOW
   ```

5. **Verify with Multimeter:**
   - Measure CH0 output with multimeter (should show ~2.5V average at 50% duty)
   - Measure CH1 output (should toggle between 0V and 5V)

### Phase 2: Pan Motor Control via PCA9685

**Duration:** ~1 hour  
**Prerequisites:** Phase 1 complete, motor and encoder wired

**Steps:**

1. **Move Pololu Wiring from Jetson to PCA9685:**
   - Disconnect DIR wire from Jetson (was not working anyway)
   - Disconnect PWM wire from Jetson Pin 13
   - Connect Pololu PWM → PCA9685 CH0
   - Connect Pololu DIR → PCA9685 CH1

2. **Create `pan_motor_driver.py`:**
   ```python
   from adafruit_pca9685 import PCA9685
   import board
   import busio
   
   class PanMotorDriver:
       def __init__(self, pwm_channel=0, dir_channel=1):
           i2c = busio.I2C(board.SCL, board.SDA)
           self.pca = PCA9685(i2c, address=0x40)
           self.pca.frequency = 1000  # 1kHz for DC motor
           
           self.pwm_channel = pwm_channel
           self.dir_channel = dir_channel
       
       def set_speed(self, speed):
           """Set motor speed (-1.0 to 1.0, negative = reverse)"""
           direction = 1 if speed >= 0 else 0
           speed_abs = abs(speed)
           
           # Set direction (0% = LOW, 100% = HIGH)
           self.pca.channels[self.dir_channel].duty_cycle = 0xFFFF if direction else 0x0000
           
           # Set PWM speed (0-100%)
           duty = int(speed_abs * 0xFFFF)
           self.pca.channels[self.pwm_channel].duty_cycle = duty
       
       def stop(self):
           """Stop motor immediately"""
           self.pca.channels[self.pwm_channel].duty_cycle = 0
   ```

3. **Test Motor Control:**
   ```bash
   cd /home/severin/dev/r2d2/tests/pan_motor
   sudo python3 test_pca9685_motor.py  # New test script
   ```

4. **Expected Results:**
   - Motor spins in both directions
   - Variable speed control works
   - Direction changes cleanly
   - No GPIO pinmux issues!

### Phase 3: Tilt Servo via PCA9685

**Duration:** ~30 minutes  
**Prerequisites:** Phase 1 complete, servo wired

**Steps:**

1. **Wire Tilt Servo:**
   - Servo Signal (Orange) → PCA9685 CH2
   - Servo V+ (Red) → PCA9685 V+ rail (5V from BEC)
   - Servo GND (Brown) → Common GND

2. **Test Servo Range:**
   ```python
   from adafruit_servokit import ServoKit
   
   kit = ServoKit(channels=16, address=0x40)
   
   # Test servo movement
   kit.servo[2].angle = 90   # Center
   kit.servo[2].angle = 60   # Tilt up
   kit.servo[2].angle = 120  # Tilt down
   ```

3. **Find Safe Mechanical Limits:**
   - Move servo through full range (0-180°)
   - Note where servo starts to strain
   - Update configuration with safe min/max angles

### Phase 4: Encoder & Home Sensor Testing

**Duration:** ~30 minutes  
**Prerequisites:** Encoder and home sensor wired

**Steps:**

1. **Test Encoder Reading:**
   ```python
   import Jetson.GPIO as GPIO
   
   GPIO.setmode(GPIO.BOARD)
   GPIO.setup(16, GPIO.IN)  # Encoder Ch A
   GPIO.setup(18, GPIO.IN)  # Encoder Ch B
   
   # Manually rotate motor shaft, watch for state changes
   while True:
       a = GPIO.input(16)
       b = GPIO.input(18)
       print(f"A={a}, B={b}")
       time.sleep(0.1)
   ```

2. **Test Home Sensor:**
   ```python
   import Jetson.GPIO as GPIO
   
   GPIO.setmode(GPIO.BOARD)
   GPIO.setup(23, GPIO.IN)  # Home sensor
   
   # Move dome to home position, watch for trigger
   while True:
       home = GPIO.input(23)
       print(f"Home: {home}")
       time.sleep(0.1)
   ```

### Phase 5: ROS 2 Integration

**Duration:** ~2-3 hours  
**Prerequisites:** All hardware tests passing

**Steps:**

1. **Rewrite `servo_driver.py`** (for tilt servo via PCA9685)
2. **Create `pan_motor_driver.py`** (for dome motor via PCA9685)
3. **Create `home_sensor.py`** (IR sensor interface)
4. **Update `tilt_tracking_node.py`** (use new servo driver)
5. **Create `pan_tracking_node.py`** (dome rotation tracking)
6. **Test face tracking end-to-end**

See Section 9 for detailed ROS 2 integration plan.

---

## 7. Working Code to Preserve

### 7.1 Direct PWM Test (Pin 13 - For Reference)

This code worked during testing (January 6, 2026) and proves that Pin 13 can control motor speed:

```python
#!/usr/bin/env python3
"""
Working PWM Motor Speed Test
Pin 13 (GPIO32/PWM01) successfully controls motor speed
Date: January 6, 2026
"""

import Jetson.GPIO as GPIO
import time

# This pin WORKS for PWM output
PIN_PWM = 13  # GPIO32/SPI2_SCK

def test_motor_speed():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO.setup(PIN_PWM, GPIO.OUT, initial=GPIO.LOW)
    
    # Initialize PWM at 1kHz
    pwm = GPIO.PWM(PIN_PWM, 1000)
    pwm.start(0)
    
    try:
        print("Testing motor speeds...")
        
        # Test different speeds
        for speed in [20, 40, 60, 80]:
            print(f"Speed: {speed}%")
            pwm.ChangeDutyCycle(speed)
            time.sleep(2)
        
        # Stop motor
        pwm.ChangeDutyCycle(0)
        print("Motor stopped")
        
    finally:
        pwm.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    test_motor_speed()
```

**Result:** ✅ Motor spins at variable speeds (20%, 40%, 60%, 80%)

### 7.2 PCA9685 Motor Control (Future Implementation)

```python
#!/usr/bin/env python3
"""
PCA9685-based Motor Control
To be used when PCA9685 hardware arrives
"""

from adafruit_pca9685 import PCA9685
import board
import busio
import time

class PCA9685MotorDriver:
    """Motor driver using PCA9685 for PWM and DIR signals"""
    
    def __init__(self, pwm_channel=0, dir_channel=1, i2c_address=0x40):
        # Initialize I2C
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c, address=i2c_address)
        self.pca.frequency = 1000  # 1kHz for DC motor PWM
        
        self.pwm_channel = pwm_channel
        self.dir_channel = dir_channel
        
        self.stop()  # Start with motor stopped
    
    def set_speed(self, speed):
        """
        Set motor speed and direction.
        
        Args:
            speed: -1.0 to 1.0
                   Positive = forward (DIR HIGH)
                   Negative = reverse (DIR LOW)
                   0 = stopped
        """
        # Determine direction and absolute speed
        if speed >= 0:
            direction = 1  # Forward
            speed_abs = speed
        else:
            direction = 0  # Reverse
            speed_abs = abs(speed)
        
        # Clamp speed to valid range
        speed_abs = max(0.0, min(1.0, speed_abs))
        
        # Set direction (digital output: 0% = LOW, 100% = HIGH)
        dir_duty = 0xFFFF if direction else 0x0000
        self.pca.channels[self.dir_channel].duty_cycle = dir_duty
        
        # Set PWM speed (0-100% duty cycle)
        pwm_duty = int(speed_abs * 0xFFFF)
        self.pca.channels[self.pwm_channel].duty_cycle = pwm_duty
    
    def stop(self):
        """Stop motor immediately"""
        self.pca.channels[self.pwm_channel].duty_cycle = 0

# Example usage
if __name__ == "__main__":
    motor = PCA9685MotorDriver(pwm_channel=0, dir_channel=1)
    
    try:
        print("Testing motor via PCA9685...")
        
        # Forward at 30%
        print("Forward 30%")
        motor.set_speed(0.3)
        time.sleep(2)
        
        # Stop
        print("Stop")
        motor.stop()
        time.sleep(1)
        
        # Reverse at 30%
        print("Reverse 30%")
        motor.set_speed(-0.3)
        time.sleep(2)
        
        # Stop
        print("Stop")
        motor.stop()
        
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        motor.stop()
```

### 7.3 PCA9685 Servo Control (Tilt Servo)

```python
#!/usr/bin/env python3
"""
PCA9685-based Servo Control
For tilt servo with 5V logic
"""

from adafruit_servokit import ServoKit
import time

class TiltServoDriver:
    """Tilt servo driver using PCA9685"""
    
    def __init__(self, channel=2, i2c_address=0x40, min_angle=60, max_angle=120):
        self.kit = ServoKit(channels=16, address=i2c_address)
        self.channel = channel
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.current_angle = 90  # Start at center
        
        # Set initial position
        self.set_angle(90)
    
    def set_angle(self, angle):
        """Set servo angle (degrees)"""
        # Clamp to safe limits
        angle = max(self.min_angle, min(self.max_angle, angle))
        
        # Set servo position
        self.kit.servo[self.channel].angle = angle
        self.current_angle = angle
    
    def get_angle(self):
        """Get current servo angle"""
        return self.current_angle

# Example usage
if __name__ == "__main__":
    servo = TiltServoDriver(channel=2, min_angle=60, max_angle=120)
    
    try:
        print("Testing tilt servo via PCA9685...")
        
        # Center
        print("Center (90°)")
        servo.set_angle(90)
        time.sleep(1)
        
        # Tilt up
        print("Tilt up (60°)")
        servo.set_angle(60)
        time.sleep(1)
        
        # Tilt down
        print("Tilt down (120°)")
        servo.set_angle(120)
        time.sleep(1)
        
        # Back to center
        print("Center (90°)")
        servo.set_angle(90)
        
    except KeyboardInterrupt:
        print("\nStopped by user")
```

---

## 8. Lessons Learned

### 8.1 Jetson GPIO is NOT like Raspberry Pi

**Key Differences:**
1. **Pin numbering:** Jetson uses different GPIO numbers than Raspberry Pi BCM mode
2. **Pinmux complexity:** Many pins have multiple functions (GPIO, I2C, UART, PWM, CAN, SPI)
3. **Default configuration:** Pins are not GPIO-output-enabled by default
4. **Library compatibility:** `Jetson.GPIO` library exists, but hardware configuration matters more than software
5. **Character device vs sysfs:** Modern JetPack uses `/dev/gpiochip*` instead of `/sys/class/gpio/`

**Lesson:** Never assume GPIO pins work without testing with a multimeter!

### 8.2 Always Verify with Hardware

**What We Learned:**
- ❌ **Software reporting success doesn't mean hardware works**
  - `GPIO.output(pin, HIGH)` returned success
  - `GPIO.input(pin)` read back as HIGH
  - But multimeter showed **0V on the pin**!
- ✅ **Always test with multimeter before assuming pin works**
- ✅ **Test each pin individually before building complex circuits**

### 8.3 I2C Expanders are the Right Approach for Jetson

**Why I2C is Better Than Direct GPIO:**
1. **Pinmux safe:** I2C pins (3, 5) are dedicated I2C, no pinmux conflicts
2. **Scalable:** One I2C bus can support many devices (up to 127 devices)
3. **Fewer wires:** 2 wires (SDA, SCL) instead of many GPIO wires
4. **Industry standard:** Well-supported libraries and proven reliability
5. **Consistent with LED plan:** We're already using MCP23017 for LEDs

**Lesson:** On Jetson, use I2C expanders (PCA9685, MCP23017) instead of direct GPIO whenever possible.

### 8.4 Documentation is Critical

**What Helped:**
- ✅ Detailed pin reference diagrams (from NVIDIA and user's custom diagram)
- ✅ Systematic testing approach (one pin at a time)
- ✅ Keeping notes during debugging (this document!)

**What Didn't Help:**
- ❌ Raspberry Pi GPIO tutorials (incompatible pin numbering)
- ❌ Assuming software success means hardware works
- ❌ Not testing with multimeter first

### 8.5 Hardware Limitations Can Be Opportunities

**Original Plan:** Direct GPIO control for 3 motors (6 pins)  
**Discovered Limitation:** Only 1 working GPIO output pin  
**Better Solution:** PCA9685 I2C PWM controller (2 pins, 16 channels)

**Lesson:** The hardware limitation forced us to find a better, more scalable solution. PCA9685 is cleaner, more reliable, and easier to expand than direct GPIO would have been.

---

## 9. ROS 2 Integration

### 9.1 ROS 2 Topics (Final Design)

**Input Topics (Consumed by Head Control):**

| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| `/r2d2/perception/face_bbox` | geometry_msgs/Point | image_listener | Face position (x, y normalized, z = size) |
| `/r2d2/audio/person_status` | std_msgs/String | speech_node | Person status for tracking gating |

**Output Topics (Published by Head Control):**

| Topic | Type | Description |
|-------|------|-------------|
| `/r2d2/head_control/tilt_angle` | std_msgs/Float32 | Current tilt servo angle (degrees) |
| `/r2d2/head_control/pan_angle` | std_msgs/Float32 | Current pan motor angle (degrees from home) |
| `/r2d2/head_control/tracking_status` | std_msgs/String | Tracking state (tracking/holding/neutral/homing) |

### 9.2 ROS 2 Package Structure

```
ros2_ws/src/r2d2_head_control/
├── config/
│   ├── pan_tilt_params.yaml         # Combined configuration
│   └── r2d2-head-tracking.service   # Systemd service
├── launch/
│   └── head_tracking.launch.py      # Combined pan + tilt launch
├── r2d2_head_control/
│   ├── __init__.py
│   ├── pid_controller.py            # PID (already exists) ✅
│   ├── pca9685_servo_driver.py      # Servo via PCA9685 (new)
│   ├── pca9685_motor_driver.py      # Motor via PCA9685 (new)
│   ├── encoder_reader.py            # Quadrature encoder (new)
│   ├── home_sensor.py               # IR home sensor (new)
│   ├── tilt_tracking_node.py        # Tilt tracking (update for PCA9685)
│   └── pan_tracking_node.py         # Pan tracking (new)
├── package.xml
├── setup.py
└── setup.cfg
```

### 9.3 Configuration Parameters

**File:** `config/pan_tilt_params.yaml`

```yaml
pan_tilt_controller:
  ros__parameters:
    # PCA9685 Configuration
    pca9685_i2c_address: 0x40
    pca9685_frequency: 1000  # 1kHz for DC motors, will auto-adjust for servo
    
    # Channel Allocation
    pan_motor_pwm_channel: 0   # PCA9685 CH0
    pan_motor_dir_channel: 1   # PCA9685 CH1
    tilt_servo_channel: 2      # PCA9685 CH2
    
    # Pan Motor Settings
    pan_enabled: true
    pan_tracking_rate_hz: 20.0
    pan_max_speed: 0.5         # 0-1 (50% max)
    pan_deadband: 0.10         # 10% of frame
    pan_smoothing_alpha: 0.2   # Heavy smoothing (larger mass)
    
    # Pan Motor Encoder
    pan_encoder_pin_a: 16      # GPIO23
    pan_encoder_pin_b: 18      # GPIO24
    pan_encoder_ppr: 12        # Pulses per revolution (to be measured)
    
    # Pan Motor Home Sensor
    pan_home_sensor_pin: 23    # GPIO11
    pan_home_on_startup: true
    pan_home_timeout_sec: 30.0
    
    # Pan Motor Limits
    pan_min_angle: 0.0         # Home position
    pan_max_angle: 350.0       # Safe limit (avoid cable wrap)
    pan_neutral_angle: 175.0   # Center
    
    # Pan PID Gains (need tuning)
    pan_kp: 1.0
    pan_ki: 0.1
    pan_kd: 0.5
    
    # Tilt Servo Settings
    tilt_enabled: true
    tilt_tracking_rate_hz: 30.0
    tilt_deadband: 0.08        # 8% of frame
    tilt_smoothing_alpha: 0.3  # More responsive
    
    # Tilt Servo Limits (to be determined via testing)
    tilt_min_angle: 60.0       # Tilt up (max)
    tilt_max_angle: 120.0      # Tilt down (max)
    tilt_neutral_angle: 90.0   # Center
    
    # Tilt PID Gains
    tilt_kp: 30.0
    tilt_ki: 0.5
    tilt_kd: 5.0
    
    # Loss Handling (both axes)
    hold_timeout_sec: 0.5      # Hold position briefly
    neutral_timeout_sec: 3.0   # Return to neutral after this
    neutral_return_speed: 5.0  # Degrees per second
    
    # Hardware Simulation (for testing without hardware)
    simulate_hardware: false
```

---

## 10. Rollback Strategy

**Baseline:** `golden-2026-01-05` (system fully operational before pan/tilt work)

### 10.1 If PCA9685 Approach Fails

**Unlikely scenario:** PCA9685 hardware arrives but doesn't work

**Rollback steps:**
```bash
# Quick rollback to last golden checkpoint
git reset --hard golden-2026-01-05

# If you've pushed to GitHub and want to revert there too:
git push --force origin main
```

### 10.2 After Successful Implementation

**Create new golden tag when everything is tested and working:**

```bash
git tag -a golden-pan-tilt-complete -m "Pan/tilt tracking: PCA9685 + motor + encoder + servo + homing"
git push origin golden-pan-tilt-complete
```

---

## Appendix A: Diagnostic Commands Used

### GPIO Diagnostics

```bash
# List GPIO chips
ls /dev/gpiochip*

# Check GPIO pin info (requires gpiod tools)
sudo apt install gpiod
gpioinfo gpiochip0
gpioinfo gpiochip1

# Monitor GPIO state
gpioget gpiochip0 32  # Pin 13 (working)
gpioget gpiochip1 8   # Pin 16 (not working)

# Test GPIO output
gpioset gpiochip0 32=1  # Set high
gpioset gpiochip0 32=0  # Set low
```

### I2C Diagnostics

```bash
# List I2C buses
ls /dev/i2c-*

# Scan I2C bus for devices
i2cdetect -y 1  # or -y 8

# Read I2C device register
i2cget -y 1 0x40 0x00
```

### Pinmux Diagnostics

```bash
# Check pinmux configuration (requires root and jetson-io)
sudo /opt/nvidia/jetson-io/jetson-io.py

# View current pinmux settings
sudo cat /sys/kernel/debug/tegra_pinctrl_reg

# Check if pin is exported
ls /sys/class/gpio/  # (doesn't exist on JetPack R36.4+)
```

---

## Appendix B: Hardware References

### Pololu G2 24v21 Resources

- **Product Page:** https://www.pololu.com/product/2995
- **Datasheet:** https://www.pololu.com/file/0J1487/g2-high-power-motor-driver-24v21-datasheet.pdf
- **User Guide:** https://www.pololu.com/docs/0J77

### PCA9685 Resources

- **BerryBase Product:** https://www.berrybase.ch/berrybase-16-kanal-pwm-servo-treiber-board-pca9685-i2c-12bit-1-6khz-3-3-5v
- **NXP PCA9685 Datasheet:** https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf
- **Adafruit Library:** https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
- **Adafruit ServoKit:** https://github.com/adafruit/Adafruit_CircuitPython_ServoKit

### Jetson GPIO Resources

- **NVIDIA Jetson GPIO Library:** https://github.com/NVIDIA/jetson-gpio
- **Jetson AGX Orin Pinout:** https://jetsonhacks.com/nvidia-jetson-agx-orin-gpio-header-pinout/
- **Jetson.GPIO Python Documentation:** https://github.com/NVIDIA/jetson-gpio/blob/master/README.md

---

**Document Status:** ✅ Complete - Ready for PCA9685 hardware arrival  
**Last Updated:** January 6, 2026  
**Next Review:** When PCA9685 boards arrive (expected ~5-7 days)

---

**End of Document**

