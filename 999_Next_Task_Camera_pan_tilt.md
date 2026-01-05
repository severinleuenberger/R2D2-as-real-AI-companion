# R2D2 Camera Pan/Tilt Tracking System - Implementation Plan

**Date:** January 3, 2026  
**Last Updated:** January 5, 2026  
**Status:** BLOCKED - Waiting for hardware components  
**Branch:** Working on `main` (golden tag: golden-2026-01-05)  
**Author:** AI Agent (Claude)

---

## Executive Summary

This document captures the complete plan for implementing face-centered camera tracking for R2D2. The system will adjust camera orientation (tilt via servo, pan via dome motor) to keep detected faces centered in the camera frame.

**Goal:** When a human face is detected, R2D2 actively tracks that face by adjusting head pan and camera tilt to keep the face centered. Movement should feel intentional, calm, responsive, and "alive."

**Current Status:** Phase 1 code complete, hardware blocked pending component delivery.

---

## Table of Contents

1. [Current State (Already Implemented)](#1-current-state-already-implemented)
2. [Hardware Discovery Summary](#2-hardware-discovery-summary)
3. [Components to Order](#3-components-to-order)
4. [Implementation Plan](#4-implementation-plan-after-hardware-arrives)
5. [ROS 2 Topics](#5-ros-2-topics-final-design)
6. [Configuration Parameters](#6-configuration-parameters)
7. [Architecture Diagram](#7-architecture-diagram)
8. [Systemd Service](#8-systemd-service)
9. [Next Steps](#9-next-steps-immediate-actions)
10. [Rollback Strategy](#10-rollback-strategy)

---

## 1. Current State (Already Implemented)

### 1.1 Git Workflow

- **Branch:** Working on `main` branch (simplified workflow)
- **Baseline Checkpoint:** `golden-2026-01-05` (system fully operational)
- **On Completion:** Create `golden-pan-tilt-complete` tag
- **Rollback:** Reset to `golden-2026-01-05` if issues arise

### 1.2 ROS 2 Package Created

**Location:** `ros2_ws/src/r2d2_head_control/`

**Package Structure:**
```
r2d2_head_control/
├── config/
│   ├── tilt_tracking_params.yaml      # Configuration parameters
│   └── r2d2-tilt-tracking.service     # Systemd service file
├── launch/
│   └── tilt_tracking.launch.py        # ROS 2 launch file
├── r2d2_head_control/
│   ├── __init__.py
│   ├── pid_controller.py              # PID with anti-windup
│   ├── servo_driver.py                # Servo control (needs rewrite)
│   └── tilt_tracking_node.py          # Main tracking node
├── package.xml
├── setup.py
└── setup.cfg
```

### 1.3 Implemented Components

#### PID Controller (`pid_controller.py`)

Full PID implementation with:
- Anti-windup (integral clamping)
- Output limiting
- Derivative filtering (low-pass filter for noise reduction)
- Runtime gain adjustment

**Key Parameters:**
- `kp`, `ki`, `kd`: PID gains
- `output_min`, `output_max`: Output clamping
- `integral_limit`: Anti-windup limit
- `derivative_filter_alpha`: Derivative smoothing

#### Servo Driver (`servo_driver.py`)

**IMPORTANT:** Current implementation uses direct GPIO PWM.

**Problem Discovered:** Jetson AGX Orin GPIO outputs 3.3V logic, but servo requires 5V logic signal.

**Solution Required:** Replace with PCA9685 I2C PWM driver (see Section 4).

Current features (to be preserved in rewrite):
- Angle-based control (set_angle, get_angle)
- Configurable min/max angle limits
- Simulation mode for testing without hardware
- Sweep test function

#### Tilt Tracking Node (`tilt_tracking_node.py`)

Main ROS 2 node for face tracking with:
- Subscribes to `/r2d2/perception/face_bbox` (face position)
- Subscribes to `/r2d2/audio/person_status` (tracking gating)
- Publishes `/r2d2/head_control/tilt_angle` (current angle)
- Smooth tracking with PID control
- Deadband to prevent jitter
- Exponential smoothing on input
- Loss handling (hold position, then return to neutral)

### 1.4 Perception Integration (Already Complete)

Face bounding box publisher added to `image_listener.py`:

**Topic:** `/r2d2/perception/face_bbox` (geometry_msgs/Point)

**Fields:**
- `x`: Normalized X center of face (0.0 = left, 1.0 = right)
- `y`: Normalized Y center of face (0.0 = top, 1.0 = bottom)
- `z`: Normalized face size (area relative to frame)

### 1.5 Test Script Created

**Location:** `scripts/test/test_servo_range.py`

Purpose: Test servo full range in 5% steps to determine safe mechanical limits.

**Note:** Does not work with current wiring due to 3.3V signal issue.

### 1.6 Code Safety Analysis (Verified)

The head tracking code has been analyzed and **confirmed safe** - it does not conflict with any existing R2D2 systems.

#### GPIO Pin Analysis (No Overlap)

| Service | GPIO (BCM) | Physical Pin | Purpose |
|---------|------------|--------------|---------|
| **status_led_node** | GPIO17 | Pin 22 | White LED |
| **status_led_node** | GPIO27 | Pin 15 | Green LED (RGB mode) |
| **status_led_node** | GPIO22 | Pin 15 | Blue LED (RGB mode) |
| **Power button** | GPIO32 | Pin 32 | Shutdown |
| **servo_driver (head control)** | GPIO13 | Pin 33 | Tilt servo |

**Result:** No GPIO pin conflicts - servo uses GPIO13 which is not used by any other service.

#### ROS 2 Topic Analysis (No Conflicts)

| Topic | Action | Conflict? |
|-------|--------|-----------|
| `/r2d2/perception/face_bbox` | Subscribes | New topic, created for this feature |
| `/r2d2/audio/person_status` | Subscribes | Read-only subscription |
| `/r2d2/head_control/tilt_angle` | Publishes | New topic, no overlap |

**Result:** Head control only reads from existing topics (doesn't modify them) and publishes to new topics.

#### Code Isolation

- ✅ Separate ROS 2 package (`r2d2_head_control`)
- ✅ No shared state with other nodes
- ✅ Same GPIO mode as existing code (both use BCM)
- ✅ Independent systemd service (when installed)
- ✅ Currently non-functional due to 3.3V issue (no impact on running system)

**Conclusion:** Safe to proceed - the head tracking code is well-isolated and will not interfere with LED, power button, audio, perception, or any other R2D2 functionality.

---

## 2. Hardware Discovery Summary

### 2.1 Tilt Servo Problem Identified

**Root Cause:** Jetson AGX Orin GPIO outputs 3.3V logic, but RC servos require 5V logic signal to reliably detect HIGH.

**Testing Performed:**
1. Servo confirmed working with external servo tester (outputs 5V signal)
2. External 5V BEC power + shared ground on Jetson tested
3. Software PWM on Pin 33 (BOARD mode) implemented
4. No servo movement observed despite clean PWM waveform

**Conclusion:** The 3.3V GPIO signal is insufficient for the servo.

**Solution:** Use **PCA9685 I2C PWM driver** which outputs 5V logic signals.

### 2.2 Pan Motor System Analysis

**Motor:** DeAgostini DC motor with built-in Hall encoder (Issue 96)

**Encoder Details:**
- Green PCB with 8-spoke magnetic encoder wheel
- Hall effect sensors detect rotation
- 6-pin connector for encoder signals (VCC, GND, Ch A, Ch B, possibly Index)
- Provides quadrature output for position tracking

**Motor Control:** Requires Pololu motor driver for PWM speed control (not just on/off).

**Home Position Sensor:** R2D2 Infrared-L-V1.1 (already installed in R2D2)
- PCB marked "R2D2 Infrared-L-V1.1" dated 2018/03/02
- 3-pin connector:
  - L (Blue wire): Signal output
  - + (Red wire): VCC (5V)
  - - (Black wire): GND
- Detects dome center/home position via IR reflective sensing

### 2.3 Pin Assignments

#### Currently In Use (DO NOT CHANGE)

| Pin | GPIO | Function | Notes |
|-----|------|----------|-------|
| 32 | GPIO32 | Shutdown Button | **CRITICAL - DO NOT REASSIGN** |
| 22 | GPIO17 | White LED Control | Visual status indicator |
| 1/17 | 3.3V | LED Power | Power for LED panel |
| 6 | GND | Common Ground | Shared ground |

#### Proposed for Pan/Tilt System

| Pin | GPIO | Function | Device |
|-----|------|----------|--------|
| 3 | I2C SDA | I2C Data | PCA9685 |
| 5 | I2C SCL | I2C Clock | PCA9685 |
| 15 | GPIO22 | PWM Output | Pan Motor Speed |
| 29 | GPIO5 | Digital Out | Pan Motor Direction 1 |
| 31 | GPIO6 | Digital Out | Pan Motor Direction 2 |
| TBD | - | Digital In | Encoder Channel A |
| TBD | - | Digital In | Encoder Channel B |
| TBD | - | Digital In | Home Sensor Signal |

---

## 3. Components to Order

### 3.1 Required Components

| Component | Purpose | Interface | Approx. Price |
|-----------|---------|-----------|---------------|
| **BerryBase PCA9685 16-Channel PWM Driver** | Tilt servo control with 5V logic | I2C | CHF 5-10 |
| **Pololu TB6612FNG or DRV8833** | Pan motor PWM speed control | GPIO | CHF 5-10 |

**Selected Board:** [BerryBase 16-Kanal PWM Servo Treiber Board PCA9685](https://www.berrybase.ch/berrybase-16-kanal-pwm-servo-treiber-board-pca9685-i2c-12bit-1-6khz-3-3-5v)

This is a generic PCA9685 board - functionally identical to the Adafruit version but more affordable. Uses the same PCA9685 chip, same I2C protocol, same Python library (`adafruit-circuitpython-pca9685`).

**Why PCA9685:**
- Outputs 5V logic signals (servo requirement)
- Hardware PWM (very precise timing, no jitter)
- 16 channels (expandable for future servos)
- I2C interface (only 2 pins needed)
- Industry standard for servo control

**Why Pololu Driver for Pan:**
- DC motors need high-frequency PWM (10-20 kHz)
- Provides H-bridge for bidirectional control
- Current limiting for motor protection
- Compatible with Jetson 3.3V GPIO

### 3.2 Already Available

| Component | Status | Location |
|-----------|--------|----------|
| Tilt servo (RC servo) | Confirmed working with servo tester | In R2D2 |
| Pan motor with encoder | Part of DeAgostini kit | In R2D2 dome |
| Home position IR sensor | Installed | In R2D2 body |
| 5V BEC | Available | Power system |

---

## 4. Implementation Plan (After Hardware Arrives)

### Phase 1: Tilt Servo via PCA9685

#### 4.1 Hardware Setup

**PCA9685 Wiring:**

```
PCA9685 Board          Jetson AGX Orin 40-Pin Header
═══════════════        ═══════════════════════════════
VCC  ─────────────────► Pin 1 or 17 (3.3V)
GND  ─────────────────► Pin 6 (GND)
SDA  ─────────────────► Pin 3 (I2C SDA)
SCL  ─────────────────► Pin 5 (I2C SCL)
V+   ─────────────────► External 5V BEC (servo power)

PCA9685 CH0            Tilt Servo
═══════════════        ═══════════════
Signal ───────────────► Orange wire
V+     ───────────────► Red wire (5V)
GND    ───────────────► Brown/Black wire
```

**Important:** V+ on PCA9685 is the servo power rail, separate from logic power.

#### 4.2 Software Changes Required

**File: `servo_driver.py` - Complete Rewrite**

Replace GPIO PWM with PCA9685 I2C control:

```python
# New dependencies
from adafruit_servokit import ServoKit

class ServoDriver:
    def __init__(self, channel=0, i2c_address=0x40, ...):
        self.kit = ServoKit(channels=16, address=i2c_address)
        self.channel = channel
        # ... rest of initialization
    
    def set_angle(self, angle):
        angle = max(self.min_angle, min(self.max_angle, angle))
        self.kit.servo[self.channel].angle = angle
        self.current_angle = angle
```

**New Dependencies to Install:**
```bash
pip3 install adafruit-circuitpython-pca9685 adafruit-circuitpython-servokit
```

**File: `tilt_tracking_params.yaml` - Update Parameters**

Remove GPIO-specific parameters, add PCA9685 parameters:

```yaml
tilt_tracking_node:
  ros__parameters:
    # Remove: servo_gpio_pin
    pca9685_i2c_address: 0x40    # Default I2C address
    servo_channel: 0             # PCA9685 channel (0-15)
    # ... rest unchanged
```

#### 4.3 Testing Procedure

1. **Install Python libraries:**
   ```bash
   pip3 install adafruit-circuitpython-pca9685 adafruit-circuitpython-servokit
   ```

2. **Verify I2C communication:**
   ```bash
   # Find Jetson I2C bus (usually bus 1 or 8)
   ls /dev/i2c-*
   
   # Scan for PCA9685 (should show 0x40)
   i2cdetect -y 1   # or i2cdetect -y 8
   ```

3. **Test servo with PCA9685:**
   ```python
   from adafruit_servokit import ServoKit
   kit = ServoKit(channels=16)
   kit.servo[0].angle = 90   # Center
   kit.servo[0].angle = 60   # Up
   kit.servo[0].angle = 120  # Down
   ```

4. **Run range test to determine safe limits:**
   - Move servo through full range
   - Note mechanical limits (where servo strains)
   - Update `min_angle` and `max_angle` in config

5. **Test face tracking end-to-end:**
   ```bash
   cd ~/dev/r2d2/ros2_ws
   source install/setup.bash
   ros2 launch r2d2_head_control tilt_tracking.launch.py
   ```

### Phase 2: Pan Motor Control

#### 4.4 Hardware Setup

**Pololu Driver Wiring (TB6612FNG example):**

```
Pololu TB6612FNG       Jetson AGX Orin 40-Pin Header
════════════════       ═══════════════════════════════
VM   ─────────────────► External 5V-14.8V (motor power)
GND  ─────────────────► Common ground (Pin 6 + motor power GND)
VCC  ─────────────────► Pin 1 or 17 (3.3V logic power)
STBY ─────────────────► Pin 1 or 17 (3.3V - always enabled)
PWMA ─────────────────► Pin 15 (GPIO22) - PWM speed
AIN1 ─────────────────► Pin 29 (GPIO5) - Direction 1
AIN2 ─────────────────► Pin 31 (GPIO6) - Direction 2
AO1  ─────────────────► Motor terminal +
AO2  ─────────────────► Motor terminal -
```

**Motor Encoder Wiring:**

```
Encoder (6-pin)        Jetson AGX Orin
═══════════════        ═══════════════
VCC (Red)    ─────────► Pin 2 or 4 (5V)
GND (Black)  ─────────► Pin 6 (GND)
Ch A (Yellow) ────────► GPIO TBD (interrupt capable)
Ch B (Green)  ────────► GPIO TBD (interrupt capable)
```

**Home Sensor Wiring:**

```
IR Sensor (3-pin)      Jetson AGX Orin
═════════════════      ═══════════════
L (Blue)     ─────────► GPIO TBD (digital input)
+ (Red)      ─────────► Pin 2 or 4 (5V)
- (Black)    ─────────► Pin 6 (GND)
```

#### 4.5 New Software Components

**File: `pan_motor_driver.py` (New)**

```python
class PanMotorDriver:
    """DC motor driver with encoder feedback for dome pan control."""
    
    def __init__(self, pwm_pin, dir1_pin, dir2_pin, 
                 encoder_a_pin, encoder_b_pin):
        # GPIO setup for motor control
        # Encoder interrupt setup
        # Position tracking
    
    def set_speed(self, speed):
        """Set motor speed (-1.0 to 1.0, negative = reverse)"""
    
    def get_position(self):
        """Get current position in degrees (from encoder)"""
    
    def home(self):
        """Execute homing routine using home sensor"""
```

**File: `home_sensor.py` (New)**

```python
class HomeSensor:
    """IR home position sensor interface."""
    
    def __init__(self, gpio_pin):
        # GPIO setup with interrupt
        # Debouncing logic
    
    def is_home(self):
        """Return True if at home position"""
    
    def wait_for_home(self, timeout=30.0):
        """Block until home position detected"""
```

**File: `pan_tracking_node.py` (New)**

```python
class PanTrackingNode(Node):
    """ROS 2 node for horizontal face tracking via dome motor."""
    
    # Subscribes to /r2d2/perception/face_bbox
    # PID control for pan position
    # Homing routine on startup
    # Publishes /r2d2/head_control/pan_angle
    # Soft limits at 0° and 350°
```

#### 4.6 Homing Sequence

On node startup:
1. Log "Starting homing sequence..."
2. Rotate dome slowly (e.g., 10% speed) in one direction
3. Monitor home sensor for trigger
4. When home sensor triggers:
   - Stop motor immediately
   - Set position counter to 0° (center reference)
   - Log "Homing complete - position set to 0°"
5. Enable tracking mode
6. During operation: Track position via encoder pulse counting

**Position Calculation:**
```
degrees = (encoder_pulses / pulses_per_revolution) × 360°
```

---

## 5. ROS 2 Topics (Final Design)

### Input Topics (Consumed by Head Control)

| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| `/r2d2/perception/face_bbox` | geometry_msgs/Point | image_listener | Face position (x, y normalized, z = size) |
| `/r2d2/audio/person_status` | std_msgs/String | speech_node | Person status for tracking gating |

### Output Topics (Published by Head Control)

| Topic | Type | Description |
|-------|------|-------------|
| `/r2d2/head_control/tilt_angle` | std_msgs/Float32 | Current tilt servo angle (degrees) |
| `/r2d2/head_control/pan_angle` | std_msgs/Float32 | Current pan motor angle (degrees) |
| `/r2d2/head_control/tracking_status` | std_msgs/String | Tracking state (tracking/holding/neutral/homing) |

---

## 6. Configuration Parameters

### Tilt Tracking (`tilt_tracking_params.yaml`)

```yaml
tilt_tracking_node:
  ros__parameters:
    # Enable/disable
    enabled: true
    tracking_rate_hz: 30.0
    
    # PCA9685 Configuration (NEW)
    pca9685_i2c_address: 0x40   # Default I2C address
    servo_channel: 0            # PCA9685 channel (0-15)
    
    # Input processing
    deadband: 0.08              # 8% of frame - ignore small movements
    smoothing_alpha: 0.3        # Exponential filter (0=smooth, 1=responsive)
    
    # PID gains
    kp: 30.0                    # Proportional (degrees per unit error)
    ki: 0.5                     # Integral (low to avoid oscillation)
    kd: 5.0                     # Derivative (damping)
    
    # Servo limits (TO BE DETERMINED VIA TESTING)
    min_angle: 60.0             # Maximum tilt UP
    max_angle: 120.0            # Maximum tilt DOWN
    neutral_angle: 90.0         # Rest position
    
    # Loss handling
    hold_timeout_sec: 0.5       # Hold position briefly when face lost
    neutral_timeout_sec: 3.0    # Return to neutral after this time
    neutral_return_speed: 5.0   # Degrees per second
    
    # Hardware simulation (for testing)
    simulate_hardware: false
```

### Pan Tracking (`pan_tracking_params.yaml` - To Be Created)

```yaml
pan_tracking_node:
  ros__parameters:
    # Enable/disable
    enabled: true
    tracking_rate_hz: 20.0      # Slower than tilt (motor inertia)
    
    # Motor GPIO pins
    motor_pwm_pin: 22           # GPIO22 = Pin 15
    motor_dir1_pin: 5           # GPIO5 = Pin 29
    motor_dir2_pin: 6           # GPIO6 = Pin 31
    
    # Encoder pins
    encoder_a_pin: TBD          # To be determined
    encoder_b_pin: TBD          # To be determined
    encoder_ppr: TBD            # Pulses per revolution
    
    # Home sensor
    home_sensor_pin: TBD        # To be determined
    
    # Input processing
    deadband: 0.10              # 10% of frame (larger for pan)
    smoothing_alpha: 0.2        # More smoothing (larger mass)
    
    # PID gains (need tuning for motor dynamics)
    kp: 1.0
    ki: 0.1
    kd: 0.5
    
    # Position limits
    min_angle: 0.0              # Home position
    max_angle: 350.0            # Mechanical limit (not full 360°)
    neutral_angle: 175.0        # Center
    
    # Speed limits
    max_speed: 0.5              # Maximum motor speed (0-1)
    homing_speed: 0.1           # Slow speed for homing
    
    # Homing
    home_on_startup: true
    home_timeout_sec: 30.0
```

---

## 7. Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                    R2D2 HEAD TRACKING SYSTEM                        │
└─────────────────────────────────────────────────────────────────────┘

                        ┌─────────────────┐
                        │   OAK-D Lite    │
                        │     Camera      │
                        └────────┬────────┘
                                 │ RGB Frames
                                 ▼
                        ┌─────────────────┐
                        │ image_listener  │
                        │ (face detection)│
                        └────────┬────────┘
                                 │ /r2d2/perception/face_bbox
                                 │ (Point: x, y, z)
                    ┌────────────┴────────────┐
                    │                         │
                    ▼                         ▼
        ┌───────────────────┐     ┌───────────────────┐
        │ tilt_tracking_node│     │ pan_tracking_node │
        │   (vertical)      │     │   (horizontal)    │
        └─────────┬─────────┘     └─────────┬─────────┘
                  │                         │
                  │ PID                     │ PID
                  ▼                         ▼
        ┌───────────────────┐     ┌───────────────────┐
        │    PCA9685        │     │  Pololu Driver    │
        │ I2C PWM Driver    │     │   (GPIO PWM)      │
        │ (5V logic out)    │     │                   │
        └─────────┬─────────┘     └─────────┬─────────┘
                  │                         │
                  ▼                         ▼
        ┌───────────────────┐     ┌───────────────────┐
        │   Tilt Servo      │     │    Pan Motor      │
        │  (camera angle)   │     │ (dome rotation)   │
        └───────────────────┘     └────────┬──────────┘
                                           │
                                  ┌────────┴────────┐
                                  │                 │
                                  ▼                 ▼
                        ┌─────────────┐   ┌─────────────┐
                        │   Encoder   │   │ Home Sensor │
                        │ (position)  │   │  (center)   │
                        └──────┬──────┘   └──────┬──────┘
                               │                 │
                               └────────┬────────┘
                                        │ Feedback
                                        ▼
                              ┌───────────────────┐
                              │ pan_tracking_node │
                              │ (position loop)   │
                              └───────────────────┘

I2C Bus (Pins 3, 5):
  └── PCA9685 @ 0x40

GPIO Pins:
  ├── Pin 15 (GPIO22): Pan Motor PWM
  ├── Pin 29 (GPIO5):  Pan Motor DIR1
  ├── Pin 31 (GPIO6):  Pan Motor DIR2
  ├── Pin TBD:         Encoder A
  ├── Pin TBD:         Encoder B
  └── Pin TBD:         Home Sensor

Power:
  ├── External 5V BEC → PCA9685 V+ (servo power)
  ├── External 5V-14.8V → Pololu VM (motor power)
  └── Jetson 3.3V → PCA9685 VCC, Pololu VCC (logic power)
```

---

## 8. Systemd Service

### Current Service File

**Location:** `ros2_ws/src/r2d2_head_control/config/r2d2-tilt-tracking.service`

```ini
[Unit]
Description=R2D2 Head Control Tilt Tracking Service
After=network.target r2d2-camera-perception.service r2d2-audio-notification.service
Wants=r2d2-camera-perception.service r2d2-audio-notification.service

[Service]
Type=simple
User=severin
WorkingDirectory=/home/severin/dev/r2d2/ros2_ws
ExecStart=/bin/bash -c "source install/setup.bash && ros2 launch r2d2_head_control tilt_tracking.launch.py"
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
```

### Installation (After Testing Complete)

```bash
# Copy service file
sudo cp ~/dev/r2d2/ros2_ws/src/r2d2_head_control/config/r2d2-tilt-tracking.service /etc/systemd/system/

# Reload systemd
sudo systemctl daemon-reload

# Enable auto-start
sudo systemctl enable r2d2-tilt-tracking.service

# Start service
sudo systemctl start r2d2-tilt-tracking.service

# Check status
systemctl status r2d2-tilt-tracking.service
```

---

## 9. Next Steps (Immediate Actions)

### Step 0: Verify Baseline Checkpoint

Before starting hardware work, verify current state is safe:

```bash
# Verify we're on the golden checkpoint
git describe --tags --abbrev=0

# Should show: golden-2026-01-05 or newer
# This is your rollback point if anything goes wrong
```

If any issues during development, you can always return to this golden point:
```bash
git reset --hard golden-2026-01-05
```

### Step 1: Order Components

- [x] Select PCA9685 board (BerryBase chosen over Adafruit - same chip, lower cost)
- [ ] Complete order for BerryBase PCA9685 from berrybase.ch
- [ ] Order Pololu TB6612FNG or DRV8833 motor driver

### Step 2: When PCA9685 Arrives

1. Wire PCA9685 to Jetson I2C (Pins 3, 5) and external 5V BEC
2. Install Python libraries:
   ```bash
   pip3 install adafruit-circuitpython-pca9685 adafruit-circuitpython-servokit
   ```
3. Test I2C communication:
   ```bash
   i2cdetect -y 1  # or -y 8 depending on Jetson I2C bus
   ```
4. Run servo range test to find safe mechanical limits
5. Rewrite `servo_driver.py` to use PCA9685
6. Update configuration with discovered angle limits
7. Test tilt tracking end-to-end

### Step 3: When Motor Driver Arrives

1. Wire Pololu driver to Jetson GPIO and motor
2. Connect encoder signals to Jetson GPIO
3. Connect home sensor to Jetson GPIO
4. Implement `pan_motor_driver.py`
5. Implement `home_sensor.py`
6. Implement `pan_tracking_node.py` with homing routine
7. Test pan tracking
8. Test combined pan/tilt tracking

### Step 4: Integration & Deployment

1. Test full system with face detection
2. Tune PID parameters for smooth tracking
3. Install systemd service
4. Verify auto-start after reboot
5. Create golden tag for successful completion

---

## 10. Rollback Strategy

All development follows the simplified main-only workflow with golden tag checkpoints.

**Baseline:** `golden-2026-01-05` (system fully operational before pan/tilt work)

**To rollback if issues arise:**

```bash
# Quick rollback to last golden checkpoint
git reset --hard golden-2026-01-05

# If you've pushed to GitHub and want to revert there too:
git push --force origin main
```

**After Successful Implementation:**

Create a new golden tag when everything is tested and working:

```bash
git tag -a golden-pan-tilt-complete -m "Pan/tilt tracking: servo + motor + encoder + homing"
git push origin golden-pan-tilt-complete
```

**Main branch stays clean** with golden tags marking stable, working states.

---

## Appendix A: Testing Commands

### Check Face BBox Topic
```bash
ros2 topic echo /r2d2/perception/face_bbox
```

### Check Tilt Angle Topic
```bash
ros2 topic echo /r2d2/head_control/tilt_angle
```

### Manual Servo Test (after PCA9685 installed)
```python
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)
kit.servo[0].angle = 90   # Center
kit.servo[0].angle = 60   # Up  
kit.servo[0].angle = 120  # Down
```

### Launch Tilt Tracking
```bash
cd ~/dev/r2d2/ros2_ws
source install/setup.bash
ros2 launch r2d2_head_control tilt_tracking.launch.py
```

### Launch with Simulation Mode (no hardware)
```bash
ros2 launch r2d2_head_control tilt_tracking.launch.py simulate_hardware:=true
```

---

## Appendix B: Troubleshooting

### PCA9685 Not Detected on I2C

**Symptoms:** `i2cdetect` shows no device at 0x40

**Solutions:**
1. Check wiring (VCC, GND, SDA, SCL)
2. Verify I2C bus number: `ls /dev/i2c-*`
3. Check if I2C is enabled in Jetson config
4. Try different I2C bus: `i2cdetect -y 8` instead of `-y 1`

### Servo Jitters or Oscillates

**Symptoms:** Servo moves erratically, buzzes, or overshoots

**Solutions:**
1. Increase deadband (e.g., 0.08 → 0.12)
2. Decrease Kp (proportional gain)
3. Increase Kd (derivative gain for damping)
4. Increase smoothing_alpha (more input filtering)
5. Check for electrical noise on power supply

### Motor Doesn't Respond

**Symptoms:** Pan motor doesn't move when commanded

**Solutions:**
1. Check motor driver wiring
2. Verify STBY pin is HIGH (connected to 3.3V)
3. Check PWM signal with oscilloscope or logic analyzer
4. Verify motor power supply has sufficient current
5. Test motor directly with bench power supply

### Home Sensor Not Triggering

**Symptoms:** Homing routine times out

**Solutions:**
1. Check sensor wiring (VCC, GND, Signal)
2. Verify signal pin with multimeter (should toggle when reflector passes)
3. Clean IR sensor lens
4. Adjust sensor position relative to reflector
5. Add debouncing if getting false triggers

---

**Document Status:** PLANNING COMPLETE - Awaiting Hardware  
**Last Updated:** January 3, 2026  
**Next Review:** When components arrive

---

**End of Document**

