# R2D2 Audio System Reference

> **Last Updated**: January 9, 2026  
> **Status**: Fully Operational - Dual audio output with hardware switch  
> **Platform**: NVIDIA Jetson AGX Orin 64GB

---

## Executive Summary

The R2D2 audio system provides dual audio output capability with hardware-based switching between PAM8403 onboard speaker and Bluetooth wireless audio. A physical GPIO toggle switch enables instant output selection without software intervention.

**System Status (January 9, 2026):**
- ✅ PAM8403 speaker operational (J511 analog audio output)
- ✅ Bluetooth A2DP operational (FreeBuds 4i, 44.1kHz stereo)
- ✅ GPIO hardware switch functional (Pin 22, automatic switching)
- ✅ Volume control via ROS2 (master volume system)
- ✅ All services auto-start on boot

**Quick Reference:**
- **Switch DOWN** → PAM8403 onboard speaker
- **Switch UP** → Bluetooth wireless audio
- **Manual switch**: `~/dev/r2d2/scripts/audio_switch.sh [bluetooth|pam8403|status]`

---

## Table of Contents

1. [Overview & Architecture](#1-overview--architecture)
2. [Hardware Setup](#2-hardware-setup)
3. [Software Configuration](#3-software-configuration)
4. [Volume Control System](#4-volume-control-system)
5. [Testing & Verification](#5-testing--verification)
6. [Troubleshooting](#6-troubleshooting)
7. [Reference](#7-reference)

---

## 1. Overview & Architecture

### 1.1 Audio System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    R2D2 AUDIO SYSTEM                            │
│                  (Dual Output with Hardware Switch)             │
└─────────────────────────────────────────────────────────────────┘

AUDIO SOURCES (ROS2 Nodes):
├── speech_node (PyAudio)         → Speech TTS output
├── gesture_intent_node (ffplay)  → Gesture feedback beeps
└── audio_notification_node (ffplay) → Recognition alerts
          ↓
    [PulseAudio Layer]
          ↓
┌─────────────────────────────────────────┐
│   GPIO Switch (Pin 22)                  │
│   Monitored by: audio_switch_service.py │
│   ├─ UP (HIGH)   → Bluetooth            │
│   └─ DOWN (LOW)  → PAM8403              │
└─────────────────────────────────────────┘
          ↓
    ┌─────┴─────┐
    ↓           ↓
[BLUETOOTH]  [PAM8403 SPEAKER]
FreeBuds 4i  J511 Header → Amplifier → 8Ω Speaker
A2DP Sink    ALSA → I2S → Analog Output
44.1kHz      48kHz Stereo
```

### 1.2 Design Philosophy

**SIMPLE, STABLE, RELIABLE:**
- Audio outputs are **exclusive** (never both simultaneously)
- Physical switch provides clear, deterministic selection
- No complex parallel audio routing
- Easy debugging (check switch position = know audio output)
- Automatic switching via systemd service
- Configuration persists across reboots

### 1.3 Key Components

| Component | Purpose | Status |
|-----------|---------|--------|
| **PAM8403 Amplifier** | Onboard speaker output | ✅ Operational |
| **8Ω Speaker** | Physical audio output | ✅ Connected |
| **FreeBuds 4i** | Bluetooth wireless audio | ✅ Paired |
| **GPIO Switch** | Hardware output selector | ✅ Installed |
| **Audio Switch Service** | Automatic PulseAudio routing | ✅ Deployed |
| **Audio Routing Service** | ALSA mixer configuration | ✅ Deployed |
| **Volume Control Node** | ROS2 master volume | ✅ Operational |

---

## 2. Hardware Setup

### 2.1 PAM8403 Speaker System (AGX Orin Specific)

#### Hardware Connection

The PAM8403 connects to the Jetson AGX Orin **J511 Audio Header**, which provides analog headphone output.

**J511 Audio Header Pinout:**

```
TOP VIEW (Looking down at board):
        ╔═══════════════════════════╗
Row A:  ║ 1    3    5    7    9     ║
        ║                            ║
Row B:  ║ 2    4    6    8   10     ║
        ╚═══════════════════════════╝
```

| Pin | Signal | Function | Connection |
|-----|--------|----------|------------|
| **2** | **AGND** | **Audio Ground** | **PAM8403 GND** |
| **9** | **HPO_L** | **Headphone LEFT channel** | **PAM8403 RIN input** |

**Wiring:**
- J511 Pin 2 (AGND) → PAM8403 GND
- J511 Pin 9 (HPO_L) → PAM8403 RIN (right input)
- PAM8403 R+ → Speaker positive terminal
- PAM8403 R- → Speaker negative terminal

**Speaker Specifications:**
- Impedance: 8Ω
- Power: 2-3W
- Single channel (mono) from PAM8403 right output

#### ALSA Configuration (Critical for AGX Orin!)

Unlike Jetson Nano, the AGX Orin requires explicit ALSA mixer routing configuration:

**Required Mixer Settings:**
```bash
# Route ADMAIF1 to I2S6 output (feeds J511 headphone codec)
amixer -c 1 cset numid=1218 1  # I2S6 Mux = ADMAIF1

# Set headphone volume to maximum
amixer -c 1 cset numid=310 39   # HP Playback Volume = 39

# Enable headphone output mixer
amixer -c 1 cset numid=1331 on  # HPO MIX HPVOL Switch = on
```

**Why This Is Needed:**
- AGX Orin has complex audio processing engine (APE)
- Digital audio (ADMAIF1) must be routed through I2S6 to reach analog output
- Without this routing, you get only a "deep buzz" instead of clear audio
- These settings do NOT persist across reboots (must be re-applied)

#### Auto-Configuration Service

**Service:** `r2d2-audio-routing.service`  
**Script:** `/home/severin/dev/r2d2/scripts/configure_audio_routing.sh`  
**Purpose:** Automatically configure ALSA mixer routing on every boot

**Installation:**
```bash
# Service already deployed
sudo systemctl status r2d2-audio-routing.service

# Should show: active (exited), enabled
```

**Verification:**
```bash
# Check mixer routing is configured
amixer -c 1 cget numid=1218
# Should show: values=1 (ADMAIF1 routed to I2S6)
```

#### PulseAudio Sink

**Sink Name:** `alsa_output.platform-sound.analog-stereo`

**Verify:**
```bash
pactl list sinks short | grep platform-sound
# Should show: 1  alsa_output.platform-sound.analog-stereo  ...
```

#### Testing PAM8403

```bash
# Direct ALSA test (bypass PulseAudio)
speaker-test -D hw:APE,0 -c 2 -t wav -l 1

# PulseAudio test
pactl set-default-sink alsa_output.platform-sound.analog-stereo
paplay /usr/share/sounds/alsa/Front_Center.wav

# R2D2 beep test
ffplay -autoexit -nodisp ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/Voicy_R2-D2\ -\ 12.mp3
```

**Expected:** Clear audio from speaker, no buzz or distortion.

---

### 2.2 Bluetooth Audio

#### Device Information

| Property | Value |
|----------|-------|
| **Device** | HUAWEI FreeBuds 4i |
| **MAC Address** | `28:54:71:BB:C6:53` |
| **Profile** | A2DP Sink (High Quality Stereo) |
| **Sample Rate** | 44.1kHz |
| **Channels** | 2 (stereo) |
| **Codec** | SBC/AAC |

#### Initial Pairing (One-Time Setup)

```bash
# Enter Bluetooth control
bluetoothctl

# Enable scanning
scan on

# Wait for FreeBuds 4i to appear, then:
pair 28:54:71:BB:C6:53
trust 28:54:71:BB:C6:53
connect 28:54:71:BB:C6:53

# Exit
exit
```

#### Daily Connection

```bash
# Connect FreeBuds
bluetoothctl connect 28:54:71:BB:C6:53

# Verify connection
pactl list sinks short | grep bluez
# Should show: bluez_sink.28_54_71_BB_C6_53.a2dp_sink
```

#### PulseAudio Sink

**Sink Name:** `bluez_sink.28_54_71_BB_C6_53.a2dp_sink`

**Profile:** A2DP (Advanced Audio Distribution Profile)
- High quality stereo audio
- 44.1kHz sample rate
- Low latency for speech

#### Service Environment Variables

All ROS2 audio services require PulseAudio environment variables to access the user's audio session:

```ini
Environment="XDG_RUNTIME_DIR=/run/user/1000"
Environment="PULSE_SERVER=unix:/run/user/1000/pulse/native"
```

**Services requiring these:**
- `r2d2-speech-node.service`
- `r2d2-gesture-intent.service`
- `r2d2-audio-notification.service`
- `r2d2-audio-switch.service`

#### Testing Bluetooth

```bash
# Connect device
bluetoothctl connect 28:54:71:BB:C6:53

# Set as default
pactl set-default-sink bluez_sink.28_54_71_BB_C6_53.a2dp_sink

# Test
paplay /usr/share/sounds/alsa/Front_Center.wav
```

**Expected:** Clear audio from FreeBuds 4i earbuds.

---

### 2.3 GPIO Audio Switch

#### Hardware Specifications

| Component | Specification |
|-----------|--------------|
| **Switch Type** | SPST (Single Pole Single Throw) toggle switch |
| **GPIO Pin** | Pin 22 (GPIO17 on AGX Orin) |
| **Ground Pin** | Pin 9 (GND) |
| **Pull-up Resistor** | 2.2kΩ to 3.3V (Pin 1) |
| **Switch UP** | Open circuit → GPIO HIGH → Bluetooth |
| **Switch DOWN** | Closed to GND → GPIO LOW → PAM8403 |

**⚠️ Important - AGX Orin Specific:**
- Must use **GPIO.BOARD mode** (physical pin numbering)
- Physical **Pin 22** (NOT Pin 11 as in some Nano docs)
- **2.2kΩ resistor** required (10kΩ too weak due to internal pull-down)
- GPIO reads ~3.3V when switch UP, ~0V when switch DOWN

#### Wiring Diagram

```
40-Pin GPIO Header Connections
┌────────────────────────────────────────┐
│  Pin 1 (3.3V) ──┬── 2.2kΩ Resistor ──┬──┐
│                 │                     │  │
│  Pin 22 (GPIO17) ────────────────────┘  │
│                                      │  │
│  Pin 9 (GND) ──── Switch ────────────┘
└────────────────────────────────────────┘

Switch Logic:
- UP (Open):   No connection to GND → Resistor pulls GPIO HIGH → Bluetooth
- DOWN (Closed): Connected to GND → GPIO LOW → PAM8403
```

#### Parts List

- 1× SPST toggle switch (on-off type)
- 1× 2.2kΩ metal film resistor (1/4W)
  - Color bands: Red-Red-Red-Gold
- 3× Female-to-female jumper wires
- Optional: Project box or mounting bracket

#### Installation Steps

**CRITICAL: Power off R2D2 before wiring!**

```bash
sudo shutdown -h now
```

**Step 1: Install pull-up resistor**
- One leg → Pin 1 (3.3V, top-left corner)
- Other leg → Pin 22 (GPIO17, right side, 11th pin down)

**Step 2: Connect switch terminal 1**
- Wire from switch → Pin 22 (GPIO17) - shares connection with resistor

**Step 3: Connect switch terminal 2**
- Wire from switch → Pin 9 (GND, left side, 5th pin down)

**Step 4: Visual inspection**
- Verify no shorts to adjacent pins
- Check resistor connections secure
- Ensure switch toggles freely

**Step 5: Power on R2D2**

#### Testing GPIO Switch (Isolated)

Before deploying the automatic service, test the switch hardware:

```bash
cd ~/dev/r2d2/scripts
python3 test_gpio_switch.py
```

**Expected output:**
```
==================================================
GPIO SWITCH TEST
==================================================
Pin: Physical Pin 22 (GPIO17)
Expected behavior:
  - Switch UP (open):   Should read HIGH (1)
  - Switch DOWN (closed): Should read LOW (0)
==================================================

Flip the switch to test. Press Ctrl+C to exit.

[HH:MM:SS] Switch: UP (HIGH) → Would select: BLUETOOTH
[HH:MM:SS] Switch: DOWN (LOW) → Would select: PAM8403
```

**Success Criteria:**
- [ ] Switch UP reads GPIO HIGH consistently
- [ ] Switch DOWN reads GPIO LOW consistently
- [ ] State changes detected within 1 second
- [ ] No spurious changes when not touching switch

---

## 3. Software Configuration

### 3.1 PulseAudio Sinks

The system uses two PulseAudio sinks for audio output:

| Sink Name | Device | Usage |
|-----------|--------|-------|
| `alsa_output.platform-sound.analog-stereo` | PAM8403 Speaker | Onboard audio output |
| `bluez_sink.28_54_71_BB_C6_53.a2dp_sink` | FreeBuds 4i | Bluetooth wireless audio |

**Check Available Sinks:**
```bash
pactl list sinks short

# Expected output:
# 0  alsa_output.usb-HP__Inc_HyperX_QuadCast_S-00...  (microphone, ignore)
# 1  alsa_output.platform-sound.analog-stereo        (PAM8403)
# 2  bluez_sink.28_54_71_BB_C6_53.a2dp_sink         (Bluetooth)
```

**Check Current Default:**
```bash
pactl get-default-sink
```

**Manual Switch:**
```bash
# Switch to PAM8403
pactl set-default-sink alsa_output.platform-sound.analog-stereo

# Switch to Bluetooth
pactl set-default-sink bluez_sink.28_54_71_BB_C6_53.a2dp_sink
```

---

### 3.2 Audio Switcher Scripts

#### Software Switcher (Manual)

**Script:** `scripts/audio_switch.sh`  
**Purpose:** Manual software-based audio output switching

**Usage:**
```bash
# Switch to Bluetooth
~/dev/r2d2/scripts/audio_switch.sh bluetooth

# Switch to PAM8403
~/dev/r2d2/scripts/audio_switch.sh pam8403

# Check current mode
~/dev/r2d2/scripts/audio_switch.sh status
```

**Output:**
```
✓ Audio output: BLUETOOTH (FreeBuds 4i)
✓ Audio output: PAM8403 (Onboard Speaker)
```

#### GPIO Switch Service (Automatic)

**Script:** `scripts/audio_switch_service.py`  
**Purpose:** Monitors GPIO switch and automatically routes audio

**Features:**
- Polls GPIO Pin 22 every 0.5 seconds
- Detects switch state changes
- Automatically sets PulseAudio default sink
- Logs all switch changes
- Handles Bluetooth unavailable gracefully

**Configuration:**
```python
SWITCH_PIN = 22  # Physical Pin 22 (BOARD mode)
BLUETOOTH_SINK = "bluez_sink.28_54_71_BB_C6_53.a2dp_sink"
PAM8403_SINK = "alsa_output.platform-sound.analog-stereo"
```

**Logic:**
- `GPIO.input(22) == HIGH` → Switch UP → Bluetooth mode
- `GPIO.input(22) == LOW` → Switch DOWN → PAM8403 mode

#### ALSA Mixer Configuration Script

**Script:** `scripts/configure_audio_routing.sh`  
**Purpose:** Configure Jetson AGX Orin ALSA mixer routing on boot

**What It Does:**
```bash
# Route ADMAIF1 to I2S6 (feeds J511 headphone output)
amixer -c 1 cset numid=1218 1

# Set HP volume to max
amixer -c 1 cset numid=310 39

# Enable HP output mixer
amixer -c 1 cset numid=1331 on
```

**Why Needed:** AGX Orin requires explicit audio path routing (different from Nano).

---

### 3.3 SystemD Services

#### Service 1: Audio Routing Configuration

**Service:** `r2d2-audio-routing.service`  
**Type:** oneshot (runs once at boot)  
**Purpose:** Configure ALSA mixer routing for PAM8403

```ini
[Unit]
Description=Configure Jetson AGX Orin Audio Routing for PAM8403
After=sound.target alsa-restore.service

[Service]
Type=oneshot
ExecStart=/home/severin/dev/r2d2/scripts/configure_audio_routing.sh
RemainAfterExit=yes
User=root

[Install]
WantedBy=multi-user.target
```

**Status:** ✅ Enabled and operational

**Check Status:**
```bash
systemctl status r2d2-audio-routing.service
# Should show: active (exited), enabled
```

#### Service 2: GPIO Audio Switch Monitor

**Service:** `r2d2-audio-switch.service`  
**Type:** daemon (runs continuously)  
**Purpose:** Monitor GPIO switch and automatically route audio

```ini
[Unit]
Description=R2D2 Audio Output Switch Monitor
After=pulseaudio.service bluetooth.service
Wants=pulseaudio.service bluetooth.service

[Service]
Type=simple
User=severin
Environment="XDG_RUNTIME_DIR=/run/user/1000"
Environment="PULSE_SERVER=unix:/run/user/1000/pulse/native"
ExecStart=/usr/bin/python3 /home/severin/dev/r2d2/scripts/audio_switch_service.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**Status:** ✅ Enabled and running

**Check Status:**
```bash
systemctl status r2d2-audio-switch.service
# Should show: active (running), enabled

# Monitor logs in real-time
journalctl -u r2d2-audio-switch.service -f
```

**Expected Log Output:**
```
R2D2 Audio Switch Service Started
Switch UP=Bluetooth, Switch DOWN=PAM8403
Switch changed → PAM8403
Audio output: PAM8403 (alsa_output.platform-sound.analog-stereo)
```

#### Service Management

```bash
# Start service
sudo systemctl start r2d2-audio-switch.service

# Stop service
sudo systemctl stop r2d2-audio-switch.service

# Restart service
sudo systemctl restart r2d2-audio-switch.service

# Enable auto-start
sudo systemctl enable r2d2-audio-switch.service

# Disable auto-start
sudo systemctl disable r2d2-audio-switch.service

# View recent logs
journalctl -u r2d2-audio-switch.service -n 50
```

---

## 4. Volume Control System

### 4.1 Architecture

R2D2 implements a centralized master volume control that affects all audio sources:

```
┌─────────────────────────────────────────────────┐
│          volume_control_node (ROS2)             │
│  ┌──────────────────────────────────────────┐   │
│  │ Master Volume (0.0 - 0.7)                │   │
│  │ ROS2 Parameter: master_volume_default    │   │
│  └──────────────┬───────────────────────────┘   │
│                 ↓                                │
│  /r2d2/audio/master_volume (Float32, 10 Hz)     │
└─────────────────┼───────────────────────────────┘
                  │
    ┌─────────────┼─────────────┐
    ↓             ↓             ↓
[audio_notif] [gesture_int] [speech_node]
 (beeps)       (beeps)       (TTS)
```

### 4.2 Volume Levels

| Value | Description | Actual Volume (with 0.7 cap) |
|-------|-------------|------------------------------|
| `0.0` | Mute | 0.0 |
| `0.35` | Default (comfortable) | 0.245 |
| `0.5` | Medium | 0.35 |
| `1.0` | Maximum safe | 0.7 (capped) |

**Volume Calculation:**
```
effective_volume = master_volume × local_audio_volume × max_volume_cap

Example:
- master_volume: 0.35 (from parameter)
- local_audio_volume: 0.1 (in audio_params.yaml)
- max_volume_cap: 0.7 (prevents distortion)
- Effective: 0.35 × 0.1 × 0.7 = 0.0245 (2.45% to audio player)
```

### 4.3 Changing Volume

#### Method 1: ROS2 Parameter (Recommended)

```bash
# Set to 50%
ros2 param set /volume_control_node master_volume_default 0.5

# Set to default (35%)
ros2 param set /volume_control_node master_volume_default 0.35

# Set to mute
ros2 param set /volume_control_node master_volume_default 0.0

# Get current setting
ros2 param get /volume_control_node master_volume_default
```

**Effect:** Immediate, no restart required

#### Method 2: Configuration File (Permanent)

**File:** `ros2_ws/src/r2d2_audio/config/audio_params.yaml`

```yaml
master_volume_default: 0.35  # Change this value
max_volume_cap: 0.7          # Safety limit (prevents distortion)
audio_volume: 0.1            # Local volume for MP3 beeps
```

After editing:
```bash
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_audio
sudo systemctl restart r2d2-volume-control.service
```

#### Method 3: Check Current Volume

```bash
# Echo master volume topic
ros2 topic echo /r2d2/audio/master_volume --once

# Expected: data: 0.35 (or current value)
```

### 4.4 Volume Control Service

**Service:** `r2d2-volume-control.service`  
**Status:** ✅ Operational

```bash
# Check status
systemctl status r2d2-volume-control.service

# View logs
journalctl -u r2d2-volume-control -f
```

### 4.5 Future: Physical Volume Knob

**Status:** Planned, not yet implemented

**Options considered:**
1. Teensy 2.0 USB Serial ADC (hardware available)
2. ADS1115 I2C ADC (recommended for purchase)
3. Rotary encoder (simplest wiring)

**See original [`260_VOLUME_CONTROL_REFERENCE.md`](260_VOLUME_CONTROL_REFERENCE.md) lines 203-315 for detailed implementation plans.**

---

## 5. Testing & Verification

### 5.1 Testing PAM8403 Playback

```bash
# Set switch to DOWN position

# Test 1: Direct ALSA
speaker-test -D hw:APE,0 -c 2 -t wav -l 1
# Expected: Voice saying "Front Left" / "Front Right"

# Test 2: PulseAudio
paplay /usr/share/sounds/alsa/Front_Center.wav
# Expected: Clear "Front Center" voice

# Test 3: R2D2 beep
ffplay -autoexit -nodisp ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/Voicy_R2-D2\ -\ 12.mp3
# Expected: R2D2 sound from speaker
```

### 5.2 Testing Bluetooth Playback

```bash
# Connect FreeBuds
bluetoothctl connect 28:54:71:BB:C6:53

# Set switch to UP position

# Test 1: System sound
paplay /usr/share/sounds/alsa/Front_Center.wav
# Expected: Audio from Bluetooth earbuds

# Test 2: R2D2 beep
ffplay -autoexit -nodisp ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/Voicy_R2-D2\ -\ 12.mp3
# Expected: R2D2 sound from earbuds
```

### 5.3 Testing GPIO Switch Transitions

```bash
# Monitor switch service logs
journalctl -u r2d2-audio-switch.service -f

# Flip switch UP → Should log: "Switch changed → BLUETOOTH"
# Flip switch DOWN → Should log: "Switch changed → PAM8403"
```

**Dynamic Switching Test:**
```bash
# Start continuous audio
speaker-test -t wav -c 2

# While playing:
# - Flip switch UP → Audio should switch to Bluetooth (<0.5s)
# - Flip switch DOWN → Audio should switch to PAM8403 (<0.5s)

# Press Ctrl+C to stop
```

### 5.4 R2D2 Service Integration Tests

#### Test Speech TTS

```bash
cd ~/dev/r2d2
python3 tools/minimal_monitor.py

# Test with switch in BOTH positions:
# 1. Index finger gesture → Start session beep
# 2. Speak to R2D2 → TTS response
# 3. Fist gesture → Stop session beep

# Verify audio plays from correct output based on switch position
```

#### Test Gesture Feedback Beeps

```bash
# Monitor gesture intent service
journalctl -u r2d2-gesture-intent.service -f

# Perform gestures (index finger, fist)
# Verify beeps play through correct output
```

#### Test Recognition Audio Notifications

```bash
# Stand in front of camera until recognized
# Listen for recognition beep
# Walk away, listen for loss beep

# Verify beeps play through correct output
```

### 5.5 Success Criteria

**Basic Functionality:**
- [ ] PAM8403 plays clear audio without buzz/distortion
- [ ] Bluetooth plays clear audio (A2DP mode)
- [ ] Switch detects both UP and DOWN positions
- [ ] Audio output changes within 0.5 seconds of switch flip

**Service Integration:**
- [ ] All R2D2 services use correct audio output
- [ ] Speech TTS works in both positions
- [ ] Gesture beeps work in both positions
- [ ] Recognition alerts work in both positions

**Stability:**
- [ ] Services auto-start on boot
- [ ] Configuration persists after reboot
- [ ] No audio glitches during switching
- [ ] System stable for 1+ hour of use

---

## 6. Troubleshooting

### 6.1 PAM8403 No Audio

**Symptoms:** Speaker silent, no audio output

**Diagnostic Steps:**

**Check 1: ALSA Device**
```bash
aplay -l | grep APE
# Expected: card 1: APE [NVIDIA Jetson AGX Orin APE], device 0: ...
```
If missing: Reboot system, check J511 wiring

**Check 2: ALSA Mixer Routing**
```bash
amixer -c 1 cget numid=1218
# Should show: values=1 (ADMAIF1)

# If values=0, reconfigure:
amixer -c 1 cset numid=1218 1
amixer -c 1 cset numid=310 39
amixer -c 1 cset numid=1331 on
```

**Check 3: Direct ALSA Test**
```bash
speaker-test -D hw:APE,0 -c 2 -t wav
# Should hear clear voice, not buzz
```

**Check 4: PulseAudio Sink**
```bash
pactl list sinks short | grep platform-sound
# Should appear in list

# If missing, restart PulseAudio:
pulseaudio -k
pulseaudio --start
```

**Check 5: Hardware**
- Verify 5V power to PAM8403 (use multimeter)
- Check speaker connection (measure 8Ω resistance)
- Verify J511 wiring (Pin 2 to GND, Pin 9 to RIN)

**Common Causes:**
- ALSA mixer not configured (values=0) → Run configure_audio_routing.sh
- Wrong input used (LIN vs RIN) → Move wire to RIN input
- Speaker disconnected → Check speaker wiring
- PAM8403 not powered → Check 5V supply

---

### 6.2 Bluetooth Connection Issues

**Symptoms:** Cannot connect to FreeBuds, or audio plays through wrong device

**Check 1: Bluetooth Pairing**
```bash
bluetoothctl
devices  # Should show: 28:54:71:BB:C6:53
info 28:54:71:BB:C6:53
# Should show: Paired: yes, Trusted: yes

# If not paired:
pair 28:54:71:BB:C6:53
trust 28:54:71:BB:C6:53
connect 28:54:71:BB:C6:53
```

**Check 2: PulseAudio Bluetooth Module**
```bash
pactl list modules short | grep bluez
# Should show: module-bluez5-device

# If missing:
sudo apt install pulseaudio-module-bluetooth
pulseaudio -k  # Restart PulseAudio
```

**Check 3: A2DP Profile**
```bash
pactl list cards | grep -A 20 "bluez_card"
# Look for: a2dp_sink profile

# If only handsfree available:
pactl set-card-profile bluez_card.28_54_71_BB_C6_53 a2dp_sink
```

**Check 4: Connection**
```bash
bluetoothctl connect 28:54:71:BB:C6:53
# Should show: Connection successful

# If fails: Turn off/on FreeBuds, retry
```

---

### 6.3 GPIO Switch Not Detected

**Symptoms:** No log messages when flipping switch, always selects same output

**Check 1: GPIO Reading**
```bash
python3 -c "import Jetson.GPIO as GPIO; GPIO.setmode(GPIO.BOARD); GPIO.setup(22, GPIO.IN); print(f'Pin 22: {GPIO.input(22)}'); GPIO.cleanup()"

# Flip switch and run again
# Should change between 0 and 1
```

**Check 2: Multimeter Voltage Test**
```bash
# With switch UP: Measure Pin 22 to GND
# Expected: ~3.3V

# With switch DOWN: Measure Pin 22 to GND
# Expected: ~0V
```

**Check 3: Service Logs**
```bash
journalctl -u r2d2-audio-switch.service --no-pager | tail -30
# Should show switch state changes
```

**Check 4: Service Running**
```bash
systemctl status r2d2-audio-switch.service
# Should show: active (running)

# If not running:
sudo systemctl start r2d2-audio-switch.service
```

**Common Causes:**
- Wiring issue (check all 3 connections)
- Wrong resistor value (should be 2.2kΩ, not 10kΩ)
- Service not running → Start service
- Wrong GPIO mode in script → Verify BOARD mode, Pin 22

---

### 6.4 Audio Doesn't Switch

**Symptoms:** Switch detected in logs, but audio continues on wrong output

**Check 1: Default Sink**
```bash
pactl get-default-sink
# Should match switch position:
# - Switch UP → bluez_sink.28_54_71_BB_C6_53.a2dp_sink
# - Switch DOWN → alsa_output.platform-sound.analog-stereo
```

**Check 2: Sinks Exist**
```bash
pactl list sinks short | grep -E "bluez|platform-sound"

# Both should appear:
# 1  alsa_output.platform-sound.analog-stereo  ...
# 2  bluez_sink.28_54_71_BB_C6_53.a2dp_sink   ...
```

**Check 3: Manual Switch Test**
```bash
# Manually set sink
pactl set-default-sink alsa_output.platform-sound.analog-stereo
paplay /usr/share/sounds/alsa/Front_Center.wav
# Should hear from speaker

pactl set-default-sink bluez_sink.28_54_71_BB_C6_53.a2dp_sink  
paplay /usr/share/sounds/alsa/Front_Center.wav
# Should hear from Bluetooth
```

**Check 4: Service Permissions**
```bash
# Verify service has correct environment variables
systemctl cat r2d2-audio-switch.service | grep Environment
# Should show:
# Environment="XDG_RUNTIME_DIR=/run/user/1000"
# Environment="PULSE_SERVER=unix:/run/user/1000/pulse/native"
```

**Common Causes:**
- Bluetooth not connected → Connect FreeBuds first
- Sink names wrong in script → Verify sink names match
- PulseAudio not responding → Restart PulseAudio
- Wrong user session → Check XDG_RUNTIME_DIR points to user 1000

---

### 6.5 "Deep Buzz" Instead of Clear Audio

**Symptoms:** speaker-test plays but you hear only a deep buzz, not voice

**This is ALSA mixer routing issue!**

**Solution:**
```bash
# Reconfigure ALSA mixer
/home/severin/dev/r2d2/scripts/configure_audio_routing.sh

# Or manually:
amixer -c 1 cset numid=1218 1   # Route ADMAIF1 to I2S6
amixer -c 1 cset numid=310 39   # Max HP volume
amixer -c 1 cset numid=1331 on  # Enable HP mixer

# Test again
speaker-test -D hw:APE,0 -c 2 -t wav -l 1
```

**Why It Happens:**
- AGX Orin ALSA mixer defaults to no routing (numid=1218 is 0)
- Audio data sent but not routed to J511 output
- Results in unintelligible buzz instead of clear audio

**Prevention:**
- Ensure `r2d2-audio-routing.service` is enabled
- Service runs automatically on boot
- Settings applied before audio playback attempts

---

### 6.6 Bluetooth Disconnects When Switching to PAM8403

**Symptoms:** Switching to PAM8403 disconnects Bluetooth, requires manual reconnect

**This is normal PulseAudio behavior!**

**What Happens:**
- When PAM8403 becomes default sink, Bluetooth audio stream stops
- FreeBuds may auto-suspend or disconnect after inactivity
- Device stays paired, just audio suspended

**Solutions:**

**Option 1: Keep Bluetooth Connected**
```bash
# After switching back to Bluetooth:
bluetoothctl connect 28:54:71:BB_C6_53
# Reconnects in 1-2 seconds
```

**Option 2: Use Software Switcher**
The manual software switcher doesn't cause disconnection as aggressively:
```bash
~/dev/r2d2/scripts/audio_switch.sh bluetooth
~/dev/r2d2/scripts/audio_switch.sh pam8403
```

**This is expected behavior**, not a bug. Bluetooth devices typically suspend when not actively used.

---

### 6.7 Services Don't Use Correct Audio Output

**Symptoms:** Manual tests work, but R2D2 services use different output

**Check 1: Service Environment**
```bash
# Check speech node service
systemctl cat r2d2-speech-node.service | grep Environment

# Should show PulseAudio variables:
# Environment="XDG_RUNTIME_DIR=/run/user/1000"
# Environment="PULSE_SERVER=unix:/run/user/1000/pulse/native"
```

**Check 2: Default Sink**
```bash
pactl get-default-sink
# Services use this sink
```

**Fix: Add Environment Variables**

Edit service file:
```ini
[Service]
Environment="XDG_RUNTIME_DIR=/run/user/1000"
Environment="PULSE_SERVER=unix:/run/user/1000/pulse/native"
```

Then:
```bash
sudo systemctl daemon-reload
sudo systemctl restart <service-name>
```

---

## 7. Reference

### 7.1 Scripts

| Script | Location | Purpose |
|--------|----------|---------|
| **audio_switch.sh** | `scripts/audio_switch.sh` | Manual software-based audio switching |
| **audio_switch_service.py** | `scripts/audio_switch_service.py` | Automatic GPIO-based switching service |
| **test_gpio_switch.py** | `scripts/test_gpio_switch.py` | GPIO switch hardware testing |
| **configure_audio_routing.sh** | `scripts/configure_audio_routing.sh` | ALSA mixer configuration for boot |

### 7.2 SystemD Services

| Service | Type | Purpose | Auto-Start |
|---------|------|---------|------------|
| **r2d2-audio-routing.service** | oneshot | Configure ALSA mixer on boot | ✅ Enabled |
| **r2d2-audio-switch.service** | daemon | Monitor GPIO and switch audio | ✅ Enabled |
| **r2d2-volume-control.service** | daemon | ROS2 master volume control | ✅ Enabled |

### 7.3 Configuration Files

| File | Purpose |
|------|---------|
| `ros2_ws/src/r2d2_audio/config/audio_params.yaml` | Volume parameters |
| `ros2_ws/src/r2d2_speech/config/speech_params.yaml` | Speech system audio config |
| `r2d2-audio-routing.service` | Audio routing service definition |
| `r2d2-audio-switch.service` | Audio switch service definition |

### 7.4 ALSA Devices

| Device | Card | Description |
|--------|------|-------------|
| `hw:APE,0` | Card 1, Device 0 | Primary audio output (ALSA direct) |
| `hw:APE,1` through `hw:APE,19` | Card 1, Devices 1-19 | Additional ADMAIF devices (not used) |

### 7.5 PulseAudio Sinks

| Sink Name | Description | Usage |
|-----------|-------------|-------|
| `alsa_output.platform-sound.analog-stereo` | PAM8403 onboard speaker | Local audio output |
| `bluez_sink.28_54_71_BB_C6_53.a2dp_sink` | FreeBuds 4i Bluetooth | Wireless audio output |
| `alsa_output.usb-HP__Inc_HyperX_QuadCast_S-00...` | HyperX microphone monitor | Not used for output |

### 7.6 GPIO Pins

| Physical Pin | GPIO Name | Function | Connection |
|--------------|-----------|----------|------------|
| Pin 1 | 3.3V | Power | Pull-up resistor |
| Pin 9 | GND | Ground | Switch terminal 2 |
| Pin 22 | GPIO17 | Input | Switch terminal 1 + resistor |

**Critical:** Use `GPIO.BOARD` mode in code (physical pin numbering).

### 7.7 ALSA Mixer Controls (AGX Orin Specific)

| Control | numid | Purpose | Required Value |
|---------|-------|---------|----------------|
| I2S6 Mux | 1218 | Route audio to J511 output | 1 (ADMAIF1) |
| CVB-RT HP Playback Volume | 310 | Headphone output volume | 39 (maximum) |
| CVB-RT HPO MIX HPVOL Switch | 1331 | Enable HP output mixer | on |

**Verification:**
```bash
amixer -c 1 cget numid=1218  # Should be: values=1
amixer -c 1 cget numid=310   # Should be: values=39,39
amixer -c 1 cget numid=1331  # Should be: values=on
```

### 7.8 Common Commands Reference

**Audio Output Selection:**
```bash
# Manual software switch
~/dev/r2d2/scripts/audio_switch.sh bluetooth
~/dev/r2d2/scripts/audio_switch.sh pam8403
~/dev/r2d2/scripts/audio_switch.sh status

# Check default sink
pactl get-default-sink

# List all sinks
pactl list sinks short
```

**Service Management:**
```bash
# Audio routing service
systemctl status r2d2-audio-routing.service
sudo systemctl restart r2d2-audio-routing.service

# Audio switch service
systemctl status r2d2-audio-switch.service
sudo systemctl restart r2d2-audio-switch.service
journalctl -u r2d2-audio-switch.service -f

# Volume control service
systemctl status r2d2-volume-control.service
sudo systemctl restart r2d2-volume-control.service
```

**Volume Control:**
```bash
# Set volume
ros2 param set /volume_control_node master_volume_default 0.5

# Get volume
ros2 param get /volume_control_node master_volume_default

# Check volume topic
ros2 topic echo /r2d2/audio/master_volume --once
```

**Testing:**
```bash
# Test PAM8403
speaker-test -D hw:APE,0 -c 2 -t wav -l 1
paplay /usr/share/sounds/alsa/Front_Center.wav

# Test Bluetooth
bluetoothctl connect 28:54:71:BB:C6:53
paplay /usr/share/sounds/alsa/Front_Center.wav

# Test GPIO switch
python3 ~/dev/r2d2/scripts/test_gpio_switch.py
```

---

## 8. Technical Details

### 8.1 AGX Orin vs Nano Differences

| Aspect | Jetson Nano | Jetson AGX Orin |
|--------|-------------|-----------------|
| **ALSA Mixer** | Simple, auto-configured | Complex, requires manual routing |
| **GPIO Mode** | BCM mode works | Must use BOARD mode (physical pins) |
| **Pull-up Resistor** | 10kΩ sufficient | 2.2kΩ required (internal pull-down) |
| **Audio Sink Name** | `stereo-fallback` | `analog-stereo` |
| **Pin for GPIO17** | Pin 11 | Pin 22 |
| **ALSA Device** | `hw:APE,0` | `hw:APE,0` (same) |

### 8.2 Why 2.2kΩ Resistor?

**Problem:** AGX Orin GPIO pins have internal pull-down resistors

**With 10kΩ pull-up:**
- External 10kΩ pull-up fights internal ~10kΩ pull-down
- Results in ~1.6V (voltage divider)
- GPIO reads LOW incorrectly

**With 2.2kΩ pull-up:**
- Stronger pull-up overpowers internal pull-down
- Results in ~3.0-3.3V when switch open
- GPIO reads HIGH correctly

**Calculation:**
```
V_out = V_cc × (R_pulldown) / (R_pullup + R_pulldown)

With 10kΩ pull-up vs 10kΩ pull-down:
V_out = 3.3V × 10kΩ / (10kΩ + 10kΩ) = 1.65V ❌ Too low

With 2.2kΩ pull-up vs 10kΩ pull-down:
V_out = 3.3V × 10kΩ / (2.2kΩ + 10kΩ) = 2.7V ✅ Reads as HIGH
```

### 8.3 GPIO Pin Numbering Modes

**BOARD Mode (Physical Pin Numbers):**
```python
GPIO.setmode(GPIO.BOARD)
GPIO.setup(22, GPIO.IN)  # Physical Pin 22
```

**BCM Mode (Broadcom GPIO Numbers - Don't Use on AGX Orin):**
```python
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.IN)  # GPIO17 (doesn't work correctly on AGX Orin)
```

**AGX Orin Requirement:** Must use BOARD mode with physical pin 22.

### 8.4 Audio Routing Pipeline

**PAM8403 Path:**
```
Application (speech_node, ffplay, etc.)
    ↓
PulseAudio (alsa_output.platform-sound.analog-stereo)
    ↓
ALSA (hw:APE,0 = ADMAIF1)
    ↓
Jetson APE (Audio Processing Engine)
    ├─ ADMAIF1 input
    ├─ Routed to I2S6 (numid=1218)
    └─ I2S6 feeds RT5640 codec
    ↓
RT5640 Audio Codec (DAC)
    ↓
J511 Header Pin 9 (HPO_L) - Analog audio output
    ↓
PAM8403 Amplifier RIN input
    ↓
PAM8403 R+ / R- outputs
    ↓
8Ω Speaker
```

**Bluetooth Path:**
```
Application (speech_node, ffplay, etc.)
    ↓
PulseAudio (bluez_sink.28_54_71_BB_C6_53.a2dp_sink)
    ↓
Bluez5 Bluetooth Stack
    ↓
A2DP Profile (SBC/AAC codec)
    ↓
Bluetooth Radio (2.4 GHz)
    ↓
FreeBuds 4i (wireless)
```

---

## 9. Installation Summary

### 9.1 What Was Installed (January 8-9, 2026)

**Hardware:**
- New PAM8403 audio amplifier board
- 8Ω speaker connection
- SPST toggle switch
- 2.2kΩ metal film resistor
- Wiring to J511 header and GPIO Pin 22

**Software:**
- `configure_audio_routing.sh` - ALSA mixer configuration script
- `r2d2-audio-routing.service` - Boot-time mixer configuration
- Updated `audio_switch.sh` - Correct sink names
- Updated `audio_switch_service.py` - BOARD mode, Pin 22, better error handling
- Updated `test_gpio_switch.py` - BOARD mode, Pin 22
- `r2d2-audio-switch.service` - Automatic switching daemon

### 9.2 Key Learnings

**AGX Orin Audio Path Discovery:**
- J511 provides analog headphone output (not direct I2S digital)
- ALSA mixer routing is NOT automatic (unlike Nano)
- Mixer controls must be configured on every boot
- Testing revealed correct routing: ADMAIF1 → I2S6 → J511

**GPIO Pull-up Resistor Discovery:**
- 10kΩ resistor insufficient (internal pull-down interference)
- 2.2kΩ required to overpower internal pull-down
- Measured voltages confirmed: 1V with 10kΩ, 3.3V with 2.2kΩ

**GPIO Pin Mode Discovery:**
- BCM mode doesn't work correctly on AGX Orin for Pin 22
- BOARD mode (physical pin numbering) required
- Pin 22 is GPIO17 on AGX Orin (different mapping than Raspberry Pi)

---

## 10. Related Documentation

**Speech System:**
- [`200_SPEECH_SYSTEM_REFERENCE.md`](200_SPEECH_SYSTEM_REFERENCE.md) - Speech system architecture
- [`201_SPEECH_SYSTEM_INSTALLATION.md`](201_SPEECH_SYSTEM_INSTALLATION.md) - Installation guide
- [`203_SPEECH_SYSTEM_TROUBLESHOOTING.md`](203_SPEECH_SYSTEM_TROUBLESHOOTING.md) - Speech troubleshooting

**Hardware:**
- [`002_HARDWARE_REFERENCE.md`](002_HARDWARE_REFERENCE.md) - Complete hardware reference
- [`001_ARCHITECTURE_OVERVIEW.md`](001_ARCHITECTURE_OVERVIEW.md) - System architecture

**Services:**
- [`005_SYSTEMD_SERVICES_REFERENCE.md`](005_SYSTEMD_SERVICES_REFERENCE.md) - Service management

**User Guide:**
- [`000_UX_AND_FUNCTIONS.md`](000_UX_AND_FUNCTIONS.md) - User experience overview

---

## Appendix A: Mono Audio Support (Single Earbud Use)

All audio sources output **dual-mono stereo** - identical content on both L+R channels. This ensures users with only one earbud (left OR right) hear all audio without loss.

**Implementation:**
- **Speech TTS:** AudioPlayback class duplicates mono to stereo
- **Gesture Beeps:** ffplay uses pan filter `pan=stereo|c0=c0|c1=c0`
- **Audio Files:** R2D2 beeps are mono, automatically duplicated

**Testing:**
```bash
# Test with single earbud - should hear full audio
ffplay -autoexit -nodisp -af 'pan=stereo|c0=c0|c1=c0' \
  ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/Voicy_R2-D2\ -\ 12.mp3
```

---

## Appendix B: Quick Troubleshooting Flowchart

```
Audio Problem?
│
├─ No audio at all?
│  ├─ Check volume: ros2 param get /volume_control_node master_volume_default
│  ├─ Check default sink: pactl get-default-sink
│  └─ Check device connected: aplay -l (PAM8403) or bluetoothctl (Bluetooth)
│
├─ Buzz instead of clear audio?
│  └─ ALSA mixer routing issue → Run configure_audio_routing.sh
│
├─ Switch doesn't work?
│  ├─ Check service: systemctl status r2d2-audio-switch.service
│  ├─ Check logs: journalctl -u r2d2-audio-switch.service -f
│  └─ Test GPIO: python3 test_gpio_switch.py
│
└─ Audio on wrong output?
   ├─ Check switch position (flip to change)
   ├─ Check default sink: pactl get-default-sink
   └─ Manually set: ~/dev/r2d2/scripts/audio_switch.sh [bluetooth|pam8403]
```

---

## Appendix C: Historical Notes

**Previous Issues (Resolved):**

1. **PAM8403 Hardware Failure (Dec 2025)**
   - **Issue:** Original board damaged during soldering
   - **Resolution:** New board installed January 8, 2026
   - **Status:** ✅ Resolved

2. **Bluetooth HFP-Only Connection (Dec 2025)**
   - **Issue:** FreeBuds connecting in HFP (16kHz mono) instead of A2DP
   - **Resolution:** Switched to FreeBuds 4i (different device)
   - **Status:** ✅ Resolved (A2DP working)

3. **ALSA Routing "Deep Buzz" (Jan 2026)**
   - **Issue:** Audio data sent but not routed to J511 output
   - **Resolution:** Discovered mixer routing requirement (numid 1218, 310, 1331)
   - **Status:** ✅ Resolved with auto-configuration service

4. **GPIO Switch 10kΩ Resistor Failure (Jan 2026)**
   - **Issue:** 10kΩ pull-up insufficient, GPIO read 1V instead of 3.3V
   - **Resolution:** Switched to 2.2kΩ resistor (overpowers internal pull-down)
   - **Status:** ✅ Resolved

5. **GPIO BCM Mode Not Working (Jan 2026)**
   - **Issue:** GPIO.BCM mode didn't read Pin 22 correctly
   - **Resolution:** Must use GPIO.BOARD mode on AGX Orin
   - **Status:** ✅ Resolved

---

**Document Version:** 1.0 (Consolidation of 260, 261, 262, 280, 999)  
**Last Updated:** January 9, 2026  
**Maintainer:** R2D2 Project


