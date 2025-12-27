# R2D2 Volume Control Reference

> **Last Updated**: 2025-12-27  
> **Status**: Software-only mode active (hardware knob pending)

## Overview

Centralized master volume control for all R2D2 audio sources. The volume control node publishes to a ROS2 topic that all audio nodes subscribe to, providing unified volume adjustment across:

- MP3 notification beeps (recognition, loss alerts)
- Gesture feedback sounds  
- Speech TTS (OpenAI Realtime API responses)

## Quick Start

### Check Current Volume

```bash
# Echo the master volume topic
ros2 topic echo /r2d2/audio/master_volume --once
```

### Change Volume

```bash
# Method 1: ROS2 Parameter (recommended - instant effect)
ros2 param set /volume_control_node master_volume_default 0.5

# Examples:
ros2 param set /volume_control_node master_volume_default 0.0   # Mute
ros2 param set /volume_control_node master_volume_default 0.35  # Default
ros2 param set /volume_control_node master_volume_default 1.0   # Maximum (capped to 0.7)
```

### Volume Levels

| Value | Description |
|-------|-------------|
| `0.0` | Mute |
| `0.35` | Default (35% of max) |
| `0.5` | Medium (50% of max) |
| `1.0` | Maximum safe volume |

**Note**: All values are scaled to the `max_volume_cap` (default 0.7) to prevent distortion.

---

## Volume Control System

### Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    volume_control_node                          │
│  ┌────────────────┐    ┌─────────────────────────────────────┐  │
│  │ Hardware Input │    │ Software Input                      │  │
│  │ (future)       │    │ - ros2 param set                    │  │
│  │ - ADC/Knob     │    │ - ros2 service call                 │  │
│  └───────┬────────┘    └────────────────┬────────────────────┘  │
│          │                              │                        │
│          └──────────────┬───────────────┘                        │
│                         ▼                                        │
│              ┌─────────────────────┐                             │
│              │ Master Volume       │                             │
│              │ (0.0 - 0.7)         │                             │
│              └──────────┬──────────┘                             │
│                         │                                        │
│                         ▼                                        │
│              /r2d2/audio/master_volume (Float32)                 │
└─────────────────────────┼───────────────────────────────────────┘
                          │
        ┌─────────────────┼─────────────────┐
        ▼                 ▼                 ▼
┌───────────────┐ ┌───────────────┐ ┌───────────────┐
│audio_notif... │ │gesture_intent │ │ speech_node   │
│ node          │ │ node          │ │               │
│               │ │               │ │               │
│ effective =   │ │ effective =   │ │ PCM scaling   │
│ master *      │ │ master *      │ │ by master     │
│ audio_volume  │ │ audio_volume  │ │ volume        │
└───────────────┘ └───────────────┘ └───────────────┘
```

### Volume Calculation

Each audio node applies the master volume as a multiplier:

```
effective_volume = master_volume × local_audio_volume
```

Example:
- Master volume: 0.35 (35% from knob/parameter)
- Local audio_volume: 0.1 (10% in audio_params.yaml)
- Effective: 0.035 (3.5% to ffplay)

### Parameters

Defined in `ros2_ws/src/r2d2_audio/config/audio_params.yaml`:

```yaml
# Master volume (controlled by this system)
master_volume_default: 0.35  # 35% of max_volume_cap
max_volume_cap: 0.7          # Prevents distortion (from baseline tests)

# Local volumes (per-node)
audio_volume: 0.1            # 10% for MP3 beeps
```

---

## Installation

### 1. Build the Interfaces Package

```bash
cd ~/dev/r2d2/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select r2d2_interfaces r2d2_audio
source install/setup.bash
```

### 2. Install Systemd Service

```bash
# Copy service file
sudo cp ~/dev/r2d2/ros2_ws/src/r2d2_audio/config/r2d2-volume-control.service \
    /etc/systemd/system/

# Reload systemd
sudo systemctl daemon-reload

# Enable and start
sudo systemctl enable r2d2-volume-control.service
sudo systemctl start r2d2-volume-control.service

# Check status
sudo systemctl status r2d2-volume-control.service
```

### 3. Verify Operation

```bash
# Check the topic is being published
ros2 topic echo /r2d2/audio/master_volume --once

# Check the node is running
ros2 node list | grep volume

# View logs
journalctl -u r2d2-volume-control -f
```

---

## Software Volume Control

### Method 1: ROS2 Parameter (Recommended)

Change volume dynamically at runtime:

```bash
# Set to 50% (maps to 0.35 actual volume with 0.7 cap)
ros2 param set /volume_control_node master_volume_default 0.5

# Set to mute
ros2 param set /volume_control_node master_volume_default 0.0

# Set to maximum (will be capped to max_volume_cap = 0.7)
ros2 param set /volume_control_node master_volume_default 1.0

# Get current setting
ros2 param get /volume_control_node master_volume_default
```

**Volume Mapping**:
- Input: 0.0 to 1.0 (what you set)
- Output: 0.0 to 0.7 (actual volume, scaled by max_volume_cap)

| You Set | Actual Volume | Description |
|---------|---------------|-------------|
| 0.0 | 0.0 | Mute |
| 0.35 | 0.245 | Default |
| 0.5 | 0.35 | Medium |
| 1.0 | 0.7 | Maximum safe |

### Method 3: Configuration File

Edit `audio_params.yaml` for permanent default:

```yaml
master_volume_default: 0.35  # Change this value
```

Then restart the service:

```bash
sudo systemctl restart r2d2-volume-control.service
```

---

## Adding a Physical Volume Knob (Future)

The system is designed to support a physical potentiometer. Here are three options:

### Option 1: Teensy 2.0 (USB Serial ADC)

**Hardware you have available.** Uses Teensy's built-in ADC and communicates via USB serial.

#### Wiring

```
Potentiometer B5K:
  - Pin 1 (CCW) → GND
  - Pin 2 (Wiper) → Teensy Pin F0 (A5 / ADC0)
  - Pin 3 (CW) → Teensy VCC (5V)

Teensy 2.0:
  - USB → Jetson USB port
  - VCC → (from USB)
  - GND → (from USB)
```

#### Teensy Arduino Sketch

```cpp
// volume_knob.ino - Teensy 2.0 Volume Knob
const int POT_PIN = A5;  // F0 pin

void setup() {
    Serial.begin(115200);
    pinMode(POT_PIN, INPUT);
}

void loop() {
    int value = analogRead(POT_PIN);  // 0-1023
    Serial.println(value);
    delay(50);  // 20Hz
}
```

#### Code Changes Required

In `volume_control_node.py`, add USB serial reading:

```python
import serial

# In __init__:
self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)

# In _poll_callback:
if self.serial_port.in_waiting:
    line = self.serial_port.readline().decode().strip()
    raw_value = int(line)  # 0-1023
    normalized = raw_value / 1023.0
    volume = self._apply_dead_zones(normalized)
    self.current_volume = volume
```

### Option 2: ADS1115 I2C ADC (Standard Solution)

**Recommended if you need to purchase hardware.** No microcontroller needed.

#### Wiring

```
Potentiometer B5K:
  - Pin 1 (CCW) → GND
  - Pin 2 (Wiper) → ADS1115 A0
  - Pin 3 (CW) → 3.3V

ADS1115 Module:
  - VDD → Pin 1 (3.3V) on Jetson 40-pin header
  - GND → Pin 6 (GND)
  - SCL → Pin 5 (I2C SCL)
  - SDA → Pin 3 (I2C SDA)
```

#### Software Setup

```bash
# Install library
pip3 install adafruit-circuitpython-ads1x15

# Test I2C detection
i2cdetect -y 1  # Should show 0x48 for ADS1115
```

Then in systemd service, change:

```ini
ExecStart=... -p hardware_enabled:=true
```

### Option 3: Rotary Encoder (No ADC Needed)

**Simplest wiring**, but incremental (not absolute position).

#### Wiring

```
Rotary Encoder:
  - CLK → GPIO pin (e.g., Pin 11 / GPIO17)
  - DT  → GPIO pin (e.g., Pin 13 / GPIO27)
  - SW  → GPIO pin (optional, for push button)
  - +   → 3.3V
  - GND → GND
```

#### Code Changes Required

Use `RPi.GPIO` or `gpiod` to read encoder pulses and increment/decrement volume.

---

## Baseline Test Results

From testing on 2025-12-27 (see `ros2_ws/src/r2d2_audio/test/BASELINE_RESULTS.md`):

| Finding | Value |
|---------|-------|
| Distortion threshold | 0.7 |
| Minimum audible | 0.01 - 0.05 |
| Comfortable range | 0.05 - 0.3 |
| Default setting | 0.35 (middle of safe range) |

The `max_volume_cap` of 0.7 prevents any configuration from causing audio distortion.

---

## Troubleshooting

### No Audio

1. Check if volume control node is running:
   ```bash
   ros2 node list | grep volume
   ```

2. Check current volume:
   ```bash
   ros2 topic echo /r2d2/audio/master_volume --once
   ```

3. If volume is 0, set it higher:
   ```bash
   ros2 param set /volume_control_node master_volume_default 0.5
   ```

### Volume Not Changing

1. Check if parameter change is accepted:
   ```bash
   ros2 param get /volume_control_node master_volume_default
   ```

2. Check node logs:
   ```bash
   journalctl -u r2d2-volume-control -f
   ```

3. Restart the service:
   ```bash
   sudo systemctl restart r2d2-volume-control.service
   ```

### Audio Distortion

1. Lower the volume:
   ```bash
   ros2 param set /volume_control_node master_volume_default 0.3
   ```

2. Check if `max_volume_cap` is correctly set (should be 0.7)

### Service Won't Start

1. Check for errors:
   ```bash
   journalctl -u r2d2-volume-control -e
   ```

2. Ensure workspace is built:
   ```bash
   cd ~/dev/r2d2/ros2_ws
   source /opt/ros/humble/setup.bash
   colcon build --packages-select r2d2_interfaces r2d2_audio
   ```

---

## Files Reference

| File | Purpose |
|------|---------|
| `ros2_ws/src/r2d2_audio/r2d2_audio/volume_control_node.py` | Main node |
| `ros2_ws/src/r2d2_audio/config/audio_params.yaml` | Volume parameters |
| `ros2_ws/src/r2d2_audio/config/r2d2-volume-control.service` | Systemd service |
| `ros2_ws/src/r2d2_interfaces/srv/SetVolume.srv` | Service definition |
| `ros2_ws/src/r2d2_audio/test/BASELINE_RESULTS.md` | Baseline test data |
| `~/.r2d2/volume_state.json` | Persisted volume (auto-created) |
| `~/.r2d2/adc_calibration.json` | ADC calibration (for hardware mode) |
