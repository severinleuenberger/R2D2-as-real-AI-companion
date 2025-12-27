# 260 - Volume Control Reference

## Overview

Centralized audio volume control system using a **physical B5K (5kΩ) rotary potentiometer** connected via an ADS1115 ADC. The system uses **software volume multipliers** since the J511 audio hardware does not support ALSA mixer controls.

## Architecture

### Component Diagram

```
┌──────────────────────┐     ┌──────────────────┐     ┌────────────────────────────┐
│ Physical Potentiometer │     │  ADS1115 ADC     │     │   volume_control_node      │
│     B5K (5kΩ)         │────►│   (I2C, 16-bit)  │────►│   (ROS2)                   │
│  0-3.3V analog        │     │   0x48 address   │     │                            │
└──────────────────────┘     └──────────────────┘     │ Publishes:                 │
                                                       │ /r2d2/audio/master_volume  │
                                                       │   (Float32, 0.0-1.0)       │
                                                       └────────────┬───────────────┘
                                                                    │
           ┌────────────────────────────────────────────────────────┼──────────────────┐
           │                                                        │                  │
           ▼                                                        ▼                  ▼
┌────────────────────────┐     ┌────────────────────────┐     ┌────────────────────────┐
│ audio_notification_node │     │  gesture_intent_node   │     │     speech_node        │
│ (Recognition/Loss beeps)│     │ (Gesture feedback)     │     │ (OpenAI TTS playback)  │
│                        │     │                        │     │                        │
│ effective_volume =     │     │ effective_volume =     │     │ AudioPlayback.         │
│ master × audio_volume  │     │ master × audio_volume  │     │ set_master_volume()    │
└────────────────────────┘     └────────────────────────┘     └────────────────────────┘
           │                              │                              │
           │                              │                              │
           ▼                              ▼                              ▼
┌──────────────────────────────────────────────────────────────────────────────────────┐
│                       PAM8403 Amplifier + Speaker (hw:1,0)                           │
└──────────────────────────────────────────────────────────────────────────────────────┘
```

### Volume Calculation

```python
effective_volume = master_volume × local_volume
# Example: 0.5 (knob at 50%) × 0.02 (local 2%) = 0.01 (final 1%)
```

## Hardware Setup

### Components Required

| Component | Specification | Purpose |
|-----------|---------------|---------|
| Potentiometer | B5K, 5kΩ, linear taper | Analog voltage divider |
| ADC Module | ADS1115, 16-bit, I2C | Analog-to-digital conversion |
| Wiring | Jumper wires, 3.3V compatible | Connections |

### Wiring Diagram

```
Potentiometer:
├── Pin 1 (CCW terminal) → GND
├── Pin 2 (Wiper)        → ADS1115 A0
└── Pin 3 (CW terminal)  → 3.3V

ADS1115:
├── VDD → 3.3V (Jetson Pin 1 or 17)
├── GND → GND  (Jetson Pin 6, 9, 14, 20, 25, 30, 34, or 39)
├── SCL → I2C SCL (Jetson Pin 5, GPIO 3)
├── SDA → I2C SDA (Jetson Pin 3, GPIO 2)
└── A0  → Potentiometer wiper
```

### Jetson AGX Orin 40-Pin Header Reference

```
3.3V  (1) (2)  5V
SDA   (3) (4)  5V       ← I2C Data
SCL   (5) (6)  GND      ← I2C Clock, Ground
GPIO7 (7) (8)  TXD
GND   (9) (10) RXD
...
```

## Software Components

### 1. Volume Control Node

**Location**: `ros2_ws/src/r2d2_audio/r2d2_audio/volume_control_node.py`

**Function**: Reads potentiometer via ADC and publishes master volume

**ROS2 Topic**: `/r2d2/audio/master_volume` (std_msgs/Float32)

**Parameters**:
| Parameter | Default | Description |
|-----------|---------|-------------|
| `adc_i2c_bus` | 1 | I2C bus number (0-8 on Jetson) |
| `adc_address` | 0x48 | ADS1115 I2C address |
| `adc_channel` | 0 | ADC channel (0-3) |
| `poll_rate_hz` | 10.0 | ADC polling rate |
| `volume_smoothing` | true | Enable exponential smoothing |
| `smoothing_alpha` | 0.2 | Smoothing factor (0.0-1.0) |
| `min_change_threshold` | 0.01 | Minimum change to publish |
| `dead_zone_low` | 0.05 | Map 0-5% ADC to 0.0 |
| `dead_zone_high` | 0.05 | Map 95-100% ADC to 1.0 |
| `master_volume_default` | 0.5 | Default if no ADC |
| `hardware_enabled` | true | Enable ADC hardware |

**Usage**:
```bash
# Run standalone
ros2 run r2d2_audio volume_control_node

# With parameters
ros2 run r2d2_audio volume_control_node --ros-args -p poll_rate_hz:=20.0

# Monitor volume
ros2 topic echo /r2d2/audio/master_volume
```

### 2. Audio Parameters Configuration

**Location**: `ros2_ws/src/r2d2_audio/config/audio_params.yaml`

```yaml
# Master volume control (set by volume knob, read by all nodes)
master_volume_default: 0.5  # Default on startup (0.0-1.0)

# Individual volume multipliers (relative to master)
audio_notification_volume: 1.0  # Recognition/loss beeps
gesture_feedback_volume: 1.0    # Gesture acknowledgment beeps
speech_tts_volume: 1.0          # OpenAI speech responses

# Local audio volume (per-node default)
audio_volume: 0.02  # Content volume for MP3 files
```

### 3. Audio Nodes Integration

All audio-playing nodes subscribe to `/r2d2/audio/master_volume` and apply the multiplier:

**audio_notification_node.py**:
```python
# In _play_audio_file():
effective_volume = self.master_volume * self.audio_volume
subprocess.Popen(['ffplay', ... '-af', f'volume={effective_volume}', ...])
```

**gesture_intent_node.py**:
```python
# In _play_audio_feedback():
effective_volume = self.master_volume * self.audio_volume
subprocess.Popen(['ffplay', ... '-af', f'volume={effective_volume}', ...])
```

**AudioPlayback (speech TTS)**:
```python
# In play_chunk():
audio_float = audio_array.astype(np.float32) * self.master_volume
audio_array = np.clip(audio_float, -32768, 32767).astype(np.int16)
```

## Baseline Testing

### Test Script

**Location**: `ros2_ws/src/r2d2_audio/test/test_volume_baseline.py`

**Purpose**: Establish min/max volume levels and create calibration data

**Usage**:
```bash
# Interactive mode (recommended for first baseline)
python3 test_volume_baseline.py --mode interactive --output test_results/baseline.json

# Automatic mode (plays all sounds)
python3 test_volume_baseline.py --mode automatic --output test_results/auto.json

# Verify against baseline
python3 test_volume_baseline.py --verify test_results/baseline.json

# List available audio sources
python3 test_volume_baseline.py --list-sources
```

**Output Format** (JSON):
```json
{
  "test_date": "2025-12-27T10:30:00",
  "hardware": {
    "device": "hw:1,0",
    "amplifier": "PAM8403",
    "speaker": "8Ω"
  },
  "baseline_results": {
    "recognition_beep": {
      "0.01": {"rating": 1, "notes": "Inaudible"},
      "0.02": {"rating": 2, "notes": "Quiet, acceptable"},
      "0.05": {"rating": 3, "notes": "Good volume"},
      ...
    }
  },
  "recommendations": {
    "min_volume": 0.01,
    "default_volume": 0.02,
    "max_volume": 0.3,
    "optimal_range": "0.02 - 0.1"
  }
}
```

## Systemd Service

### Service File

**Location**: `/etc/systemd/system/r2d2-volume-control.service`

**Installation**:
```bash
# Copy service file
sudo cp ros2_ws/src/r2d2_audio/config/r2d2-volume-control.service /etc/systemd/system/

# Enable and start
sudo systemctl daemon-reload
sudo systemctl enable r2d2-volume-control.service
sudo systemctl start r2d2-volume-control.service

# Check status
sudo systemctl status r2d2-volume-control.service

# View logs
journalctl -u r2d2-volume-control.service -f
```

## Troubleshooting

### ADC Not Detected

1. Check I2C connection:
```bash
i2cdetect -y 1  # Try buses 0-8
```

2. Verify ADS1115 at address 0x48

3. Check wiring (SDA, SCL, VDD, GND)

### Volume Not Changing

1. Check topic is publishing:
```bash
ros2 topic echo /r2d2/audio/master_volume
```

2. Verify nodes are subscribed:
```bash
ros2 topic info /r2d2/audio/master_volume
```

3. Test audio directly:
```bash
ffplay -nodisp -autoexit -af volume=0.1 test.mp3
```

### Jittery Volume

Increase smoothing:
```bash
ros2 run r2d2_audio volume_control_node --ros-args -p smoothing_alpha:=0.1
```

Or increase threshold:
```bash
ros2 run r2d2_audio volume_control_node --ros-args -p min_change_threshold:=0.02
```

### Software-Only Mode

If ADC hardware is not available, the node runs in software-only mode:
```bash
ros2 run r2d2_audio volume_control_node --ros-args -p hardware_enabled:=false
```

Volume can then be controlled via:
```bash
ros2 topic pub /r2d2/audio/master_volume std_msgs/Float32 "data: 0.5"
```

## Volume Mapping

### Linear vs Logarithmic

The default mapping is **linear**: knob position directly maps to volume percentage.

For perceived loudness (dB scale), consider logarithmic mapping:
```python
# In volume_control_node.py
def _apply_logarithmic_curve(self, linear_volume):
    """Convert linear to logarithmic for perceived loudness."""
    if linear_volume <= 0:
        return 0.0
    # dB = 20 * log10(linear)
    # Mapped to 0.0-1.0 range
    return linear_volume ** 0.4  # Approximate logarithmic curve
```

### Dead Zones

- **Low dead zone** (0-5%): Maps to 0.0 (mute)
- **High dead zone** (95-100%): Maps to 1.0 (max)
- **Active range**: Linear 0.0-1.0 between dead zones

## Integration Checklist

- [ ] Wire potentiometer to ADS1115
- [ ] Connect ADS1115 to Jetson I2C (pins 3, 5)
- [ ] Verify I2C detection (`i2cdetect -y 1`)
- [ ] Install Python library: `pip3 install adafruit-circuitpython-ads1x15`
- [ ] Run baseline tests
- [ ] Start volume_control_node
- [ ] Verify all audio nodes respond to volume changes
- [ ] Enable systemd service for auto-start

## Related Documents

- [002_HARDWARE_REFERENCE.md](002_HARDWARE_REFERENCE.md) - Hardware wiring details
- [005_SYSTEMD_SERVICES_REFERENCE.md](005_SYSTEMD_SERVICES_REFERENCE.md) - Service management
- [100_PERCEPTION_STATUS_REFERENCE.md](100_PERCEPTION_STATUS_REFERENCE.md) - Audio notification integration

