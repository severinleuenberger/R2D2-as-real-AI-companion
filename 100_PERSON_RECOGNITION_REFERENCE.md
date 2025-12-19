# R2D2 Person Recognition System - Reference Documentation
## Complete Architecture and Technical Reference

**Date:** December 17, 2025  
**Status:** ✅ COMPLETE AND OPERATIONAL  
**Platform:** NVIDIA Jetson AGX Orin 64GB + ROS 2 Humble  
**Hardware:** OAK-D Lite Camera + PAM8403 Speaker + Optional RGB LED

---

## Executive Summary

The R2D2 person recognition system provides real-time face recognition, audio alerts, visual feedback (LED), and state management for person tracking. The system enables intelligent interaction through a sophisticated state machine that recognizes specific individuals, tracks their presence, and provides multi-modal feedback.

**Key Capabilities:**
- Real-time person recognition at 6-13 Hz
- Smart state management with jitter tolerance and loss confirmation
- Audio alerts: "Hello!" on recognition, "Oh, I lost you!" on loss
- Visual feedback: RGB LED shows system state (RED/BLUE/GREEN)
- Production-ready: Systemd services, error handling, logging
- Training pipeline: Complete workflow for LBPH model creation

**For installation instructions**, see: [`101_PERSON_RECOGNITION_INSTALLATION.md`](101_PERSON_RECOGNITION_INSTALLATION.md)  
**For quick reference**, see: [`102_PERSON_RECOGNITION_QUICK_START.md`](102_PERSON_RECOGNITION_QUICK_START.md)

---

## Architecture Overview

### System Diagram

```
OAK-D Camera → Perception Pipeline → Face Recognition → Status Machine → Audio/LED Output
     ↓              ↓                      ↓                  ↓                ↓
  30 FPS RGB    Image Processing      Person ID         RED/BLUE/GREEN    🔊 Beeps + 🔴🟢🔵 LEDs
```

### Complete Data Flow

```
OAK-D Lite Camera (30 FPS)
    ↓
r2d2_camera node
    ↓
/oak/rgb/image_raw (30 Hz)
    ↓
r2d2_perception node (image_listener)
    ├─ Downscale (1920×1080 → 640×360)
    ├─ Grayscale conversion
    ├─ Brightness computation → /r2d2/perception/brightness (13 Hz)
    ├─ Face detection (Haar Cascade, raw)
    ├─ Hysteresis Filter (temporal smoothing, 2s/5s thresholds)
    ├─ Stable face_count → /r2d2/perception/face_count (13 Hz, smoothed)
    └─ Face recognition (LBPH, gated) → /r2d2/perception/person_id (6.5 Hz)
    ↓
r2d2_audio package
    ├─ audio_notification_node: State machine (RED/BLUE/GREEN)
    │  ├─ Subscribes: /r2d2/perception/person_id
    │  ├─ Publishes: /r2d2/audio/person_status (JSON, 10 Hz)
    │  └─ Plays: MP3 audio alerts (ffplay)
    ├─ status_led_node: RGB LED control (GPIO)
    │  ├─ Subscribes: /r2d2/audio/person_status
    │  └─ Controls: GPIO pins 17, 27, 22
    └─ database_logger_node: Event logging
       └─ Subscribes: /r2d2/audio/person_status
```

---

## Core Components

### 1. Perception Pipeline (r2d2_perception)

**Purpose:** Processes camera frames and provides face detection

**Key Responsibilities:**
- Subscribe to `/oak/rgb/image_raw` (camera frames)
- Downscale images (1920×1080 → 640×360) for efficiency
- Detect faces using Haar Cascade classifier
- Apply temporal smoothing (hysteresis filter) to stabilize detections
- Publish face count and brightness metrics
- Optional: LBPH face recognition for person identification

**Key Methods:**
- `process_frame()`: Main image processing loop
- `detect_faces()`: Haar Cascade face detection
- `recognize_face()`: LBPH person identification
- `compute_brightness()`: Mean brightness calculation

**Performance:**
- Frame processing rate: 12-13 FPS
- CPU usage: 8-10% (without recognition), 10-15% (with recognition)
- Memory usage: ~200 MB

**Hysteresis Filter (Temporal Smoothing):**
- **Purpose:** Filters out flickering face detections to prevent false positives
- **Implementation:** State machine with time-based thresholds
- **Parameters:**
  - `face_presence_threshold`: 2.0s (must detect face continuously before confirming presence)
  - `face_absence_threshold`: 5.0s (must lose face continuously before confirming absence)
- **Effect:** Published `face_count` is stable (0 or 1 only), preventing timer resets in audio node
- **Benefit:** "Lost you!" beep plays reliably at ~20s after person leaves (5s + 15s)

### 2. Face Recognition (LBPH Recognizer)

**Algorithm:** Local Binary Pattern Histograms (LBPH)

**Training Requirements:**
- ~80+ diverse images of target person
- Different lighting conditions, angles, distances
- Trained model saved as XML file

**Recognition Process:**
1. Extract face region from frame using Haar Cascade
2. Apply LBPH algorithm to compute face descriptor
3. Compare descriptor against trained model
4. Return person ID and confidence score

**Confidence Interpretation:**
- Lower scores = better match (distance-based metric)
- Threshold: 70.0 (default, configurable)
- Typical range: 35-50 for target person, 80-120 for unknown

### 3. Audio Notification System (audio_notification_node)

**Purpose:** State machine that tracks person recognition and drives audio alerts

**Key Responsibilities:**
- Subscribe to `/r2d2/perception/person_id`
- Track recognition state (RED/BLUE/GREEN)
- Play MP3 audio alerts on state transitions
- Publish status JSON for LED and other consumers
- Implement jitter tolerance and loss confirmation

**State Machine Logic (RED-Primary Design):**
- **RED is Primary:** While target person is recognized, ALL other face detections are IGNORED
- **RED Status Timer:** 15 seconds (resets on each target person recognition)
- **Post-RED Transition:** When RED times out → GREEN (face visible) or BLUE (no face)
- **GREEN Entry Delay:** 2 seconds of stable face detection before BLUE→GREEN
- **BLUE Entry Delay:** 3 seconds of no face before GREEN→BLUE (hysteresis)
- **Cooldown:** 2 seconds between same alert type
- **Speaking Protection:** 35 seconds consecutive non-RED prevents gesture interruption

**Audio Files:**
- Recognition alert: `Voicy_R2-D2 - 2.mp3` (~2 seconds)
- Loss alert: `Voicy_R2-D2 - 5.mp3` (~5 seconds)

### 4. Status LED Node (status_led_node)

**Purpose:** Visual feedback via GPIO LED control (supports white LED and RGB modes)

**White LED Mode (Current, Default):**
- **GPIO Pin:** GPIO 17 (Physical Pin 22 on 40-pin header)
- **Control:** Simple on/off
- **Hardware:** White LED panel (16 SMD LEDs, 3V, 20-50mA)
- **Wiring:**
  - Red wire → Pin 1 or 17 (3.3V)
  - Blue wire → Pin 22 (GPIO 17)
  - Black wire → Pin 6 (GND)

**LED Behavior (White Mode):**
| Status | GPIO 17 | Visual | Meaning |
|--------|---------|--------|---------|
| RED (recognized) | HIGH (ON) | 💡 LED ON | Person recognized |
| BLUE (lost) | LOW (OFF) | ⚫ LED OFF | Person lost/idle |
| GREEN (unknown) | LOW (OFF) | ⚫ LED OFF | Unknown person |

**RGB LED Mode (Legacy, Optional):**
- **GPIO Pins:** RED=17, GREEN=27, BLUE=22
- **Control:** Separate color control
- **Behavior:** RED=recognized, BLUE=lost, GREEN=unknown

**For detailed wiring instructions, see:** [`HARDWARE_WHITE_LED_WIRING.md`](HARDWARE_WHITE_LED_WIRING.md)

---

## ROS 2 Integration

### Published Topics

**Perception Topics:**
- `/oak/rgb/image_raw` (sensor_msgs/Image, 30 Hz) - Raw camera frames
- `/r2d2/perception/brightness` (std_msgs/Float32, 13 Hz) - Mean brightness
- `/r2d2/perception/face_count` (std_msgs/Int32, 13 Hz) - Number of faces
- `/r2d2/perception/person_id` (std_msgs/String, 6.5 Hz*) - Person name
- `/r2d2/perception/face_confidence` (std_msgs/Float32, 6.5 Hz*) - Confidence score
- `/r2d2/perception/is_target_person` (std_msgs/Bool, 6.5 Hz*) - Boolean convenience

**Audio & Status Topics:**
- `/r2d2/audio/person_status` (std_msgs/String, JSON, 10 Hz) - Status (RED/BLUE/GREEN)
- `/r2d2/audio/notification_event` (std_msgs/String, Event) - Alert events

*Only published if `enable_face_recognition:=true`

### ROS 2 Packages

| Package | Purpose | Location |
|---------|---------|----------|
| `r2d2_camera` | OAK-D camera driver | `ros2_ws/src/r2d2_camera` |
| `r2d2_perception` | Image processing & face recognition | `ros2_ws/src/r2d2_perception` |
| `r2d2_audio` | Audio notifications & LED control | `ros2_ws/src/r2d2_audio` |
| `r2d2_bringup` | Launch files & configuration | `ros2_ws/src/r2d2_bringup` |

### Launch Parameters

**Face Recognition Parameters:**
| Parameter | Default | Type | Purpose |
|-----------|---------|------|---------|
| `enable_face_recognition` | `false` | bool | Enable/disable LBPH recognition |
| `face_recognition_model_path` | `~/dev/r2d2/data/face_recognition/models/severin_lbph.xml` | string | Path to trained model |
| `recognition_confidence_threshold` | `70.0` | float | Threshold (lower=stricter) |
| `recognition_frame_skip` | `2` | int | Process every Nth frame |

**Audio Notification Parameters:**
| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `target_person` | String | `target_person` | any name | Person to recognize |
| `audio_volume` | Float | `0.05` | 0.0-1.0 | Global volume control |
| `jitter_tolerance_seconds` | Float | `5.0` | 1.0-10.0 | Brief gap tolerance |
| `loss_confirmation_seconds` | Float | `15.0` | 5.0-30.0 | Confirmation window |
| `cooldown_seconds` | Float | `2.0` | 1.0-5.0 | Min between same alert |
| `recognition_cooldown_after_loss_seconds` | Float | `5.0` | 3.0-10.0 | Quiet period after loss |
| `recognition_audio_file` | String | `Voicy_R2-D2 - 2.mp3` | filename | Audio for "Hello!" |
| `loss_audio_file` | String | `Voicy_R2-D2 - 5.mp3` | filename | Audio for "Lost you!" |

**Status LED Parameters:**
| Parameter | Type | Default | Options | Description |
|-----------|------|---------|---------|-------------|
| `led_mode` | String | `white` | `white`, `rgb` | LED control mode |
| `led_pin_white` | Int | `17` | GPIO pin | White LED GPIO (Pin 22) |
| `led_pin_red` | Int | `17` | GPIO pin | RGB red (legacy) |
| `led_pin_green` | Int | `27` | GPIO pin | RGB green (legacy) |
| `led_pin_blue` | Int | `22` | GPIO pin | RGB blue (legacy) |
| `simulate_gpio` | Bool | `false` | true/false | Simulation mode |

---

## Hardware Configuration

### OAK-D Lite Camera

**Specifications:**
- **Product:** OAK-D Lite Auto Focus
- **RGB Sensor:** 1920×1080 @ 30 FPS
- **Depth Sensor:** Stereo depth (OV9782 stereo pair)
- **Interface:** USB 3.0 with USB-C connector
- **Power:** 500mA @ 5V (bus-powered)
- **Processor:** Intel Movidius MyriadX
- **Connection:** Direct USB to Jetson (not through hub)

**Detection Verification:**
```bash
lsusb | grep Movidius
# Should show: 03e7:2485 Intel Movidius MyriadX
```

### PAM8403 Speaker Amplifier

**Hardware Architecture:**

The Jetson AGX Orin has two audio subsystems:
1. **HDA (High Definition Audio)** - Card 0 - HDMI outputs only (no analog)
2. **APE (Audio Processing Engine)** - Card 1 - I2S interface (direct analog output)

**Current Configuration:** Using APE Card 1 with I2S interface to drive HPO_L (left channel) pin on J511 header via PAM8403 amplifier.

**Wiring:**
```
Jetson 40-pin header (J30):
  Pin 2  (5V)   → PAM8403 +5V (power supply)
  Pin 6  (GND)  → PAM8403 GND (power ground)

Jetson audio panel header (J511):
  Pin 9  (HPO_L, left analog audio out) → PAM8403 LIN (left audio input)
  Pin 2  (AGND, audio ground)          → PAM8403 GND (audio signal ground)

PAM8403 speaker output (class-D amplifier):
  L+ → speaker + (wired correctly)
  L− → speaker − (wired correctly)
```

**Amplifier Specs:**
- Input: 3.3V max (Jetson HPO_L output)
- Gain: 23dB fixed (configurable via PAM8403 GAIN pin if needed)
- Output: 3W @ 8Ω (adequate for alert tones)
- Supply voltage: 5V from Jetson Pin 2

**ALSA Configuration:**
- Audio Card: Card 1 (APE) - NVIDIA Jetson Audio Processing Engine
- Audio Device: Device 0 - tegra-dlink-0 XBAR-ADMAIF1-0 (I2S)
- Sample Rates: 16, 44.1, 48 kHz
- Channels: 2 (stereo), using left channel only (HPO_L pin)
- Bit Depth: 16-bit PCM

**ALSA Config File:** `/etc/asound.conf`
```
pcm.!default {
    type asym
    playback.pcm "speaker_out"
    capture.pcm "speaker_in"
}

pcm.speaker_out {
    type dmix
    ipc_key 1234
    slave {
        pcm "hw:1,0"           # Card 1 (APE), Device 0 (I2S)
        rate 44100
        channels 2
        period_size 4096
        buffer_size 65536
    }
    bindings {
        0 0                    # Map dmix channel 0 to left speaker
        1 1
    }
}
```

### Status LED Hardware

**White LED Panel (Current, Default):**
- **Type:** Non-addressable white LED array (16 SMD LEDs)
- **Voltage:** 3V DC
- **Current:** 20-50mA total
- **GPIO Control:** GPIO 17 (Physical Pin 22 on 40-pin header)
- **Wiring:** See [`HARDWARE_WHITE_LED_WIRING.md`](HARDWARE_WHITE_LED_WIRING.md)

**RGB LED (Legacy, Optional):**
- **GPIO Pins:** RED=17, GREEN=27, BLUE=22
- **Voltage:** 3.3V logic levels (Jetson GPIO)
- **Control Mode:** Set `led_mode:=rgb` in launch parameters

---

## State Machine

### Three-State Recognition Model

**🔴 RED - Target Person Recognized (Active Engagement)**
- **Conditions:** Target person is currently visible
- **Status:** `{"status": "red", "person_identity": "target_person", ...}`
- **LED:** Solid RED (GPIO pin 17)
- **Audio:** "Hello!" played on transition (no repeated beeps)
- **Next State:** → BLUE after 5s jitter + 15s confirmation

**🔵 BLUE - No Person Recognized (Idle/Waiting)**
- **Conditions:** No target person visible, confirmed loss
- **Status:** `{"status": "blue", "person_identity": "no_person", ...}`
- **LED:** Solid BLUE (GPIO pin 22)
- **Audio:** "Oh, I lost you!" played on transition
- **Next State:** → RED when target person detected

**🟢 GREEN - Unknown Person Detected (Caution)**
- **Conditions:** Face detected but not the target person
- **Status:** `{"status": "green", "person_identity": "other_person", ...}`
- **LED:** Solid GREEN (GPIO pin 27)
- **Audio:** NO beeps (silent detection)
- **Next State:** → RED if target appears (target takes priority)

### State Machine Diagram

```
                              INITIAL STATE
                              (BLUE: idle)
                                   │
                    ┌──────────────┬┴──────────────┐
                    │              │               │
              target person   "unknown"      other person
              detected        or other        detected
                    │         person           │
                    │         detected         │
                    ▼              │            ▼
              🔊 Play "Hello!"     │        🟢 GREEN STATE
              🔴 RED STATE         ▼        (no beep, silent)
              (active)         🟢 GREEN     
                    │          STATE        
                    │            │          
         ┌──────────┴────────┐   │
         │                   │   │
    RED continues       target person
    (visible or        appears
     jitter < 5s)         │
         │                │
         │                ▼
         │           🔴 RED STATE
         │                │
         │    ┌───────────┴────────────┐
         │    │                        │
         │    │ target person     Loss confirmed:
         │    │ leaves            5s jitter +
         │    │ + stays           15s confirmation
         │    │ away 20s           │
         │    │ total              ▼
         │    │                🔊 Play "Lost you!"
         │    │                🔵 BLUE STATE
         │    │                (idle, waiting)
         │    │                   │
         │    └───────┬───────────┘
         │            │
         └─────────────────→ BLUE STATE (idle, waiting)
```

### Timing Configuration

**Key Timing Parameters:**

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `jitter_tolerance_seconds` | `5.0` | Brief interruption tolerance (ignores gaps < 5s) |
| `loss_confirmation_seconds` | `15.0` | Confirmation window AFTER jitter (total ~20s to loss alert) |
| `cooldown_seconds` | `2.0` | Min time between same alert type |
| `recognition_cooldown_after_loss_seconds` | `5.0` | Quiet period after loss alert |

**Total Time to Loss Alert:** ~20 seconds (5s jitter + 15s confirmation)

### Status Message Format

**Published Topic:** `/r2d2/audio/person_status` (std_msgs/String, JSON, 10 Hz)

**Message Structure:**
```json
{
  "status": "red|blue|green",
  "person_identity": "target_person|no_person|other_name",
  "timestamp_sec": 1765212914,
  "timestamp_nanosec": 949382424,
  "confidence": 0.95,
  "duration_seconds": 15.3,
  "is_loss_state": false,
  "audio_event": "recognition|loss|none"
}
```

---

## Performance Metrics

### System Performance Baselines

| Metric | Expected | Issue When |
|--------|----------|-----------|
| Camera FPS | 30 Hz | <25 Hz |
| Perception rate | 13 Hz | <10 Hz |
| Face recognition rate | 6.5 Hz | <5 Hz |
| Audio status rate | 10 Hz | <5 Hz |
| CPU usage (total) | 15-25% | >50% |
| Memory usage | ~500 MB | >2 GB |
| Recognition latency | <100 ms | >500 ms |
| Loss detection latency | ~20s | <10s or >30s |

### CPU Usage by Configuration

| Configuration | CPU | FPS | Use Case |
|---------------|-----|-----|----------|
| skip=1 | 18-25% | 13 Hz | Maximum accuracy, high CPU |
| skip=2 | 10-15% | 6.5 Hz | **Recommended** |
| skip=3 | 8-10% | 4.3 Hz | Minimal CPU, less responsive |
| skip=6 | 3-5% | 2.5 Hz | Very minimal, slow response |

### Latency Breakdown

**Total Recognition Cycle:** ~100-150ms

**Component Breakdown:**
1. **Camera capture:** ~33ms (30 FPS)
2. **Image downscaling:** ~5ms
3. **Face detection:** ~40ms (Haar Cascade)
4. **Face recognition:** ~20ms (LBPH)
5. **ROS publishing:** ~2ms

---

## Dependencies

### ROS 2 Packages
- `rclpy`: ROS 2 Python client library
- `sensor_msgs`: Standard sensor message types
- `std_msgs`: Standard message types
- `std_srvs`: Standard service types
- `cv_bridge`: OpenCV ↔ ROS Image conversion

### Python Packages
- `depthai`: DepthAI SDK 2.31.0.0 (OAK-D camera)
- `opencv-python`: OpenCV for image processing
- `numpy`: Numerical arrays (image buffers)
- `scipy`: Signal processing
- `ffmpeg-python`: Audio playback via ffplay

### System Dependencies
- `python3.10`: Python runtime
- `libasound2-dev`: ALSA audio development
- `portaudio19-dev`: PortAudio development
- `build-essential`: C/C++ compiler toolchain
- `cmake`: Build system
- `git`: Version control

### Environment Variables
```bash
# Critical for ARM/Jetson systems
export OPENBLAS_CORETYPE=ARMV8  # Prevents "illegal instruction" errors
```

---

## File Locations

### Core System

**Training Data:**
- Images: `~/dev/r2d2/data/face_recognition/severin/`
- Model: `~/dev/r2d2/data/face_recognition/models/severin_lbph.xml`

**Audio Files:**
- Source: `~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/`
- Installed: `~/dev/r2d2/ros2_ws/install/r2d2_audio/share/r2d2_audio/assets/audio/`

**Service Files:**
- Service: `/etc/systemd/system/r2d2-audio-notification.service`
- Startup script: `/home/severin/dev/r2d2/start_audio_service.sh`

**Auto-Start Services:**

**Systemd Services:**
- Camera-Perception: `/etc/systemd/system/r2d2-camera-perception.service`
- Audio Notification: `/etc/systemd/system/r2d2-audio-notification.service`
- Gesture Intent: `/etc/systemd/system/r2d2-gesture-intent.service` (NEW)

**Service Control:**
```bash
# Status
sudo systemctl status r2d2-camera-perception.service
sudo systemctl status r2d2-gesture-intent.service

# Logs
sudo journalctl -u r2d2-camera-perception.service -f
sudo journalctl -u r2d2-gesture-intent.service -f
```

### ROS 2 Packages

**Workspace:** `~/dev/r2d2/ros2_ws/`

**Source Packages:**
- `~/dev/r2d2/ros2_ws/src/r2d2_camera/` - Camera driver
- `~/dev/r2d2/ros2_ws/src/r2d2_perception/` - Image processing
- `~/dev/r2d2/ros2_ws/src/r2d2_audio/` - Audio notifications
- `~/dev/r2d2/ros2_ws/src/r2d2_bringup/` - Launch files

**Install Location:** `~/dev/r2d2/ros2_ws/install/`

### Configuration Files

**ALSA Audio:** `/etc/asound.conf`
**Environment Variables:** `~/.bashrc`
**DepthAI Virtual Environment:** `~/depthai_env/`

---

## Troubleshooting Guide

### Issue: No Audio Heard

**Symptom:** Service running but no audio plays

**Diagnosis:**
```bash
# Check service is running
sudo systemctl status r2d2-audio-notification.service

# Check ALSA configuration
aplay -l | grep "card 1"

# Verify audio file exists
ls -l ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/

# Test ffplay directly
ffplay -nodisp -autoexit -af "volume=0.05" /path/to/audio.mp3
```

**Solutions:**
1. Increase volume: `ros2 param set /audio_notification_node audio_volume 0.5`
2. Verify ALSA routing: `aplay -l` should show audio device
3. Check speaker power: Look for power indicator on PAM8403 amplifier
4. Verify J511 I2S connector is fully seated
5. Check PAM8403 wiring (see Hardware Configuration section)

### Issue: LBPH Model Not Found

**Symptom:** Face recognition not working, model file missing

**Solution:**
```bash
# Run training script first
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
python3 1_capture_training_data.py
python3 2_train_recognizer.py

# Verify model exists
ls -la ~/dev/r2d2/data/face_recognition/models/severin_lbph.xml
```

### Issue: Always Returning "unknown"

**Symptom:** Recognition always shows "unknown" even for trained person

**Solutions:**
1. Retrain with more diverse images (different angles, lighting)
2. Lower the threshold to accept weaker matches:
   ```bash
   ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
     enable_face_recognition:=true \
     recognition_confidence_threshold:=80.0
   ```
3. Add more training images:
   ```bash
   cd ~/dev/r2d2/tests/face_recognition
   python3 train_manager.py  # Select option 2
   ```

### Issue: Loss Alert Fires Too Early/Too Late

**Symptom:** "Oh, I lost you!" plays at wrong time

**Diagnosis:**
```bash
# Check parameters
ros2 param get /audio_notification_node jitter_tolerance_seconds
ros2 param get /audio_notification_node loss_confirmation_seconds
```

**Expected Timing:** ~20 seconds total (5s jitter + 15s confirmation)

**Solutions:**
1. Adjust jitter tolerance: `ros2 param set /audio_notification_node jitter_tolerance_seconds 3.0`
2. Adjust confirmation window: `ros2 param set /audio_notification_node loss_confirmation_seconds 10.0`
3. Restart service: `sudo systemctl restart r2d2-audio-notification.service`

### Issue: Service Crashes on Startup

**Symptom:** Service fails with `NameError` or `ModuleNotFoundError`

**Diagnosis:**
```bash
# Check service logs
journalctl -u r2d2-audio-notification.service -n 50
```

**Solutions:**
1. Verify environment sourcing in startup script
2. Ensure Python packages installed: `pip list | grep rclpy`
3. Rebuild package: `colcon build --packages-select r2d2_audio`
4. Restart service: `sudo systemctl restart r2d2-audio-notification.service`

### Issue: CPU Usage Too High

**Symptom:** System using 20%+ CPU

**Solutions:**
1. Increase frame skip for recognition:
   ```bash
   ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
     enable_face_recognition:=true \
     recognition_frame_skip:=3
   ```
2. Check for other processes: `top -bn1 | head -20`

### Issue: Camera Not Detected

**Symptom:** `lsusb` doesn't show OAK-D camera

**Solutions:**
1. Check USB connection (use USB 3.0 port directly, not through hub)
2. Verify power: `dmesg | tail -20` for USB errors
3. Test with: `python3 -c "import depthai as dai; print(len(dai.XLinkConnection.getAllConnectedDevices()))"`
4. Ensure OPENBLAS_CORETYPE=ARMV8 is set: `echo $OPENBLAS_CORETYPE`

### Issue: "Illegal Instruction" Error

**Symptom:** Python crashes with illegal instruction on ARM

**Solution:**
```bash
# Add to ~/.bashrc
echo "export OPENBLAS_CORETYPE=ARMV8" >> ~/.bashrc
source ~/.bashrc
```

### Issue: ALSA Device Busy

**Symptom:** "Device or resource busy" when playing audio

**Solutions:**
1. Check what's using the device: `fuser /dev/snd/pcm*`
2. Kill conflicting processes
3. Restart ALSA: `sudo alsa force-reload`

---

## References

### Documentation
- [100_PERSON_RECOGNITION_REFERENCE.md](100_PERSON_RECOGNITION_REFERENCE.md) - This document
- [101_PERSON_RECOGNITION_INSTALLATION.md](101_PERSON_RECOGNITION_INSTALLATION.md) - Installation guide
- [102_PERSON_RECOGNITION_QUICK_START.md](102_PERSON_RECOGNITION_QUICK_START.md) - Quick reference
- [001_ARCHITECTURE_OVERVIEW.md](001_ARCHITECTURE_OVERVIEW.md) - Complete R2D2 system architecture

### External Resources
- [OpenCV Documentation](https://docs.opencv.org/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Luxonis OAK-D Documentation](https://docs.luxonis.com/)
- [DepthAI Python SDK](https://docs.luxonis.com/software-v3/depthai/)
- [LBPH Face Recognition](https://docs.opencv.org/4.x/df/d25/classcv_1_1face_1_1LBPHFaceRecognizer.html)

### Hardware References
- **OAK-D Lite:** Luxonis camera with Intel Movidius MyriadX
- **PAM8403:** Class-D audio amplifier datasheet
- **Jetson AGX Orin:** NVIDIA Developer Kit user guide

---

**Document Version:** 1.0  
**Last Updated:** December 17, 2025  
**Status:** Complete and operational  
**Hardware:** OAK-D Lite + PAM8403 Speaker + Optional RGB LED  
**Platform:** NVIDIA Jetson AGX Orin 64GB with ROS 2 Humble


