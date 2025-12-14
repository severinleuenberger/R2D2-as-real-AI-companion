# Person Recognition and Status System - Complete Setup Guide

**Date:** December 9, 2025  
**Version:** 1.0 - Comprehensive Merged Documentation  
**Status:** ✅ Production Ready  
**Platform:** NVIDIA Jetson AGX Orin 64GB with ROS 2 Humble  
**Author:** R2D2 Development Team

---

## Executive Summary

This document provides a **complete, step-by-step guide** for setting up and operating R2D2's person recognition and status system. The system enables real-time face recognition, audio alerts, visual feedback (LED), and state management for person tracking.

**What This System Does:**

```
OAK-D Camera → Perception Pipeline → Face Recognition → Status Machine → Audio/LED Output
     ↓              ↓                      ↓                  ↓                ↓
  30 FPS RGB    Image Processing      Person ID         RED/BLUE/GREEN    🔊 Beeps + 🔴🟢🔵 LEDs
```

**Complete System Flow:**
1. **Camera captures** RGB frames (1920×1080 @ 30 FPS)
2. **Perception pipeline** processes frames (downscales, detects faces)
3. **Face recognition** identifies specific person using LBPH model
4. **Status machine** tracks state (RED=recognized, BLUE=lost, GREEN=unknown)
5. **Audio alerts** play MP3 sounds on state transitions
6. **LED feedback** shows visual status via GPIO RGB LED

**Key Capabilities:**
- ✅ Real-time person recognition at 6-13 Hz
- ✅ Smart state management with jitter tolerance and loss confirmation
- ✅ Audio alerts: "Hello!" on recognition, "Oh, I lost you!" on loss
- ✅ Visual feedback: RGB LED shows system state
- ✅ Production-ready: Systemd services, error handling, logging
- ✅ Training pipeline: Complete workflow for model creation

**Prerequisites (Hardware Setup):**
Before starting, ensure hardware is configured:
- **Camera Setup:** See [`102_CAMERA_SETUP_DOCUMENTATION.md`](102_CAMERA_SETUP_DOCUMENTATION.md) for OAK-D Lite camera installation
- **Audio Setup:** See [`101_SPEAKER_AUDIO_SETUP_DOCUMENTATION.md`](101_SPEAKER_AUDIO_SETUP_DOCUMENTATION.md) for speaker and ALSA configuration

---

## Table of Contents

1. [Prerequisites & Hardware Setup](#1-prerequisites--hardware-setup)
2. [Step 1: Perception Pipeline Setup](#2-step-1-perception-pipeline-setup)
3. [Step 2: Face Recognition Training](#3-step-2-face-recognition-training)
4. [Step 3: ROS 2 Face Recognition Integration](#4-step-3-ros-2-face-recognition-integration)
5. [Step 4: Audio Notification System](#5-step-4-audio-notification-system)
6. [Step 5: Status System & State Machine](#6-step-5-status-system--state-machine)
7. [Step 6: Production Deployment](#7-step-6-production-deployment)
8. [Configuration & Tuning](#8-configuration--tuning)
9. [Testing & Validation](#9-testing--validation)
10. [Troubleshooting](#10-troubleshooting)
11. [Quick Reference](#11-quick-reference)

---

## 1. Prerequisites & Hardware Setup

### 1.1 Required Hardware

| Component | Status | Documentation |
|-----------|--------|---------------|
| **OAK-D Lite Camera** | ✅ Required | [`102_CAMERA_SETUP_DOCUMENTATION.md`](102_CAMERA_SETUP_DOCUMENTATION.md) |
| **Speaker + PAM8403 Amplifier** | ✅ Required | [`101_SPEAKER_AUDIO_SETUP_DOCUMENTATION.md`](101_SPEAKER_AUDIO_SETUP_DOCUMENTATION.md) |
| **RGB LED (GPIO)** | ⚠️ Optional | GPIO pins 17, 27, 22 |

### 1.2 Software Prerequisites

**System Requirements:**
- Ubuntu 22.04 (Jammy) on NVIDIA Jetson AGX Orin
- ROS 2 Humble installed and configured
- Python 3.10.6
- DepthAI SDK 2.31.0.0 (in virtual environment)

**Environment Variables (Critical):**
```bash
# Add to ~/.bashrc
export OPENBLAS_CORETYPE=ARMV8  # Prevents "illegal instruction" errors on ARM64
```

**ROS 2 Workspace:**
```bash
# Workspace location
~/dev/r2d2/ros2_ws

# Build all packages
cd ~/dev/r2d2/ros2_ws
colcon build
source install/setup.bash
```

### 1.3 Verify Prerequisites

**Check Camera:**
```bash
# Verify camera is detected
lsusb | grep Movidius
# Should show: 03e7:2485 Intel Movidius MyriadX

# Test camera access
source ~/depthai_env/bin/activate
python3 -c "import depthai as dai; print('Camera OK')"
```

**Check Audio:**
```bash
# Verify ALSA device
aplay -l | grep "card 1"
# Should show: Card 1 (APE/I2S)

# Test audio playback
python3 ~/dev/r2d2/audio_beep.py
# Should hear a beep from speaker
```

**Check ROS 2:**
```bash
# Verify ROS 2 installation
ros2 --version
# Should show: ROS 2 Humble

# Check workspace
ros2 pkg list | grep r2d2
# Should show: r2d2_camera, r2d2_perception, r2d2_audio, r2d2_bringup
```

---

## 2. Step 1: Perception Pipeline Setup

### 2.1 Overview

The perception pipeline is the foundation that processes camera frames and provides face detection. It must be running before face recognition can work.

**What It Does:**
- Subscribes to `/oak/rgb/image_raw` (camera frames)
- Downscales images (1920×1080 → 640×360) for efficiency
- Detects faces using Haar Cascade
- Publishes face count and brightness metrics

### 2.2 Build and Install

**Step 1: Build Perception Package**
```bash
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_perception
source install/setup.bash
```

**Expected Output:**
```
Starting >>> r2d2_perception
Finished <<< r2d2_perception [1.79s]
Summary: 1 package finished [2.33s]
```

### 2.3 Launch Perception Pipeline

**Integrated Launch (Camera + Perception):**
```bash
cd ~/dev/r2d2/ros2_ws
source ~/depthai_env/bin/activate
source ~/.bashrc
source install/setup.bash

ros2 launch r2d2_bringup r2d2_camera_perception.launch.py
```

**What's Running:**
- `camera_node`: Publishes `/oak/rgb/image_raw` (30 Hz)
- `image_listener`: Processes frames, publishes:
  - `/r2d2/perception/brightness` (Float32, ~13 Hz)
  - `/r2d2/perception/face_count` (Int32, ~13 Hz)

**Verify It's Working:**
```bash
# In another terminal
ros2 topic echo /r2d2/perception/face_count
# Should show: data: 0, 1, 2, etc. (number of faces detected)

ros2 topic echo /r2d2/perception/brightness
# Should show: data: 112.5, 128.3, etc. (brightness values)
```

### 2.4 Performance Characteristics

| Metric | Value |
|--------|-------|
| **Frame Processing Rate** | 12-13 FPS (from 30 FPS camera) |
| **CPU Usage** | 8-10% (one core) |
| **Memory Usage** | ~200 MB |
| **Latency** | <5 ms per frame |

---

## 3. Step 2: Face Recognition Training

### 3.1 Overview

Before face recognition can work, you need a trained LBPH (Local Binary Pattern Histograms) model. This step captures training images and trains the recognizer.

**Training Requirements:**
- ~80+ diverse images of the target person
- Different lighting conditions, angles, distances
- Trained model saved as XML file

### 3.2 Capture Training Data

**Step 1: Navigate to Training Directory**
```bash
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8
```

**Step 2: Run Capture Script**
```bash
python3 1_capture_training_data.py
```

**What It Does:**
- Captures images from camera in 4 tasks:
  - Task 1: Bright light, 1 meter (20 seconds)
  - Task 2: Bright light, 2 meters (20 seconds)
  - Task 3: Low light, 3 meters (20 seconds)
  - Task 4: Low light, 5 meters (20 seconds)
- Saves images to `~/dev/r2d2/data/face_recognition/severin/`
- Total: ~80 images with diverse conditions

**Menu Options:**
```
1. Train new person
2. Add more pictures to existing person
3. Retrain model from existing images
4. Test accuracy at different distances
5. Real-time recognition test (30 sec, instant feedback)
6. List all people and models
7. Delete person (safe deletion)
```

**Tips for Better Recognition:**
- Vary lighting (bright, low light, natural, artificial)
- Capture from different angles (straight, 45°, 90°)
- Include different expressions (neutral, smile, looking down)
- Some images with partial occlusions (turned head, glasses)

### 3.3 Train the Model

**Option A: Using Training Manager (Recommended)**
```bash
python3 train_manager.py
# Select option 1 to train
```

**Option B: Direct Training**
```bash
python3 2_train_recognizer.py
```

**What It Does:**
- Reads all images from `~/dev/r2d2/data/face_recognition/severin/`
- Extracts faces using Haar Cascade
- Trains LBPH recognizer
- Saves model to `~/dev/r2d2/data/face_recognition/models/severin_lbph.xml`
- Duration: 30-60 seconds

**Expected Output:**
```
Training LBPH recognizer...
Processing 80 images...
Training complete!
Model saved to: ~/dev/r2d2/data/face_recognition/models/severin_lbph.xml
```

### 3.4 Test the Model

**Test Recognition Accuracy:**
```bash
python3 3_test_recognizer_demo.py
```

**What It Shows:**
- Recognition accuracy statistics
- Confidence scores for test images
- Suggestions for improving accuracy

**Verify Model Exists:**
```bash
ls -lh ~/dev/r2d2/data/face_recognition/models/severin_lbph.xml
# Should show: ~33 MB file
```

---

## 4. Step 3: ROS 2 Face Recognition Integration

### 4.1 Overview

Once the model is trained, enable face recognition in the ROS 2 perception pipeline. This publishes person identification on ROS topics.

**What It Adds:**
- Person identification: `/r2d2/perception/person_id` (String)
- Confidence scores: `/r2d2/perception/face_confidence` (Float32)
- Boolean convenience: `/r2d2/perception/is_target_person` (Bool)

### 4.2 Launch with Face Recognition

**Basic Launch (After Training):**
```bash
cd ~/dev/r2d2/ros2_ws
source ~/depthai_env/bin/activate
source ~/.bashrc
source install/setup.bash

ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true
```

**Requirements:**
- Trained model must exist at `~/dev/r2d2/data/face_recognition/models/severin_lbph.xml`
- If model not found, node logs warning and continues without recognition

### 4.3 Custom Configuration

**Launch with Custom Settings:**
```bash
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true \
  face_recognition_model_path:=/path/to/custom_model.xml \
  recognition_confidence_threshold:=75.0 \
  recognition_frame_skip:=3
```

**Launch Parameters:**

| Parameter | Default | Type | Purpose |
|-----------|---------|------|---------|
| `enable_face_recognition` | `false` | bool | Enable/disable LBPH recognition |
| `face_recognition_model_path` | `~/dev/r2d2/data/face_recognition/models/severin_lbph.xml` | string | Path to trained LBPH model |
| `recognition_confidence_threshold` | `70.0` | float | Threshold for target person classification (lower=stricter) |
| `recognition_frame_skip` | `2` | int | Process recognition every Nth frame (manages CPU load) |

### 4.4 Monitor Recognition

**Watch Person ID:**
```bash
ros2 topic echo /r2d2/perception/person_id
# Expected output:
# data: target_person
# ---
# data: unknown
# ---
# data: target_person
```

**Watch Confidence Scores:**
```bash
ros2 topic echo /r2d2/perception/face_confidence
# Expected output (lower is better):
# data: 35.2  # High confidence target person
# ---
# data: 92.1  # Low confidence, likely unknown
```

**Check Publishing Rate:**
```bash
ros2 topic hz /r2d2/perception/person_id
# Expected (with frame_skip=2):
# average rate: 6.50
# min: 0.150s max: 0.160s std dev: 0.0050s count: 13
```

### 4.5 CPU & Performance

**CPU Usage by Configuration:**

| Configuration | CPU | FPS | Use Case |
|---------------|-----|-----|----------|
| skip=1 | 18-25% | 13 Hz | Maximum accuracy, high CPU |
| skip=2 | 10-15% | 6.5 Hz | **Recommended** |
| skip=3 | 8-10% | 4.3 Hz | Minimal CPU, less responsive |
| skip=6 | 3-5% | 2.5 Hz | Very minimal, slow response |

**Recommended Configuration:**
```bash
# Balanced: 10-12% CPU, good recognition frequency
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true \
  recognition_frame_skip:=2
```

---

## 5. Step 4: Audio Notification System

### 5.1 Overview

The audio notification system provides real-time audio alerts when person recognition state changes. It implements a sophisticated state machine with jitter tolerance and loss confirmation.

**What It Does:**
- Subscribes to `/r2d2/perception/person_id`
- Tracks recognition state (RED/BLUE/GREEN)
- Plays MP3 audio alerts on state transitions
- Publishes status JSON for LED and other consumers

### 5.2 Build Audio Package

**Build r2d2_audio Package:**
```bash
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_audio
source install/setup.bash
```

### 5.3 Launch Audio Notifications

**Option A: Manual Launch (Development/Testing)**
```bash
cd ~/dev/r2d2/ros2_ws
source install/setup.bash
ros2 launch r2d2_audio audio_notification.launch.py
```

**Option B: Background Service (Production)**
```bash
# One-time setup
sudo cp /home/severin/dev/r2d2/r2d2-audio-notification.service /etc/systemd/system/
chmod +x /home/severin/dev/r2d2/start_audio_service.sh
sudo systemctl daemon-reload
sudo systemctl enable r2d2-audio-notification.service
sudo systemctl start r2d2-audio-notification.service
```

**Service Management:**
```bash
# Check status
sudo systemctl status r2d2-audio-notification.service

# View logs
sudo journalctl -u r2d2-audio-notification.service -f

# Restart service
sudo systemctl restart r2d2-audio-notification.service
```

### 5.4 Audio Behavior

**What You'll Hear:**
- 🔊 **"Hello!"** MP3 when your face enters the frame (unknown → recognized)
- ⏸ **Silent** while continuously recognized (no repeated beeps)
- 🔔 **"Oh, I lost you!"** MP3 if you're absent for >20 seconds (5s jitter + 15s confirmation)
- 🔊 **"Hello!"** MP3 when you return (lost → recognized)

**State Transitions:**
```
UNKNOWN → RECOGNIZED: 🔊 "Hello!" plays
RECOGNIZED → RECOGNIZED (jitter < 5s): ⏸ Silent (no alert)
RECOGNIZED → LOST (>20s absence): 🔔 "Oh, I lost you!" plays
LOST → RECOGNIZED: 🔊 "Hello!" plays
```

### 5.5 Audio Files

**Installed Audio Files:**
- **Recognition Alert:** `Voicy_R2-D2 - 2.mp3` (~2 seconds, "Hello!")
- **Loss Alert:** `Voicy_R2-D2 - 5.mp3` (~5 seconds, "Oh, I lost you!")

**File Locations:**
- Source: `~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/`
- Installed: `~/dev/r2d2/ros2_ws/install/r2d2_audio/share/r2d2_audio/assets/audio/`

### 5.6 Volume Control

**Global Volume Parameter:**
The `audio_volume` parameter controls ALL audio output (0.0=silent, 1.0=max).

**Current Default:** `0.05` (5% - very quiet)

**Adjust Volume:**
```bash
# Runtime change (temporary)
ros2 param set /audio_notification_node audio_volume 0.3

# Launch with custom volume
ros2 launch r2d2_audio audio_notification.launch.py audio_volume:=0.4

# Permanent (edit service file)
sudo nano /etc/systemd/system/r2d2-audio-notification.service
# Add: audio_volume:=0.3 to ExecStart line
sudo systemctl daemon-reload
sudo systemctl restart r2d2-audio-notification.service
```

**Volume Levels:**
- `0.05` = 5% (current default, very quiet)
- `0.1` = 10% (quiet, library/office)
- `0.2` = 20% (moderate, home use)
- `0.5` = 50% (loud, normal room)
- `1.0` = 100% (maximum, emergency)

---

## 6. Step 5: Status System & State Machine

### 6.1 Overview

The status system is the core state machine that tracks person recognition and drives all outputs (audio, LED, logging). It implements a three-state model with sophisticated timing controls.

### 6.2 Three-State Recognition Model

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

### 6.3 State Machine Diagram

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

### 6.4 Timing Configuration

**Key Timing Parameters:**

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `jitter_tolerance_seconds` | `5.0` | Brief interruption tolerance (ignores gaps < 5s) |
| `loss_confirmation_seconds` | `15.0` | Confirmation window AFTER jitter (total ~20s to loss alert) |
| `cooldown_seconds` | `2.0` | Min time between same alert type |
| `recognition_cooldown_after_loss_seconds` | `5.0` | Quiet period after loss alert (prevents rapid beeping) |

**Total Time to Loss Alert:** ~20 seconds (5s jitter + 15s confirmation)

### 6.5 Status Message Format

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

**Example Messages:**

**RED State:**
```json
{
  "status": "red",
  "person_identity": "target_person",
  "confidence": 0.95,
  "duration_seconds": 12.5,
  "is_loss_state": false,
  "audio_event": "none"
}
```

**BLUE State:**
```json
{
  "status": "blue",
  "person_identity": "no_person",
  "confidence": 0.0,
  "duration_seconds": 8.3,
  "is_loss_state": true,
  "audio_event": "loss"
}
```

### 6.6 LED Visual Feedback

**RGB LED Node:** `status_led_node` (optional, requires GPIO)

**GPIO Pins:**
- RED: GPIO 17
- GREEN: GPIO 27
- BLUE: GPIO 22

**LED Behavior:**

| Status | Red | Green | Blue | Visual |
|--------|-----|-------|------|--------|
| RED (recognized) | ON | OFF | OFF | 🔴 Solid Red |
| BLUE (lost) | OFF | OFF | ON | 🔵 Solid Blue |
| GREEN (unknown) | OFF | ON | OFF | 🟢 Solid Green |

**Launch LED Node:**
```bash
ros2 run r2d2_audio status_led_node
# Or include in launch file
```

---

## 7. Step 6: Production Deployment

### 7.1 Systemd Service Setup

**Audio Notification Service:**

**Installation (One-time):**
```bash
sudo cp /home/severin/dev/r2d2/r2d2-audio-notification.service /etc/systemd/system/
chmod +x /home/severin/dev/r2d2/start_audio_service.sh
sudo systemctl daemon-reload
sudo systemctl enable r2d2-audio-notification.service
sudo systemctl start r2d2-audio-notification.service
```

**Service Management:**
```bash
# Check status
sudo systemctl status r2d2-audio-notification.service

# View logs
sudo journalctl -u r2d2-audio-notification.service -f

# Restart service
sudo systemctl restart r2d2-audio-notification.service

# Stop service
sudo systemctl stop r2d2-audio-notification.service
```

**Service Features:**
- ✅ Auto-start on Jetson boot
- ✅ Auto-restart on failure (with 5s delay)
- ✅ Automatic restart limit (max 3 consecutive failures)
- ✅ Full environment sourcing
- ✅ Proper logging to systemd journal

### 7.2 Complete System Startup

**Full R2D2 Startup Sequence:**

**Terminal 1: Camera + Perception + Recognition**
```bash
cd ~/dev/r2d2/ros2_ws
source ~/depthai_env/bin/activate
source ~/.bashrc
source install/setup.bash

ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true
```

**Terminal 2: Audio Notifications (if not using service)**
```bash
cd ~/dev/r2d2/ros2_ws
source install/setup.bash
ros2 launch r2d2_audio audio_notification.launch.py
```

**Terminal 3: LED Node (optional)**
```bash
cd ~/dev/r2d2/ros2_ws
source install/setup.bash
ros2 run r2d2_audio status_led_node
```

**Terminal 4: Monitoring**
```bash
# Watch person recognition
ros2 topic echo /r2d2/perception/person_id

# Watch status messages
ros2 topic echo /r2d2/audio/person_status --no-arr

# Check topic rates
ros2 topic hz /r2d2/perception/person_id
ros2 topic hz /r2d2/audio/person_status
```

### 7.3 Post-Reboot Verification

**After Rebooting Jetson:**
```bash
# 1. Wait 30 seconds for system to stabilize
sleep 30

# 2. Check service status
sudo systemctl status r2d2-audio-notification.service
# Should show: active (running)

# 3. Verify audio system
ros2 topic echo /r2d2/audio/person_status --no-arr -n 3
# Should show: {"status": "blue", ...}

# 4. Test audio playback
ros2 topic pub --once /r2d2/perception/person_id std_msgs/String "{data: target_person}"
# Should play "Hello!" beep

# 5. Check LED feedback (if available)
# Should see RED LED light up

# 6. Verify logs for errors
journalctl -u r2d2-audio-notification.service -n 20 | grep -i error
# Should show no errors
```

---

## 8. Configuration & Tuning

### 8.1 Audio Notification Parameters

**All Parameters:**

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `target_person` | String | `target_person` | any name | Person to recognize (should match training data) |
| `audio_volume` | Float | `0.05` | 0.0-1.0 | Global volume control (0-100%) |
| `jitter_tolerance_seconds` | Float | `5.0` | 1.0-10.0 | Brief gap tolerance |
| `loss_confirmation_seconds` | Float | `15.0` | 5.0-30.0 | Confirmation window |
| `cooldown_seconds` | Float | `2.0` | 1.0-5.0 | Min between same alert type |
| `recognition_cooldown_after_loss_seconds` | Float | `5.0` | 3.0-10.0 | Quiet period after loss alert |
| `recognition_audio_file` | String | `Voicy_R2-D2 - 2.mp3` | filename | Audio file for "Hello!" |
| `loss_audio_file` | String | `Voicy_R2-D2 - 5.mp3` | filename | Audio file for "Lost you!" |
| `enabled` | Boolean | `true` | true/false | Enable/disable system |

**View Current Parameters:**
```bash
ros2 param list /audio_notification_node
ros2 param get /audio_notification_node audio_volume
```

**Set Parameters (Runtime, Temporary):**
```bash
ros2 param set /audio_notification_node audio_volume 0.3
ros2 param set /audio_notification_node loss_confirmation_seconds 20.0
```

**Set Parameters (Permanent):**
Edit service file:
```bash
sudo nano /etc/systemd/system/r2d2-audio-notification.service
# Modify ExecStart line to add parameters:
ExecStart=/home/severin/dev/r2d2/start_audio_service.sh audio_volume:=0.3 loss_confirmation_seconds:=20.0
sudo systemctl daemon-reload
sudo systemctl restart r2d2-audio-notification.service
```

### 8.2 Face Recognition Parameters

**Perception Pipeline Parameters:**

| Parameter | Default | Type | Purpose |
|-----------|---------|------|---------|
| `enable_face_recognition` | `false` | bool | Enable/disable LBPH recognition |
| `face_recognition_model_path` | `~/dev/r2d2/data/face_recognition/models/severin_lbph.xml` | string | Path to trained model |
| `recognition_confidence_threshold` | `70.0` | float | Threshold (lower=stricter) |
| `recognition_frame_skip` | `2` | int | Process every Nth frame |

**Tuning Guidelines:**
- **CPU Optimization:** Increase `recognition_frame_skip` (e.g., 3 or 4) to reduce CPU
- **Accuracy vs Speed:** Lower `recognition_confidence_threshold` (e.g., 60.0) for more detections
- **Audio Responsiveness:** Lower `loss_confirmation_seconds` (e.g., 10.0) for faster alerts
- **False Alarm Prevention:** Increase `jitter_tolerance_seconds` (e.g., 7.0) for noisy environments

### 8.3 Configuration Examples

**Loud, Responsive Setup:**
```bash
ros2 launch r2d2_audio audio_notification.launch.py \
  audio_volume:=0.8 \
  loss_confirmation_seconds:=10.0 \
  jitter_tolerance_seconds:=3.0
```

**Quiet, Patient Setup:**
```bash
ros2 launch r2d2_audio audio_notification.launch.py \
  audio_volume:=0.2 \
  loss_confirmation_seconds:=20.0 \
  jitter_tolerance_seconds:=7.0
```

**Minimal CPU Usage:**
```bash
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true \
  recognition_frame_skip:=3
```

---

## 9. Monitoring & Testing

### 9.1 Monitoring System Overview

The R2D2 person recognition system includes comprehensive monitoring tools that allow you to observe system behavior in real-time. These tools help verify system operation, diagnose issues, and understand system performance.

**Available Monitoring Tools:**

1. **Live Stream Monitor** - Real-time monitoring of recognition events and status changes
2. **Full Pipeline Dashboard** - Complete system overview with rates, status, and health metrics
3. **Component Tests** - Individual tests for camera, perception, and audio systems
4. **Integration Test** - End-to-end system validation

### 9.2 Live Stream Monitoring

**Real-Time Person Recognition Monitor:**
```bash
cd ~/dev/r2d2
./tests/system/monitor_person_recognition.sh
```

**What It Shows:**
- 👤 **Person Recognition Stream**: Live updates when person is detected/recognized
- 📊 **Status Stream**: Real-time state changes (RED/BLUE/GREEN)
- 🔔 **Audio Event Stream**: Recognition and loss alerts as they occur
- 👥 **Face Detection Stream**: Number of faces detected in real-time

**Output Example:**
```
🔍 R2D2 Person Recognition System - Live Monitor
==================================================

📡 Checking system status...
  ✅ Camera node: RUNNING
  ✅ Perception node: RUNNING
  ✅ Audio notification node: RUNNING
     Target person: target_person

👤 Person Recognition Stream:
   [14:23:15] ✅ Person: target_person (RECOGNIZED)
   [14:23:18] 👤 Person: unknown

📊 Status Stream:
   [14:23:15] 🔴 RED | Target person recognized
   [14:23:20] 🔵 BLUE | No person (idle)

🔔 Audio Event Stream:
   [14:23:15] 🔊 🎉 Recognized target_person!
   [14:23:40] 🔔 ❌ target_person lost (confirmed after 25.3s absence)
```

### 9.3 Full Pipeline Dashboard

**Complete System Monitor:**
```bash
cd ~/dev/r2d2
./tests/system/monitor_full_pipeline.sh
```

**What It Shows:**
- 🔧 **System Status**: All nodes (camera, perception, audio, LED)
- 📈 **Topic Rates**: Publication rates for all topics (Hz)
- 📊 **Current Values**: Face count, person ID, brightness
- 🎯 **Current Status**: Detailed status JSON with confidence and duration
- 🔔 **Recent Events**: Last 3 notification events

**Dashboard Updates:** Every 2 seconds (auto-refresh)

**Output Example:**
```
📊 R2D2 System Monitor - 14:25:30
==========================================

🔧 System Status:
  Camera Node:        ✅ RUNNING
  Perception Node:    ✅ RUNNING
  Audio Node:         ✅ RUNNING
  LED Node:           ✅ RUNNING

📈 Topic Rates (Hz):
  Camera (RGB):         30.0 Hz (expected: 30.0)
  Face Detection:       13.2 Hz (expected: 13.0)
  Person ID:             6.5 Hz (expected: 6.5)
  Status:               10.1 Hz (expected: 10.0)

📊 Current Values:
  Faces Detected:      1
  Person ID:           target_person
  Brightness:          125.3

🎯 Current Status:
  🔴 RED - Target person recognized
  Person:              target_person
  Confidence:          0.95
  Duration:            12.5s
```

### 9.4 Component Testing

**Test Camera & Perception Pipeline:**
```bash
cd ~/dev/r2d2
./tests/system/test_camera_pipeline.sh
```

**Tests:**
- Camera node status
- Camera topic publication rate (~30 Hz)
- Perception node status
- Perception topic rates (~13 Hz)
- Face detection functionality

**Test Audio Notification System:**
```bash
cd ~/dev/r2d2
./tests/system/test_audio_system.sh
```

**Tests:**
- Audio node status and parameters
- Status topic publication rate (~10 Hz)
- Notification event handling
- Recognition simulation and response

**Test Complete System Integration:**
```bash
cd ~/dev/r2d2
./tests/system/test_complete_system.sh
```

**Tests:**
- All system components
- Topic availability
- Data flow rates
- End-to-end recognition pipeline
- Status state transitions

### 9.5 Manual Monitoring Commands

**Quick Status Check:**
```bash
# Check all nodes
ros2 node list

# Check all topics
ros2 topic list

# Monitor person recognition
ros2 topic echo /r2d2/perception/person_id

# Monitor status messages
ros2 topic echo /r2d2/audio/person_status --no-arr

# Check topic rates
ros2 topic hz /r2d2/perception/person_id
ros2 topic hz /r2d2/audio/person_status

# View node parameters
ros2 param list /audio_notification_node
ros2 param get /audio_notification_node target_person
```

**Continuous Monitoring:**
```bash
# Watch person ID with timestamps
watch -n 0.5 'ros2 topic echo /r2d2/perception/person_id --once | grep data'

# Monitor status JSON
watch -n 0.5 'ros2 topic echo /r2d2/audio/person_status --once --no-arr'

# Check system health
watch -n 1 'ros2 node list && echo "---" && ros2 topic list | grep r2d2'
```

### 9.6 Test 1: Recognition Alert

**Setup:**
```bash
# Terminal 1: Monitor status
ros2 topic echo /r2d2/audio/person_status --no-arr

# Terminal 2: Simulate recognition
ros2 topic pub --once /r2d2/perception/person_id std_msgs/String "{data: target_person}"
```

**Expected:**
- 🔊 "Hello!" plays
- Status changes to: `{"status": "red", "person_identity": "target_person", ...}`
- LED shows RED (if enabled)

### 9.7 Test 2: Loss Detection

**Setup:**
```bash
# Terminal 1: Monitor status
ros2 topic echo /r2d2/audio/person_status --no-arr

# Terminal 2: Send recognition, then wait 20+ seconds
ros2 topic pub --once /r2d2/perception/person_id std_msgs/String "{data: target_person}"
# Wait 20 seconds...
```

**Expected:**
- 🔊 "Hello!" plays immediately
- After ~20 seconds: 🔔 "Oh, I lost you!" plays
- Status changes to: `{"status": "blue", "person_identity": "no_person", ...}`
- LED shows BLUE (if enabled)

### 9.8 Test 3: Jitter Tolerance

**Setup:**
```bash
# Terminal 1: Monitor status
ros2 topic echo /r2d2/audio/person_status --no-arr

# Terminal 2: Send recognition
ros2 topic pub --once /r2d2/perception/person_id std_msgs/String "{data: target_person}"
# Wait 2 seconds for beep...

# Terminal 3: Send "unknown" (simulating interruption)
ros2 topic pub --once /r2d2/perception/person_id std_msgs/String "{data: unknown}"
# Wait 3 seconds...

# Terminal 2: Send "severin" again
ros2 topic pub --once /r2d2/perception/person_id std_msgs/String "{data: target_person}"
```

**Expected:**
- NO second beep (jitter ignored)
- Status stays RED throughout
- LED stays RED (no flicker)

### 9.9 Validation Checklist

Before declaring the system "working":

- [ ] Service Status: `sudo systemctl status r2d2-audio-notification.service` → `active (running)`
- [ ] Topics: `ros2 topic list | grep r2d2/audio` → Both topics visible
- [ ] Recognition: Publish person_id → "Hello!" plays + status = RED
- [ ] Loss: Wait 20s → "Oh, I lost you!" plays + status = BLUE
- [ ] Jitter: Brief interruption ignored + no extra beeps
- [ ] LED: Status changes visible on RGB LED (if GPIO available)
- [ ] Logs: `journalctl -u r2d2-audio-notification.service -n 10` → No errors
- [ ] Parameters: All parameters readable via `ros2 param list`
- [ ] Post-Reboot: Service auto-starts and works after full system reboot

---

### 9.10 Monitoring Best Practices

**When to Use Each Tool:**

1. **During Development:**
   - Use `monitor_person_recognition.sh` for real-time debugging
   - Use `test_complete_system.sh` after making changes

2. **During Operation:**
   - Use `monitor_full_pipeline.sh` for system health overview
   - Check topic rates to verify performance

3. **Troubleshooting:**
   - Use component tests (`test_camera_pipeline.sh`, `test_audio_system.sh`) to isolate issues
   - Use live monitors to observe behavior patterns

4. **Performance Monitoring:**
   - Monitor topic rates to ensure system is operating at expected frequencies
   - Check CPU usage: `top -bn1 | grep python`
   - Monitor memory: `free -h`

**Expected Monitoring Values:**

| Metric | Expected Value | Warning Threshold |
|--------|---------------|-------------------|
| Camera FPS | 30 Hz | < 25 Hz |
| Face Detection Rate | 13 Hz | < 10 Hz |
| Person ID Rate | 6.5 Hz | < 5 Hz |
| Status Rate | 10 Hz | < 5 Hz |
| Recognition Latency | < 100 ms | > 500 ms |

**Interpreting Status Colors:**
- 🔴 **RED**: Target person recognized, system actively engaged
- 🔵 **BLUE**: No person detected, system idle/waiting
- 🟢 **GREEN**: Unknown person detected, system in caution mode

---

## 10. Troubleshooting

### 10.1 Issue: No Audio Heard

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

### 10.2 Issue: LBPH Model Not Found

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

### 10.3 Issue: Always Returning "unknown"

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

### 10.4 Issue: Loss Alert Fires Too Early/Too Late

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

### 10.5 Issue: Service Crashes on Startup

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

### 10.6 Issue: CPU Usage Too High

**Symptom:** System using 20%+ CPU

**Solutions:**
1. Increase frame skip for recognition:
   ```bash
   ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
     enable_face_recognition:=true \
     recognition_frame_skip:=3
   ```
2. Check for other processes: `top -bn1 | head -20`

---

## 11. Quick Reference

### 11.1 Common Commands

**Start/Stop Services:**
```bash
# Audio notification service
sudo systemctl start r2d2-audio-notification.service
sudo systemctl stop r2d2-audio-notification.service
sudo systemctl restart r2d2-audio-notification.service

# Check status
sudo systemctl status r2d2-audio-notification.service
```

**Launch Systems:**
```bash
# Camera + Perception + Recognition
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py enable_face_recognition:=true

# Audio notifications
ros2 launch r2d2_audio audio_notification.launch.py

# LED node
ros2 run r2d2_audio status_led_node
```

**Monitor Topics:**
```bash
# Person recognition
ros2 topic echo /r2d2/perception/person_id

# Status messages
ros2 topic echo /r2d2/audio/person_status --no-arr

# Notification events
ros2 topic echo /r2d2/audio/notification_event

# Check rates
ros2 topic hz /r2d2/perception/person_id
ros2 topic hz /r2d2/audio/person_status
```

**View Logs:**
```bash
# Audio service logs
sudo journalctl -u r2d2-audio-notification.service -f
sudo journalctl -u r2d2-audio-notification.service -n 50

# Filter for errors
journalctl -u r2d2-audio-notification.service -n 50 | grep ERROR
```

**Adjust Parameters:**
```bash
# Increase volume
ros2 param set /audio_notification_node audio_volume 0.5

# Faster loss detection
ros2 param set /audio_notification_node loss_confirmation_seconds 10.0

# View all parameters
ros2 param list /audio_notification_node
```

### 11.2 Training Commands

```bash
# Navigate to training directory
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8

# Capture training data
python3 1_capture_training_data.py

# Train model
python3 2_train_recognizer.py

# Test model
python3 3_test_recognizer_demo.py

# Training manager (menu)
python3 train_manager.py
```

### 11.3 File Locations

**Training Data:**
- Images: `~/dev/r2d2/data/face_recognition/severin/`
- Model: `~/dev/r2d2/data/face_recognition/models/severin_lbph.xml`

**Audio Files:**
- Source: `~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/`
- Installed: `~/dev/r2d2/ros2_ws/install/r2d2_audio/share/r2d2_audio/assets/audio/`

**Service Files:**
- Service: `/etc/systemd/system/r2d2-audio-notification.service`
- Startup script: `/home/severin/dev/r2d2/start_audio_service.sh`

**ROS 2 Packages:**
- Workspace: `~/dev/r2d2/ros2_ws/`
- Source: `~/dev/r2d2/ros2_ws/src/`
- Install: `~/dev/r2d2/ros2_ws/install/`

---

## 12. System Architecture Reference

### 12.1 Complete Data Flow

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
    ├─ Face detection (Haar Cascade) → /r2d2/perception/face_count (13 Hz)
    └─ Face recognition (LBPH, optional) → /r2d2/perception/person_id (6.5 Hz)
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

### 12.2 ROS 2 Topics Reference

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

### 12.3 Performance Baselines

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

---

## 13. Related Documentation

**Hardware Setup:**
- **Camera:** [`102_CAMERA_SETUP_DOCUMENTATION.md`](102_CAMERA_SETUP_DOCUMENTATION.md) - OAK-D Lite camera installation
- **Audio:** [`101_SPEAKER_AUDIO_SETUP_DOCUMENTATION.md`](101_SPEAKER_AUDIO_SETUP_DOCUMENTATION.md) - Speaker and ALSA configuration

**System Architecture:**
- **Overview:** [`001_ARCHITECTURE_OVERVIEW.md`](001_ARCHITECTURE_OVERVIEW.md) - Complete system architecture

**Other Documentation:**
- **Agent Instructions:** [`000_INTERNAL_AGENT_NOTES.md`](000_INTERNAL_AGENT_NOTES.md) - Development guidelines

---

## 14. Summary

This document provides a complete guide for setting up and operating R2D2's person recognition and status system. The system is **production-ready** and includes:

✅ **Perception Pipeline:** Real-time image processing and face detection  
✅ **Face Recognition:** LBPH-based person identification with training pipeline  
✅ **Audio Notifications:** Smart state machine with MP3 alerts  
✅ **Status System:** Three-state model (RED/BLUE/GREEN) with LED feedback  
✅ **Production Deployment:** Systemd services for auto-start and reliability  
✅ **Comprehensive Documentation:** Complete setup, configuration, and troubleshooting

**Quick Start Summary:**
1. Set up hardware (camera, audio) - see 101_ and 102_ documents
2. Build ROS 2 packages: `colcon build`
3. Train face recognition model: `python3 1_capture_training_data.py` → `python3 2_train_recognizer.py`
4. Launch perception with recognition: `ros2 launch r2d2_bringup r2d2_camera_perception.launch.py enable_face_recognition:=true`
5. Launch audio notifications: `ros2 launch r2d2_audio audio_notification.launch.py`
6. Monitor and tune as needed

**Ready to use!** Follow the steps in order, and refer to troubleshooting section if issues arise.

---

---

## 12. Web Dashboard Integration (December 14, 2025)

### 12.1 Service Management Enhancements

The web dashboard has been enhanced with improved service management capabilities:

**Camera Device Exclusivity:**
- The OAK-D camera can only be accessed by one service at a time
- `r2d2-camera-stream.service` and `r2d2-camera-perception.service` are mutually exclusive
- The service manager automatically:
  - Stops conflicting services before starting a new one
  - Waits 3 seconds for device release
  - Verifies service is actually stopped before starting another
- Prevents `X_LINK_DEVICE_ALREADY_IN_USE` errors

**Service Startup Verification:**
- After starting a service, the system verifies:
  - Service is actually running (`systemctl is-active`)
  - Expected topics are publishing:
    - `camera-perception`: `/oak/rgb/image_raw`
    - `audio-notification`: `/r2d2/audio/person_status`
- Returns detailed error messages if service starts but topics don't publish

**rosbridge Detection:**
- New API endpoint: `GET /api/status/rosbridge`
- Detects rosbridge availability (does NOT auto-start rosbridge)
- Checks:
  - rosbridge_websocket process running
  - Port 9090 accessibility
- UI displays prominent error banner when rosbridge is not running
- Shows manual start instructions: `cd ~/dev/r2d2/web_dashboard && ./start_rosbridge.sh`

### 12.2 Verification Script

A comprehensive verification script is available:

```bash
~/dev/r2d2/web_dashboard/verify_integration.sh
```

**What it checks:**
- Systemd service files existence
- Service status (active/failed/inactive)
- rosbridge process and port availability
- ROS 2 topics publishing
- Web dashboard API endpoints
- UI connectivity (when rosbridge running)

**Usage:**
```bash
cd ~/dev/r2d2/web_dashboard
./verify_integration.sh
```

The script provides colored output and a summary with common fixes for any failures.

### 12.3 UI Error Messages

The web dashboard UI now provides enhanced error messages:

**Error Priority:**
1. **rosbridge not running** - Most critical, shows prominent banner with start instructions
2. **rosbridge connected but topics not publishing** - Shows which topics are missing
3. **Services not running** - Shows which services need to be started

**Error Display:**
- Persistent error banner for rosbridge status
- Clear diagnostic hints with manual commands
- Service status API works independently (doesn't require rosbridge)

---

**Document Version:** 1.1  
**Last Updated:** December 14, 2025  
**Status:** ✅ Production Ready  
**Next Review:** After major system changes or user feedback

