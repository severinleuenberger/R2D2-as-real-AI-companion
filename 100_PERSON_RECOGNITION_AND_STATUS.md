# Person Recognition and Status System - Complete Setup Guide

**Date:** December 9, 2025 (Updated: December 9, 2025)  
**Version:** 2.0 - 2-State Fixed-Timing Specification  
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
  30 FPS RGB    Image Processing      Person ID         RED/BLUE          🔊 Beeps + 🔴🔵 LEDs
```

**Complete System Flow:**
1. **Camera captures** RGB frames (1920×1080 @ 30 FPS)
2. **Perception pipeline** processes frames (downscales, detects faces)
3. **Face recognition** identifies specific person using LBPH model
4. **Status machine** tracks state (RED=recognized, BLUE=not recognized)
5. **Audio alerts** play MP3 sounds on state transitions
6. **LED feedback** shows visual status via GPIO RGB LED

**Key Capabilities:**
- ✅ Real-time person recognition at 6-13 Hz
- ✅ Fixed-timing state management (15s RED hold, 5s continuous loss)
- ✅ Audio alerts: "Hello!" on recognition, "Oh, I lost you!" on loss
- ✅ Visual feedback: RGB LED shows system state (RED/BLUE only)
- ✅ Production-ready: Systemd services, error handling, logging
- ✅ Training pipeline: Complete workflow for model creation

---

## ⚠️ Specification Correction Note

**Important:** Earlier documentation versions (v1.3 and prior) incorrectly described immediate BLUE transitions when `face_count == 0` or `person_id == "no_person"`. These behaviors are **NOT** part of the authoritative specification. The correct behavior uses only fixed timing rules (15s RED hold + 5s continuous loss) with no shortcuts. This document (v2.0) represents the authoritative specification.

---

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
- Tracks recognition state (RED/BLUE)
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
- 🔊 **"Hello!"** MP3 when target person is recognized (BLUE → RED transition)
- ⏸ **Silent** while continuously recognized in RED state (no repeated beeps)
- 🔔 **"Oh, I lost you!"** MP3 when target person is confirmed lost (RED → BLUE transition)
  - Only plays after minimum 15 seconds in RED state AND 5 seconds of continuous loss
- 🔊 **"Hello!"** MP3 when target person is recognized again (BLUE → RED, immediate, no delay)

**State Transitions:**
```
BLUE → RED: 🔊 "Hello!" plays (immediate when recognized==True)
RED → RED (recognized==True): ⏸ Silent (no audio, stays in RED)
RED → RED (recognized==False, <15s hold): ⏸ Silent (must hold RED for 15s minimum)
RED → RED (recognized==False, <5s loss): ⏸ Silent (waiting for 5s continuous loss)
RED → BLUE: 🔔 "Oh, I lost you!" plays (after 15s hold AND 5s continuous loss)
BLUE → RED: 🔊 "Hello!" plays (immediate re-recognition, no cooldown)
```

**Key Timing Rules:**
- **RED Minimum Hold:** System must remain in RED state for at least 15 seconds after entering RED
- **5s Reacquire Window:** While in RED, if recognition drops, system waits 5 seconds of continuous loss before transitioning to BLUE
- **Immediate Re-recognition:** If `recognized==True` during the 5s loss window, timer is cleared and system stays in RED (no audio)
- **No Cooldowns:** BLUE → RED transitions happen immediately when recognized (no delays or cooldowns)

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

The status system is the core state machine that tracks person recognition and drives all outputs (audio, LED, logging). It implements a **two-state model** (RED/BLUE) with **fixed timing rules** that ensure reliable behavior.

**Key Principles:**
- **Continuous Evaluation:** Recognition boolean (`recognized = person_id == "severin"`) is evaluated continuously at sub-second cadence
- **Fixed Timing Constants:** `REACQUIRE_WINDOW = 5.0s` and `RED_HOLD_TIME = 15.0s` (not configurable)
- **No Shortcuts:** System never bypasses timing rules based on `face_count` or `"no_person"` messages
- **Audio on Entry Only:** Audio plays only on state transitions (BLUE → RED or RED → BLUE), never during state

### 6.2 Two-State Recognition Model

**🔴 RED - Target Person Recognized (Active Engagement)**
- **Conditions:** Target person is currently recognized (`person_id == "severin"`)
- **Status:** `{"status": "red", "person_identity": "severin", ...}` (actual recognized name)
- **LED:** Solid RED (GPIO pin 17)
- **Audio:** "Hello!" played **only** on BLUE → RED transition (never replays while in RED)
- **Minimum Hold Time:** System **must** remain in RED for at least 15 seconds after entering RED
- **Reacquire Rule:** If recognition drops while in RED, system waits 5 seconds of continuous loss before transitioning to BLUE
- **Re-recognition:** If `recognized==True` during loss timer, timer is immediately cleared and system stays in RED (no audio)

**Note:** `person_identity` contains the actual recognized person name from the perception topic (e.g., "severin"), allowing the UI to display the person's actual name instead of a generic identifier.

**🔵 BLUE - Target Person Not Recognized (Idle/Waiting)**
- **Conditions:** Target person not recognized (`person_id != "severin"`)
- **Status:** `{"status": "blue", "person_identity": "no_person", ...}`
- **LED:** Solid BLUE (GPIO pin 22)
- **Audio:** "Oh, I lost you!" played **only** on RED → BLUE transition
- **Transition Conditions:** RED → BLUE only occurs when **BOTH**:
  - Minimum 15 seconds have passed since entering RED (`RED_HOLD_TIME`)
  - Continuous `recognized==False` for at least 5 seconds (`REACQUIRE_WINDOW`)
- **Immediate Re-recognition:** BLUE → RED transitions happen immediately when `recognized==True` (no cooldown, no delay)

### 6.3 State Machine Diagram

```
                              INITIAL STATE
                              🔵 BLUE (idle)
                                   │
                                   │ recognized == True
                                   │ (immediate, no delay)
                                   ▼
                         🔊 Play "Hello!" (Voicy_R2-D2 - 2.mp3)
                         🔴 RED STATE (active)
                         red_enter_time = now
                         last_recognized_true_time = now
                                   │
                    ┌──────────────┴──────────────┐
                    │                            │
         recognized == True              recognized == False
         (stay in RED)                  (start loss timer)
                    │                            │
                    │                            │
         Clear loss timer                time_since_last_recognized
         last_recognized_true_time = now        │
         (no audio, no state change)             │
                                                │
                    ┌───────────────────────────┘
                    │
                    │ Check transition conditions:
                    │ 1. (now - red_enter_time) >= 15.0s? (RED_HOLD_TIME)
                    │ 2. (now - last_recognized_true_time) >= 5.0s? (REACQUIRE_WINDOW)
                    │
                    │ If BOTH conditions met:
                    ▼
         🔊 Play "Oh, I lost you!" (Voicy_R2-D2 - 5.mp3)
         🔵 BLUE STATE (idle)
         Clear all timers
         (ready for immediate re-recognition)
                    │
                    │ recognized == True
                    │ (immediate, no cooldown)
                    ▼
         (repeat cycle)
```

### 6.3.1 Authoritative State Machine Specification (Normative)

This section defines the authoritative specification using normative language (MUST/MUST NOT/SHALL/SHALL NOT). All implementations MUST conform to these rules.

**State Definitions:**
- **RED State:** Target person is recognized (`person_id == target_person`)
- **BLUE State:** Target person is not recognized (`person_id != target_person`)

**Transition Rules (MUST be followed exactly):**

1. **BLUE → RED Transition:**
   - **MUST** occur immediately when `recognized == True` (no delay, no cooldown)
   - **MUST** set `red_enter_time = current_time` on entry
   - **MUST** set `last_recognized_true_time = current_time` on entry
   - **MUST** play recognition audio ("Hello!") on entry only
   - **MUST NOT** replay audio while remaining in RED state

2. **RED State Minimum Hold:**
   - **MUST** remain in RED for a minimum of 15 seconds after BLUE→RED transition
   - **MUST NOT** transition to BLUE until `(current_time - red_enter_time) >= 15.0s`
   - The 15-second RED hold is **ABSOLUTE and NON-BYPASSABLE**
   - **MUST NOT** be overridden by:
     - `face_count == 0`
     - `person_id == "no_person"`
     - Camera loss
     - Missing or stale perception messages
     - Any other condition

3. **RED → BLUE Transition (ONLY Allowed Path):**
   - **MUST** occur **ONLY** when **BOTH** conditions hold:
     - `(current_time - red_enter_time) >= 15.0s` (minimum hold time met)
     - `(current_time - last_recognized_true_time) >= 5.0s` (5s continuous non-recognition)
   - **MUST NOT** occur under any other conditions
   - **MUST** clear `red_enter_time = None` on transition
   - **MUST** clear `last_recognized_true_time = None` on transition
   - **MUST** play loss audio ("Oh, I lost you!") on entry only

4. **Loss Timer Management:**
   - While in RED, if `recognized == False`:
     - **MUST** calculate `time_since_last_recognized = current_time - last_recognized_true_time`
   - If `recognized == True` at any time:
     - **MUST** immediately reset: `last_recognized_true_time = current_time`
     - **MUST** remain in RED (no audio, no state change)
   - The 5s continuous loss window **MUST** represent continuous absence (resets on reacquire), not cumulative

5. **BLUE → RED Re-recognition:**
   - **MUST** occur immediately when `recognized == True` (no cooldown, no delay)
   - **MUST NOT** impose any delay or cooldown period

**Timer Management Requirements:**
- `red_enter_time` **MUST** be set only on BLUE→RED entry
- `red_enter_time` **MUST NOT** be reset on every recognized frame
- `red_enter_time` **MUST** only be cleared on RED→BLUE transition
- `last_recognized_true_time` **MUST** be updated when `recognized` becomes True (for loss timer reset)
- `last_recognized_true_time` is **INDEPENDENT** of `red_enter_time` (both must be tracked separately)

**Input Handling Requirements:**
- Implementations **MUST** handle stale input via a timer/watchdog mechanism
- If no `person_id` message received for threshold (e.g., 3 seconds), **MUST** treat input as stale
- When input is stale, **MUST** force `recognized = False` to allow normal loss logic to progress
- **MUST NOT** use stale input detection to bypass timing rules (15s hold + 5s continuous loss still apply)

### 6.4 Fixed Timing Rules

**Timing Constants (Fixed, Not Configurable):**

| Constant | Value | Purpose |
|----------|-------|---------|
| `REACQUIRE_WINDOW` | `5.0s` | Continuous `recognized==False` required in RED before transition to BLUE |
| `RED_HOLD_TIME` | `15.0s` | Minimum seconds RED state must be held (guaranteed minimum duration) |

**Core Rules:**

1. **BLUE → RED Transition:**
   - Trigger: `recognized == True` (immediate, no delay)
   - Actions: Play "Hello!", set `red_enter_time = now`, clear loss timers
   - Audio: Plays only on entry, never replays while in RED

2. **RED Minimum Hold:**
   - System **must** remain in RED for at least 15 seconds after entering RED
   - Cannot transition to BLUE until `(now - red_enter_time) >= 15.0s`
   - This ensures RED state is maintained regardless of brief recognition dropouts

3. **5s Reacquire Rule in RED:**
   - While in RED, if `recognized == False`:
     - Start/continue loss timer: `time_since_last_recognized = now - last_recognized_true_time`
   - If `recognized == True` at any time:
     - Immediately clear loss timers: `last_recognized_true_time = now`
     - Remain in RED (no audio, no state change)

4. **RED → BLUE Transition (Only Allowed Path):**
   - Transition occurs **only** when **BOTH** conditions hold:
     - `(now - red_enter_time) >= 15.0s` (minimum hold time met)
     - `(now - last_recognized_true_time) >= 5.0s` (5s continuous loss)
   - Actions: Play "Oh, I lost you!", clear all timers, transition to BLUE

5. **BLUE Reset:**
   - After BLUE, RED may occur immediately when `recognized == True` (no cooldown, no delay)
   - BLUE is a full reset for the next "first recognition"

**⚠️ Important Notes (December 2025):**
- **No Shortcuts:** System never uses `face_count == 0` or `person_id == "no_person"` to bypass timing rules
- **No Cooldowns:** BLUE → RED transitions happen immediately (no delays or cooldowns)
- **Audio on Entry Only:** Audio plays only on state transitions, never during state
- **Continuous Evaluation:** Recognition boolean is evaluated continuously at sub-second cadence (every `person_id` callback and every 0.5s timer tick)

### 6.4.1 Known Implementation Risks

This section documents known risks and requirements for correct implementation.

**Risk 1: RED State Can Get Stuck Forever**

**Problem:** If upstream recognition continuously publishes the target person value (e.g., "severin") even when the person is gone, the system will remain in RED state indefinitely.

**Root Causes:**
- Upstream recognition never publishes a non-target value
- The RED entry timestamp (`red_enter_time`) is reset on every recognized frame (MUST only be set on BLUE→RED transition)
- Missing watchdog/timer mechanism for stale input

**Requirements:**
- Implementations **MUST** handle stale input via a timer/watchdog mechanism
- If no `person_id` message received for threshold (e.g., 3 seconds), **MUST** force `recognized = False`
- **MUST NOT** use stale input detection to bypass timing rules (15s hold + 5s continuous loss still apply)
- `red_enter_time` **MUST** be set only on BLUE→RED entry
- `red_enter_time` **MUST NOT** be reset on every recognized frame

**Risk 2: Loss Timer Not Representing Continuous Non-Recognition**

**Problem:** The 5s continuous loss window must represent continuous absence, not cumulative loss. If the timer is not reset on reacquire, the system may transition to BLUE prematurely.

**Requirements:**
- `last_recognized_true_time` **MUST** be updated to `current_time` when `recognized` becomes True
- This ensures the 5s window represents continuous absence (resets on reacquire), not cumulative loss
- The loss timer **MUST** reset immediately when recognition is reacquired

**Risk 3: Missing Watchdog/Timer Tick for Stale Input**

**Problem:** If the `person_id` topic stops publishing, the `recognized` boolean will remain at its last value. If the last value was `True`, the system will stay in RED forever.

**Requirements:**
- Implementations **MUST** track the timestamp of the last `person_id` message received
- Implementations **MUST** check for stale input in the state machine evaluation loop
- When input is stale, **MUST** force `recognized = False` to allow normal loss logic to progress
- The stale input check **MUST NOT** bypass timing rules (it only affects the `recognized` boolean)

### 6.5 Status Message Format

**Published Topic:** `/r2d2/audio/person_status` (std_msgs/String, JSON, 10 Hz)

**Message Structure:**
```json
{
  "status": "red|blue",
  "person_identity": "severin|no_person",
  "timestamp_sec": 1765212914,
  "timestamp_nanosec": 949382424,
  "confidence": 0.95,
  "duration_in_state": 15.3
}
```

**Note:** `person_identity` contains the actual recognized person name from the perception topic (e.g., "severin" when recognized, "no_person" when not recognized). This allows the web dashboard and other consumers to display the actual person's name. The system only has two states: RED (recognized) and BLUE (not recognized).

**Example Messages:**

**RED State:**
```json
{
  "status": "red",
  "person_identity": "severin",
  "confidence": 0.95,
  "duration_in_state": 12.5
}
```

**BLUE State:**
```json
{
  "status": "blue",
  "person_identity": "no_person",
  "confidence": 0.0,
  "duration_in_state": 8.3
}
```

### 6.6 LED Visual Feedback

**RGB LED Node:** `status_led_node` (optional, requires GPIO)

**GPIO Pins:**
- RED: GPIO 17 (Pin 17 on 40-pin header)
- BLUE: GPIO 22 (Pin 22 on 40-pin header)

**Note (December 2025):** The system uses the original/default Jetson AGX Orin 40-pin header configuration. All GPIO pins (17, 22, 32) are available for their default functions. No custom device tree overlays are active.

**LED Behavior:**

| Status | Red | Blue | Visual |
|--------|-----|------|--------|
| RED (recognized) | ON | OFF | 🔴 Solid Red |
| BLUE (not recognized) | OFF | ON | 🔵 Solid Blue |

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
- ✅ Permanent `target_person_name` parameter configuration (e.g., `target_person_name:=severin`)

**Camera Perception Service Configuration:**
The `r2d2-camera-perception.service` file includes the `target_person_name` parameter to ensure consistent person identification:
```bash
ExecStart=/bin/bash -c 'source install/setup.bash && ros2 launch r2d2_bringup r2d2_camera_perception.launch.py enable_face_recognition:=true target_person_name:=severin'
```
This ensures the perception node publishes the actual person name (e.g., "severin") instead of a generic identifier, allowing the web dashboard and other consumers to display the correct name.

### 7.2 Complete System Startup

**Full R2D2 Startup Sequence:**

**Terminal 1: Camera + Perception + Recognition**
```bash
cd ~/dev/r2d2/ros2_ws
source ~/depthai_env/bin/activate
source ~/.bashrc
source install/setup.bash

ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true \
  target_person_name:=severin
```

**Note:** The `target_person_name` parameter ensures the perception node publishes the correct person name (e.g., "severin") instead of a generic identifier. This parameter is automatically set in the systemd service file for production use.

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
timeout 5s ros2 topic echo /r2d2/audio/person_status --once --no-arr
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

**Active Parameters:**

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `target_person` | String | `severin` | any name | Person to recognize (should match training data) |
| `audio_volume` | Float | `0.05` | 0.0-1.0 | Global volume control (0-100%, default 5% - very quiet) |
| `recognition_audio_file` | String | `Voicy_R2-D2 - 2.mp3` | filename | Audio file for "Hello!" |
| `loss_audio_file` | String | `Voicy_R2-D2 - 5.mp3` | filename | Audio file for "Lost you!" |
| `enabled` | Boolean | `true` | true/false | Enable/disable system |

**Deprecated Parameters (kept for compatibility, not used):**
- `jitter_tolerance_seconds` - Deprecated, state machine uses fixed `REACQUIRE_WINDOW = 5.0s`
- `loss_confirmation_seconds` - Deprecated, state machine uses fixed `RED_HOLD_TIME = 15.0s`

**⚠️ Important:** Timing constants (`REACQUIRE_WINDOW = 5.0s`, `RED_HOLD_TIME = 15.0s`) are **fixed** and cannot be changed via parameters. The state machine implements exact timing rules as specified.

**View Current Parameters:**
```bash
ros2 param list /audio_notification_node
ros2 param get /audio_notification_node audio_volume
ros2 param get /audio_notification_node target_person
```

**Set Parameters (Runtime, Temporary):**
```bash
ros2 param set /audio_notification_node audio_volume 0.3
ros2 param set /audio_notification_node target_person alice
```

**Set Parameters (Permanent):**
Edit service file:
```bash
sudo nano /etc/systemd/system/r2d2-audio-notification.service
# Modify ExecStart line to add parameters:
ExecStart=/home/severin/dev/r2d2/start_audio_service.sh audio_volume:=0.3 target_person:=alice
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
- **State Machine Timing:** Timing constants are fixed (`REACQUIRE_WINDOW = 5.0s`, `RED_HOLD_TIME = 15.0s`) and cannot be tuned

### 8.3 Configuration Examples

**Loud Volume Setup:**
```bash
ros2 launch r2d2_audio audio_notification.launch.py \
  audio_volume:=0.8 \
  target_person:=severin
```

**Quiet Volume Setup:**
```bash
ros2 launch r2d2_audio audio_notification.launch.py \
  audio_volume:=0.1 \
  target_person:=severin
```

**Default Setup (5% volume, very quiet):**
```bash
ros2 launch r2d2_audio audio_notification.launch.py
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
- 📊 **Status Stream**: Real-time state changes (RED/BLUE)
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
- Status changes to: `{"status": "red", "person_identity": "severin", ...}` (actual recognized name)
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

# Check current volume (default is 0.05 = 5%, very quiet)
ros2 param get /audio_notification_node audio_volume

# Test ffplay directly (should work with default ALSA device)
ffplay -nodisp -autoexit -af "volume=0.5" /path/to/audio.mp3

# Check if ffplay processes are spawned
ps aux | grep ffplay | grep -v grep

# Monitor service logs for audio playback messages
journalctl -u r2d2-audio-notification.service -f | grep -E "Playing|audio|Error"
```

**Common Causes and Solutions:**

1. **Volume too low (most common):**
   - Default volume is 0.05 (5%), which is very quiet
   - Solution: `ros2 param set /audio_notification_node audio_volume 0.5` (50% volume)

2. **ffplay ALSA device syntax issue (fixed December 2025):**
   - **Root Cause:** ffplay doesn't support `-ao alsa:device=hw:1,0` syntax
   - **Status:** ✅ Fixed in `audio_player.py` - now uses default ALSA device from `~/.asoundrc`
   - **If still not working:** Rebuild package: `colcon build --packages-select r2d2_audio` and restart service

3. **Audio files not found:**
   - Verify files exist: `ls -lh ~/dev/r2d2/ros2_ws/install/r2d2_audio/share/r2d2_audio/assets/audio/`
   - If missing: Rebuild package: `colcon build --packages-select r2d2_audio`

4. **ALSA hardware issues:**
   - Verify ALSA routing: `aplay -l` should show audio device
   - Test ALSA directly: `aplay -D hw:1,0 /usr/share/sounds/alsa/Front_Left.wav`
   - Check speaker power: Look for power indicator on PAM8403 amplifier
   - Verify J511 I2S connector is fully seated

5. **Service not using updated code:**
   - After code changes, rebuild and restart: 
     ```bash
     cd ~/dev/r2d2/ros2_ws
     colcon build --packages-select r2d2_audio
     sudo systemctl restart r2d2-audio-notification.service
     ```

**Debugging Steps:**
1. Check service logs for "🔊 Playing" messages (INFO level)
2. Verify ffplay process spawns: `ps aux | grep ffplay`
3. Test audio_player.py directly:
   ```bash
   python3 ~/dev/r2d2/ros2_ws/install/r2d2_audio/lib/python3.10/site-packages/r2d2_audio/audio_player.py \
     ~/dev/r2d2/ros2_ws/install/r2d2_audio/share/r2d2_audio/assets/audio/Voicy_R2-D2\ -\ 2.mp3 \
     0.5 hw:1,0
   ```

**For detailed debugging guide, see:** [`AUDIO_DEBUG_FINDINGS.md`](AUDIO_DEBUG_FINDINGS.md)

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
# Check current status
ros2 topic echo /r2d2/audio/person_status --once --no-arr

# Monitor state transitions
journalctl -u r2d2-audio-notification.service -f | grep "lost\|recognized"
```

**Expected Timing:** ~20 seconds total (15s RED hold + 5s continuous loss)

**Important:** Timing is **fixed and not configurable**. The system uses:
- `RED_HOLD_TIME = 15.0s` (minimum RED state duration)
- `REACQUIRE_WINDOW = 5.0s` (continuous loss required)

**Solutions:**
1. **Timing cannot be adjusted** - these are fixed constants in the state machine
2. If alerts fire too early/too late, the issue is likely with recognition stability, not timing
3. **Improve recognition parameters:**
   - Adjust face recognition confidence thresholds
   - Improve camera positioning for stable face detection
   - Ensure adequate lighting for consistent recognition
4. **Monitor recognition stability:**
   ```bash
   ros2 topic hz /r2d2/perception/person_id
   ros2 topic echo /r2d2/perception/person_id
   ```

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

### 10.7 Issue: Audio Beeps Too Frequent

**Symptom:** Audio alerts play too often

**Diagnosis:**
```bash
# Monitor state transitions
journalctl -u r2d2-audio-notification.service -f | grep "recognized\|lost"

# Check recognition stability
ros2 topic echo /r2d2/perception/person_id
```

**Common Causes:**
1. **Rapid state transitions:** System is transitioning between RED and BLUE frequently
2. **Recognition instability:** Face recognition is unstable, causing frequent recognition/loss cycles
3. **Camera positioning:** Camera angle or lighting causing intermittent face detection

**Important:** The 2-state model does **not** have a cooldown parameter. Audio plays **only** on state transitions (BLUE→RED or RED→BLUE). If beeps are too frequent, it's due to rapid state transitions, not cooldown settings.

**Solutions:**
1. **Improve recognition stability:**
   - Adjust camera positioning for stable face view
   - Ensure adequate lighting for consistent recognition
   - Reduce camera shake or movement

2. **Improve face detection:**
   - Check face detection rate: `ros2 topic hz /r2d2/perception/face_count`
   - Ensure face is consistently detected before recognition runs

3. **Monitor state transitions:**
   ```bash
   ros2 topic echo /r2d2/audio/person_status --no-arr
   ```
   If you see frequent RED↔BLUE transitions, the issue is recognition stability, not audio settings.

**Expected Behavior:**
- Recognition alert plays **once** on BLUE→RED transition
- Loss alert plays **once** on RED→BLUE transition
- No audio while staying in the same state
- If beeps are frequent, it indicates rapid state transitions (recognition instability)

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

# Note: Timing is fixed (15s RED hold + 5s continuous loss) and cannot be adjusted
# If loss detection seems too slow/fast, improve recognition stability instead

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
    ├─ audio_notification_node: State machine (RED/BLUE)
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
- `/r2d2/audio/person_status` (std_msgs/String, JSON, 10 Hz) - Status (RED/BLUE)
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
✅ **Status System:** Two-state model (RED/BLUE) with LED feedback  
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

**Document Version:** 1.4  
**Last Updated:** December 15, 2025  
**Status:** ✅ Production Ready  
**Next Review:** After major system changes or user feedback

**Recent Changes (v2.0 - December 9, 2025):**
- **Documentation aligned to authoritative 2-state fixed-timing specification**
  - Removed all references to GREEN state (system uses only RED/BLUE)
  - Removed references to immediate BLUE shortcuts (face_count, no_person bypasses)
  - Removed configurable timing parameter suggestions (timing is fixed: 15s hold + 5s continuous loss)
  - Removed cooldown logic references (audio plays only on state transitions)
  - Added normative specification section (6.3.1) with MUST/MUST NOT language
  - Added implementation risks section (6.4.1) with explicit warnings
  - Added specification correction note explaining earlier versions were incorrect
  - Fixed monitoring commands to use supported ROS2 CLI patterns
  - Updated all state descriptions to reflect two-state model only

**Recent Changes (v1.4 - December 15, 2025):**
- **Audio Debugging & Fixes:** Enhanced troubleshooting section with ffplay ALSA device fix
  - Added detailed diagnosis steps for "No Audio Heard" issue
  - Documented ffplay compatibility fix (removed unsupported `-ao alsa:device=` syntax)
  - Enhanced error logging in audio_notification_node.py (DEBUG → INFO for playback messages)
  - See [`AUDIO_DEBUG_FINDINGS.md`](AUDIO_DEBUG_FINDINGS.md) for complete debugging process
  - **Note:** Cooldown references in v1.4 are deprecated - 2-state model uses state-transition-only audio

**Recent Changes (v1.3 - December 15, 2025) - DEPRECATED/INCORRECT:**
- **Previous documentation incorrectly described immediate BLUE transitions** when `face_count == 0` or `person_id == "no_person"`. This behavior is **NOT** part of the authoritative specification. The correct behavior uses only fixed timing rules (15s RED hold + 5s continuous loss) with no shortcuts. See v2.0 for the authoritative specification.

**Recent Changes (v1.2 - December 14, 2025):**
- Updated `person_identity` field to display actual recognized person name (e.g., "severin") instead of generic "target_person" parameter value
- Web dashboard now shows actual person names in Recognition Status and Status Stream sections
- Status messages now use the actual person ID from perception topic for better UI clarity

