# Person Recognition Status System - Master Documentation

**Date:** December 8, 2025  
**Version:** 2.0 - Comprehensive Merged Documentation  
**Status:** ✅ Fully Implemented & Tested  
**Platform:** NVIDIA Jetson AGX Orin 64GB with ROS 2 Humble  
**Author:** R2D2 Development Team

---

## 📋 Quick Links to Related Documentation

This is the **comprehensive master document** for the complete person recognition and audio notification system. Related documentation:

- **System Overview** → [`001_ARCHITECTURE_OVERVIEW.md`](001_ARCHITECTURE_OVERVIEW.md)
- **Camera Setup** → [`011_CAMERA_SETUP_DOCUMENTATION.md`](011_CAMERA_SETUP_DOCUMENTATION.md)
- **Speaker Hardware Setup** → [`012_SPEAKER_AUDIO_SETUP_DOCUMENTATION.md`](012_SPEAKER_AUDIO_SETUP_DOCUMENTATION.md)
- **Agent Instructions** → [`000_INTERNAL_AGENT_NOTES.md`](000_INTERNAL_AGENT_NOTES.md)

---

## 1. Overview

The **Person Recognition & Audio Notification System** is a complete end-to-end solution that:

1. **Detects and recognizes faces** using the OAK-D camera and LBPH face recognition
2. **Publishes recognition results** on ROS 2 topics (`/r2d2/perception/person_id`)
3. **Manages state transitions** with smart jitter tolerance and loss confirmation
4. **Plays audio alerts** when person is recognized or lost
5. **Provides visual feedback** via RGB LED status indicators
6. **Publishes status messages** for downstream integration (dialogue system, logging, etc.)

This document consolidates the complete implementation workflow from face recognition through audio notifications.

### What It Does

```
Face Recognition Input     Status Machine        Outputs
/r2d2/perception/person_id ──→ State Transitions ──→ /r2d2/audio/person_status (JSON)
                                                      ├→ Audio alerts (ffplay MP3)
                                                      ├→ LED feedback (GPIO RGB)
                                                      ├→ Dialogue context
                                                      └→ Event logging
```

### Key Capabilities

- ✅ **Real-Time Face Recognition:** LBPH-based person identification at 6-13 Hz
- ✅ **Real-Time State Tracking:** Three-state model (RED/BLUE/GREEN)
- ✅ **Smart Audio Alerts:** "Hello!" on recognition, "Oh, I lost you!" on loss
- ✅ **Jitter Handling:** 5-second tolerance for brief interruptions
- ✅ **Loss Confirmation:** 15-second confirmation window prevents false alarms
- ✅ **Anti-Spam Cooldown:** 2-second minimum between repeated alerts
- ✅ **Visual Feedback:** RGB LED shows system state via GPIO
- ✅ **Status Publishing:** JSON messages for downstream integration
- ✅ **Event Logging:** Console logging + future SQLite database
- ✅ **Auto-Start Service:** Systemd service for production deployment

### Complete System Flow

```
OAK-D Camera (1920×1080 @ 30 FPS)
    ↓
Perception Pipeline (image_listener.py)
    ├─ Downscale to 640×360
    ├─ Convert to grayscale
    ├─ Face Detection (Haar Cascade, ~13 Hz)
    └─ Face Recognition (LBPH, 6-13 Hz)
    ↓
Publishes: /r2d2/perception/person_id ("severin" or "unknown")
    ↓
Audio Notification Node (audio_notification_node.py)
    ├─ State Machine (RED/BLUE/GREEN)
    ├─ Jitter Tolerance (5s)
    ├─ Loss Confirmation (15s)
    └─ Audio Playback (ffplay MP3)
    ↓
Outputs:
    ├─ /r2d2/audio/person_status (JSON status messages)
    ├─ /r2d2/audio/notification_event (event logs)
    ├─ Audio alerts via PAM8403 amplifier + speaker
    └─ RGB LED feedback (GPIO)
```

---

## 1.1 Prerequisites

Before implementing the person recognition system, ensure:

1. ✅ **Camera is configured** (see [`011_CAMERA_SETUP_DOCUMENTATION.md`](011_CAMERA_SETUP_DOCUMENTATION.md))
   - OAK-D Lite connected and publishing to `/oak/rgb/image_raw`
   - Camera node running: `ros2 launch r2d2_camera camera.launch.py`

2. ✅ **Speaker hardware is set up** (see [`012_SPEAKER_AUDIO_SETUP_DOCUMENTATION.md`](012_SPEAKER_AUDIO_SETUP_DOCUMENTATION.md))
   - PAM8403 amplifier wired to Jetson J511 header
   - ALSA configured (`/etc/asound.conf` created)
   - Audio test successful: `aplay -D hw:1,0 /path/to/test.wav`

3. ✅ **Face recognition model is trained**
   - Training images captured: `~/dev/r2d2/data/face_recognition/severin/`
   - Model file exists: `~/dev/r2d2/data/face_recognition/models/severin_lbph.xml`
   - See Section 2.3 for training instructions

4. ✅ **ROS 2 workspace is built**
   ```bash
   cd ~/dev/r2d2/ros2_ws
   colcon build --packages-select r2d2_perception r2d2_audio
   source install/setup.bash
   ```

---

## 2. Face Recognition System

### 2.1 Overview

The face recognition system uses **OpenCV's LBPH (Local Binary Pattern Histograms) recognizer** to identify the target person in real-time. The system processes camera frames at 6-13 Hz depending on CPU budget.

**Why LBPH?**
- ✅ CPU-efficient (no GPU needed)
- ✅ Fast enough for real-time on ARM (Jetson)
- ✅ Works on standard Jetson without CUDA
- ✅ Acceptable accuracy for personal recognition (80%+)
- ✅ Small model size (33 MB)
- ✅ Works well with limited training data

### 2.2 ROS 2 Topics Published

When face recognition is enabled, the perception pipeline publishes:

| Topic | Type | Frequency | Description |
|-------|------|-----------|-------------|
| `/r2d2/perception/person_id` | `std_msgs/String` | ~6 Hz | Person name ("severin" or "unknown") |
| `/r2d2/perception/face_confidence` | `std_msgs/Float32` | ~6 Hz | Confidence score (lower = better, 0-40=strong match) |
| `/r2d2/perception/is_severin` | `std_msgs/Bool` | ~6 Hz | Boolean convenience topic |

### 2.3 Training the Face Recognition Model

**Step 1: Capture Training Data**

```bash
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8

python3 1_capture_training_data.py
```

**What it does:**
- Captures ~80 images from camera
- Saves to `~/dev/r2d2/data/face_recognition/severin/`
- 4 tasks with different lighting/distances (20 sec each)
- Diverse angles and expressions

**Step 2: Train the Model**

```bash
python3 2_train_recognizer.py
```

**What it does:**
- Reads all images from training directory
- Extracts faces using Haar Cascade
- Trains LBPH recognizer
- Saves model to `~/dev/r2d2/data/face_recognition/models/severin_lbph.xml`
- Duration: 30-60 seconds

**Step 3: Test the Model**

```bash
python3 3_test_recognizer_demo.py
```

### 2.4 Launching Face Recognition

**Basic Launch (After Training):**

```bash
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true
```

**With Custom Settings:**

```bash
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true \
  recognition_confidence_threshold:=75.0 \
  recognition_frame_skip:=2
```

**Parameters:**
- `enable_face_recognition`: Enable/disable LBPH recognition (default: `false`)
- `face_recognition_model_path`: Path to trained model (default: `~/dev/r2d2/data/face_recognition/models/severin_lbph.xml`)
- `recognition_confidence_threshold`: Threshold for classification (default: `70.0`, lower=stricter)
- `recognition_frame_skip`: Process every Nth frame (default: `2`, manages CPU load)

### 2.5 Performance & CPU Usage

| Configuration | CPU | Recognition Rate | Use Case |
|---------------|-----|------------------|----------|
| `frame_skip=1` | 18-25% | ~13 Hz | Maximum accuracy |
| `frame_skip=2` | 10-15% | ~6.5 Hz | **Recommended** |
| `frame_skip=3` | 8-10% | ~4.3 Hz | Minimal CPU |

**Recommended:** `frame_skip=2` provides good balance between accuracy and CPU usage.

### 2.6 Monitoring Face Recognition

```bash
# Watch person identification
ros2 topic echo /r2d2/perception/person_id

# Watch confidence scores
ros2 topic echo /r2d2/perception/face_confidence

# Check publishing rate
ros2 topic hz /r2d2/perception/person_id
```

---

## 3. Audio Notifications & ROS 2 Integration

### 3.1 Overview

The audio notification system subscribes to `/r2d2/perception/person_id` and plays MP3 audio alerts when state transitions occur. It uses `ffplay` to play audio files with volume control.

**Audio Files:**
- **Recognition Alert:** `Voicy_R2-D2 - 2.mp3` ("Hello!" - ~2 seconds)
- **Loss Alert:** `Voicy_R2-D2 - 5.mp3` ("Oh, I lost you!" - ~5 seconds)

**File Locations:**
- Source: `~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/`
- Installed: `~/dev/r2d2/ros2_ws/install/r2d2_audio/share/r2d2_audio/assets/audio/`

### 3.2 Audio Notification Node

**File:** `ros2_ws/src/r2d2_audio/r2d2_audio/audio_notification_node.py`

**Key Features:**
- Subscribes to `/r2d2/perception/person_id`
- Manages state transitions (RED/BLUE/GREEN)
- Plays audio alerts on state changes
- Publishes status messages to `/r2d2/audio/person_status`
- Implements jitter tolerance and loss confirmation

### 3.3 Launching Audio Notifications

**Manual Launch (Development):**

```bash
cd ~/dev/r2d2/ros2_ws
source install/setup.bash
ros2 launch r2d2_audio audio_notification.launch.py
```

**With Custom Parameters:**

```bash
ros2 launch r2d2_audio audio_notification.launch.py \
  audio_volume:=0.3 \
  loss_confirmation_seconds:=20.0
```

### 3.4 Audio Volume Control

The **`audio_volume`** parameter controls ALL audio output (0.0-1.0, default: 0.05 = 5%).

**Adjust Volume:**

```bash
# Runtime change (temporary)
ros2 param set /audio_notification_node audio_volume 0.3

# Launch with custom volume
ros2 launch r2d2_audio audio_notification.launch.py audio_volume:=0.4
```

**Volume Levels:**
- `0.05` = 5% (very quiet - current default)
- `0.2` = 20% (moderate)
- `0.5` = 50% (loud)
- `1.0` = 100% (maximum)

### 3.5 Systemd Service (Production)

**Installation:**

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
```

---

## 4. Three-State Recognition Model

The system recognizes three distinct states:

### 🔴 RED - Target Person Recognized (Active Engagement)

```
Conditions:
  • Target person "severin" is currently visible to camera
  • Face has been continuously tracked (or within jitter tolerance)
  • System is actively engaged with the target person

Behavior:
  • Status message: {"status": "red", "person_identity": "severin", ...}
  • LED Color: Solid RED on GPIO
  • Audio: No beeps (beep already played when entering this state)
  • Duration: Continuous while visible (continuously updated)
  • Next State: Transition to BLUE (loss) after 5s jitter + 15s confirmation

Example Timeline:
  0:00 → "severin" detected → Play "Hello!" beep → Enter RED state
  0:05 → Jitter gap (person briefly not visible) → Stay RED (within tolerance)
  0:10 → "severin" visible again → Stay RED (no new beep)
  0:20 → Still visible → Stay RED (duration now 20 seconds)
```

### 🔵 BLUE - No Person Recognized (Idle/Waiting)

```
Conditions:
  • No target person is visible
  • Confirmed loss: 5s jitter + 15s confirmation window
  • System is idle, waiting for next recognition

Behavior:
  • Status message: {"status": "blue", "person_identity": "no_person", ...}
  • LED Color: Solid BLUE on GPIO
  • Audio: "Oh, I lost you!" beep (played on transition to BLUE)
  • Duration: Continues until target person reappears
  • Next State: Transition to RED when target person detected

Example Timeline:
  0:00 → RED state, "severin" visible
  0:15 → "severin" leaves camera view (jitter timer starts)
  0:20 → Still not visible (5s jitter exceeded)
  0:35 → 15s confirmation window complete → Play "Lost you!" beep → Enter BLUE
  0:40 → BLUE state continues (waiting for "severin" to return)
  0:50 → "severin" reappears → Play "Hello!" beep → Back to RED
```

### 🟢 GREEN - Unknown Person Detected (Caution)

```
Conditions:
  • A face is detected but it's not the target person "severin"
  • This includes "unknown" and any other identified person
  • Unknown person is currently visible

Behavior:
  • Status message: {"status": "green", "person_identity": "other_person", ...}
  • LED Color: Solid GREEN on GPIO
  • Audio: NO beeps (silent detection)
  • Duration: While unknown person visible
  • Next State: 
    - To RED if target person appears (target takes priority)
    - Back to BLUE if unknown person leaves and target not visible

Example Timeline:
  0:00 → BLUE state, no one visible
  0:05 → Camera detects face "alice" → Enter GREEN state (no beep)
  0:10 → GREEN state continues while "alice" visible
  0:15 → "severin" enters frame → Exit GREEN → Enter RED (plays "Hello!")
  0:20 → RED state, "severin" engaged with "alice" still visible

Key Point: GREEN does NOT prevent RED recognition. Target person always has priority.
```

---

## 5. State Machine Architecture

### 3.1 State Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    PERSON RECOGNITION STATE MACHINE                      │
└─────────────────────────────────────────────────────────────────────────┘

                              INITIAL STATE
                              (BLUE: idle)
                                   │
                    ┌──────────────┬┴──────────────┐
                    │              │               │
                    │              │               │
              "severin"       "unknown"      other person
              detected        or other        detected
                    │         person           │
                    │         detected         │
                    ▼              │            ▼
              🔊 Play "Hello!"     │        🟢 GREEN STATE
              🔴 RED STATE         ▼        (no beep, silent)
              (active 15s)     🟢 GREEN     ├─ person_identity
                    │          STATE        │  = actual_name
                    │          (silent)     ├─ confidence = 0.95
                    │            │          └─ duration shown
                    │            │
         ┌──────────┴────────┐   │
         │                   │   │
    RED continues       "severin"
    (visible or        appears
     jitter < 5s)         │
         │                │
         │                ▼
         │           🔴 RED STATE
         │           (1-beep rule:
         │            no beep if
         │            within 15s)
         │                │
         │    ┌───────────┴────────────┐
         │    │                        │
         │    │ "severin"         Loss confirmed:
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
         │    GREEN still
         │    visible?
         │      │ Yes
         │      ▼
         │    🟢 GREEN
         │    (keep GREEN
         │     until exit
         │     or RED)
         │      │
         │      │ No
         │      ▼
         │    Return to
         │    previous
         │    (RED or BLUE)
         │
         └─────────────────→ BLUE STATE (idle, waiting)
                            for next recognition
```

### 3.2 Timing Configuration

```python
# All parameters are ROS 2 parameters, configurable at runtime
# File: audio_notification_node.py (lines ~60-75)

target_person = "severin"                    # Target to recognize
jitter_tolerance_seconds = 5.0               # Brief interruption tolerance
loss_confirmation_seconds = 15.0             # Confirmation window AFTER jitter
cooldown_seconds = 2.0                       # Min between same alert type
recognition_cooldown_after_loss_seconds = 5.0  # Quiet period after loss alert (NEW!)
audio_volume = 0.05                          # Volume level (0.0-1.0, default 5%)

recognition_audio_file = "Voicy_R2-D2 - 2.mp3"    # "Hello!" file
loss_audio_file = "Voicy_R2-D2 - 5.mp3"           # "Lost you!" file
```

### 3.3 Complete Timing Diagram

```
Timeline: Person recognition cycle

0:00 ┌─── RED state enters (severin detected) ────────────────────┐
     │   • Status: RED                                             │
     │   • Audio: "Hello!" plays (5% volume, ~2 seconds)           │
     │   • LED: Solid RED                                          │
     │   • 15s beep cooldown starts                                │
     │                                                              │
0:05 │   Jitter: severin briefly not detected (< 5s tolerance)     │
     │   • Status: Still RED (jitter ignored)                      │
     │   • Audio: Silent (within jitter tolerance)                 │
     │   • LED: Still RED                                          │
     │                                                              │
0:10 │   Severin reappears (within jitter window)                  │
     │   • Status: Still RED (continuous tracking)                 │
     │   • Audio: Silent (no beep yet, still in 15s cooldown)      │
     │   • LED: Still RED                                          │
     │                                                              │
0:15 ├─── RED: 15 seconds elapsed (beep cooldown expires) ────────┤
     │   • Status: RED (duration now 15 seconds)                   │
     │   • Audio: Silent (beep window closed, not re-triggering)   │
     │   • LED: Still RED                                          │
     │   • Beep window now closed (no beep on jitter)              │
     │                                                              │
0:20 │   LOSS BEGINS: Severin leaves camera view                   │
     │   • Status: Still RED (jitter timer starts)                 │
     │   • Audio: Silent (still within jitter tolerance)           │
     │   • LED: Still RED (no immediate change)                    │
     │                                                              │
0:25 ├─── JITTER EXCEEDED: 5 seconds since last detection ────────┤
     │   • Status: Still RED (now in loss confirmation window)     │
     │   • Audio: Silent (waiting for 15s confirmation)            │
     │   • LED: Still RED (waiting for confirmation)               │
     │   • Confirmation timer active: 10 seconds remaining         │
     │                                                              │
0:35 ├─── LOSS CONFIRMED: 5s jitter + 15s confirmation ──────────┤
     │   • Status: BLUE (loss confirmed, entering idle)            │
     │   • Audio: "Oh, I lost you!" plays (5% volume, ~2s)         │
     │   • LED: Solid BLUE (idle state)                            │
     │   • QUIET PERIOD STARTS: 5s suppression window              │
     │                                                              │
0:37 │   Severin reappears (DURING quiet period)                   │
     │   • Status: Still BLUE (quiet period blocks RED transition) │
     │   • Audio: Silent (quiet period suppresses 'Hello!' beep)   │
     │   • LED: Still BLUE (no transition during quiet period)     │
     │                                                              │
0:40 ├─── QUIET PERIOD EXPIRES ─────────────────────────────────┤
     │   • Status transitions allowed again                        │
     │   • If severin still visible: RED state + "Hello!" beep     │
     │   • If severin gone: stays BLUE (idle)                      │
     │                                                              │
0:41 │   Severin still visible (after quiet period)                │
     │   • Status: RED (finally recognized, 5s+ after loss beep)   │
     │   • Audio: "Hello!" plays (5% volume)                       │
     │   • LED: Solid RED (engaged again)                          │
     │   • Cycle repeats                                           │
     │                                                              │
     └──────────────────────────────────────────────────────────────┘

KEY TIMING:
- 0:00-0:05: Jitter tolerance (brief interruptions ignored)
- 0:05-0:20: Loss confirmation window (waiting 15s after jitter)
- 0:20-0:35: Detection + confirmation (~20s total to loss alert)
- 0:35-0:40: QUIET PERIOD (5s calm period, no new alerts)
- 0:40+: Recognition allowed again (no nervous rapid beeping)

TOTAL TIME TO LOSS ALERT: ~20 seconds (5s jitter + 15s confirmation)
CALM DELAY BEFORE RE-RECOGNITION: 5 seconds (prevents nervous rapid cycles)
```

---

## 6. Person Status Message Format

### 4.1 JSON Message Structure

Published to `/r2d2/audio/person_status` as ROS 2 `std_msgs/String`:

```json
{
  "status": "red|blue|green",
  "person_identity": "severin|no_person|other_name",
  "timestamp_sec": 1765212914,
  "timestamp_nanosec": 949382424,
  "confidence": 0.95,
  "duration_seconds": 15.3,
  "is_loss_state": false,
  "audio_event": "recognition|loss|none"
}
```

### 4.2 Field Descriptions

| Field | Type | Values | Description |
|-------|------|--------|-------------|
| `status` | String | `red`, `blue`, `green` | Current recognition state |
| `person_identity` | String | `severin`, `no_person`, other names | Recognized person name |
| `timestamp_sec` | Integer | Unix seconds | ROS timestamp seconds |
| `timestamp_nanosec` | Integer | 0-999999999 | ROS timestamp nanoseconds |
| `confidence` | Float | 0.0-1.0 | Recognition confidence (0.95 typical) |
| `duration_seconds` | Float | 0.0+ | Seconds in current state |
| `is_loss_state` | Boolean | true/false | True if in BLUE (loss) state |
| `audio_event` | String | `recognition`, `loss`, `none` | Which alert just triggered |

### 4.3 Example Messages

**RED State (Recognition):**
```json
{
  "status": "red",
  "person_identity": "severin",
  "timestamp_sec": 1765212914,
  "timestamp_nanosec": 949382424,
  "confidence": 0.95,
  "duration_seconds": 12.5,
  "is_loss_state": false,
  "audio_event": "none"
}
```

**BLUE State (Loss):**
```json
{
  "status": "blue",
  "person_identity": "no_person",
  "timestamp_sec": 1765212935,
  "timestamp_nanosec": 569361772,
  "confidence": 0.0,
  "duration_seconds": 8.3,
  "is_loss_state": true,
  "audio_event": "loss"
}
```

**GREEN State (Unknown Person):**
```json
{
  "status": "green",
  "person_identity": "alice",
  "timestamp_sec": 1765212920,
  "timestamp_nanosec": 123456789,
  "confidence": 0.88,
  "duration_seconds": 5.1,
  "is_loss_state": false,
  "audio_event": "none"
}
```

---

## 7. System Components

### 5.1 Core Node: audio_notification_node

**File:** `ros2_ws/src/r2d2_audio/r2d2_audio/audio_notification_node.py`

**Responsibility:** Central state machine for person recognition

**Key Methods:**

| Method | Purpose |
|--------|---------|
| `person_callback()` | Processes incoming person_id messages, manages state transitions |
| `check_loss_state()` | Timer callback (500ms interval) for loss detection and confirmation |
| `_publish_status()` | Creates and publishes JSON status messages |
| `_trigger_recognition_alert()` | Plays "Hello!" beep with cooldown enforcement |
| `_trigger_loss_alert()` | Plays "Lost you!" beep with cooldown enforcement |

**Implementation Details:**

- **State Variables:**
  ```python
  is_currently_recognized = False           # RED vs BLUE state
  loss_jitter_exceeded_time = None          # When 5s jitter exceeded
  last_recognition_alert_time = 0           # For 2s cooldown
  last_loss_alert_time = 0                  # For 2s cooldown
  ```

- **Timer Callbacks:**
  - Loss check: 500ms interval (lines ~120-150)
  - Status publishing: 100ms interval (lines ~160-180)

- **Audio Playback:**
  ```python
  # Plays MP3 using ffplay with volume control
  ffplay -nodisp -autoexit -af "volume={volume}" /path/to/file.mp3
  ```

### 5.2 Visual Feedback Node: status_led_node

**File:** `ros2_ws/src/r2d2_audio/r2d2_audio/status_led_node.py`

**Responsibility:** RGB LED feedback controller

**GPIO Pins:**
```python
RED_PIN = 17      # GPIO17 → Red LED
GREEN_PIN = 27    # GPIO27 → Green LED  
BLUE_PIN = 22     # GPIO22 → Blue LED
```

**Behavior:**

| Status | Red | Green | Blue | Visual |
|--------|-----|-------|------|--------|
| RED (recognized) | ON | OFF | OFF | 🔴 Solid Red |
| BLUE (lost) | OFF | OFF | ON | 🔵 Solid Blue |
| GREEN (unknown) | OFF | ON | OFF | 🟢 Solid Green |

**Features:**
- Auto-detection of GPIO availability
- Fallback to simulation mode if GPIO not available
- Real-time synchronization with status messages

### 5.3 Event Logging Node: database_logger_node

**File:** `ros2_ws/src/r2d2_audio/r2d2_audio/database_logger_node.py`

**Responsibility:** Event logging for conversation context

**Current Implementation:**
- Console logging with JSON parsing
- Tracks state transitions with timestamps

**Future Enhancement:**
- SQLite database integration
- Conversation tagging with person_identity
- Query interface for conversation history

**Database Schema (Planned):**
```sql
CREATE TABLE person_recognition_events (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  timestamp INTEGER,
  person_identity TEXT,
  status TEXT,           -- red, blue, green
  confidence REAL,
  duration_seconds REAL,
  audio_event TEXT,      -- recognition, loss, none
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

---

## 8. ROS 2 Integration

### 6.1 Topics

**Subscribed:**

| Topic | Type | Source | Rate | Description |
|-------|------|--------|------|-------------|
| `/r2d2/perception/person_id` | `std_msgs/String` | Face Recognition Service | ~10 Hz | Current recognized person name |

**Published:**

| Topic | Type | Rate | Subscribers | Description |
|-------|------|------|-------------|-------------|
| `/r2d2/audio/person_status` | `std_msgs/String` (JSON) | ~10 Hz | LED Node, Logger Node, Future modules | Real-time status with confidence |
| `/r2d2/audio/notification_event` | `std_msgs/String` | Event-based | Monitoring | Alert events for debugging |

### 6.2 Parameters

All parameters are ROS 2 parameters, configurable at runtime:

```bash
# View current parameters
ros2 param list /audio_notification_node

# Get a specific parameter
ros2 param get /audio_notification_node audio_volume

# Set a parameter (temporary, until restart)
ros2 param set /audio_notification_node audio_volume 0.5

# Set permanently (edit service file)
sudo nano /etc/systemd/system/r2d2-audio-notification.service
# Then: sudo systemctl daemon-reload && sudo systemctl restart r2d2-audio-notification.service
```

**All Parameters:**

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `target_person` | String | `severin` | any name | Person to recognize |
| `audio_volume` | Float | `0.05` | 0.0-1.0 | Audio playback volume (0-100%) |
| `jitter_tolerance_seconds` | Float | `5.0` | 1.0-10.0 | Brief gap tolerance (before loss detection) |
| `loss_confirmation_seconds` | Float | `15.0` | 5.0-30.0 | Confirmation window (after jitter) |
| `cooldown_seconds` | Float | `2.0` | 1.0-5.0 | Min between same alert type |
| `recognition_cooldown_after_loss_seconds` | Float | `5.0` | 3.0-10.0 | **NEW:** Quiet period after loss alert (prevents nervous rapid beeping) |
| `recognition_audio_file` | String | `Voicy_R2-D2 - 2.mp3` | filename | Audio file for "Hello!" |
| `loss_audio_file` | String | `Voicy_R2-D2 - 5.mp3` | filename | Audio file for "Lost you!" |
| `enabled` | Boolean | `true` | true/false | Enable/disable system |

### 6.3 Launch Configuration

**File:** `ros2_ws/src/r2d2_audio/launch/audio_notification.launch.py`

```python
# Launch audio notification system with custom parameters
ros2 launch r2d2_audio audio_notification.launch.py \
  audio_volume:=0.4 \
  loss_confirmation_seconds:=20.0
```

---

## 9. Production Deployment

### 7.1 Systemd Service Setup

The system runs as a background service that auto-starts on boot.

**Service File:** `/etc/systemd/system/r2d2-audio-notification.service`

**Features:**
- ✅ Auto-start on Jetson boot
- ✅ Auto-restart on failure (with 5s delay)
- ✅ Automatic restart limit (max 3 consecutive failures)
- ✅ Full environment sourcing
- ✅ Proper logging to systemd journal

**Status Check:**
```bash
# Check if service is running
sudo systemctl status r2d2-audio-notification.service

# View recent logs
journalctl -u r2d2-audio-notification.service -n 50

# Follow logs in real-time
journalctl -u r2d2-audio-notification.service -f
```

**Manual Control:**
```bash
# Start service
sudo systemctl start r2d2-audio-notification.service

# Stop service
sudo systemctl stop r2d2-audio-notification.service

# Restart service
sudo systemctl restart r2d2-audio-notification.service

# Enable auto-start on boot
sudo systemctl enable r2d2-audio-notification.service

# Disable auto-start on boot
sudo systemctl disable r2d2-audio-notification.service
```

### 7.2 Service Startup Script

**File:** `/home/severin/dev/r2d2/start_audio_notification.sh`

**Purpose:** Ensures proper environment setup before launching the node

**Environment Order (CRITICAL):**
```bash
source ~/depthai_env/bin/activate      # DepthAI first (OpenBLAS)
source ~/.bashrc                        # Then bash configuration
source ~/dev/r2d2/ros2_ws/install/setup.bash  # Finally ROS 2
export OPENBLAS_CORETYPE=ARMV8          # ARM CPU optimization
```

**Critical Variables:**
- `OPENBLAS_CORETYPE=ARMV8` - Prevents "Illegal instruction" errors on ARM64
- `ROS_DOMAIN_ID` - Isolation from other ROS 2 systems (optional)

### 7.3 Post-Reboot Verification

After rebooting the Jetson:

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
ros2 topic pub --once /r2d2/perception/person_id std_msgs/String "{data: severin}"
# Should play "Hello!" beep

# 5. Check LED feedback (if available)
# Should see RED LED light up

# 6. Verify logs for errors
journalctl -u r2d2-audio-notification.service -n 20 | grep -i error
# Should show no errors
```

---

## 10. Testing & Validation

### 8.1 Manual Testing

**Test 1: Recognition Alert**
```bash
# Terminal 1: Monitor status
ros2 topic echo /r2d2/audio/person_status --no-arr

# Terminal 2: Simulate recognition
ros2 topic pub --once /r2d2/perception/person_id std_msgs/String "{data: severin}"

# Expected:
# - "Hello!" plays (30% volume)
# - Status changes to: {"status": "red", "person_identity": "severin", ...}
# - LED shows RED (if enabled)
```

**Test 2: Loss Detection**
```bash
# Terminal 1: Monitor status
ros2 topic echo /r2d2/audio/person_status --no-arr

# Terminal 2: Send recognition, then wait 20+ seconds
ros2 topic pub --once /r2d2/perception/person_id std_msgs/String "{data: severin}"
# Wait 20 seconds...

# Expected:
# - "Hello!" plays after 2 seconds
# - After ~20 seconds: "Oh, I lost you!" plays
# - Status changes to: {"status": "blue", "person_identity": "no_person", ...}
# - LED shows BLUE (if enabled)
```

**Test 3: Jitter Tolerance**
```bash
# Terminal 1: Monitor status
ros2 topic echo /r2d2/audio/person_status --no-arr

# Terminal 2: Send recognition
ros2 topic pub --once /r2d2/perception/person_id std_msgs/String "{data: severin}"
# Wait 2 seconds for beep...

# Terminal 3: Send "unknown" (simulating interruption)
ros2 topic pub --once /r2d2/perception/person_id std_msgs/String "{data: unknown}"
# Wait 3 seconds...

# Terminal 2: Send "severin" again
ros2 topic pub --once /r2d2/perception/person_id std_msgs/String "{data: severin}"

# Expected:
# - NO second beep (jitter ignored)
# - Status stays RED throughout
# - LED stays RED (no flicker)
```

**Test 4: Volume Parameter**
```bash
# Check current volume
ros2 param get /audio_notification_node audio_volume
# Should show: 0.3

# Set to higher volume (50%)
ros2 param set /audio_notification_node audio_volume 0.5

# Test audio
ros2 topic pub --once /r2d2/perception/person_id std_msgs/String "{data: severin}"

# Expected:
# - Audio plays noticeably louder at 50% volume
```

### 8.2 Validation Checklist

Before declaring the system "working":

- [ ] Service Status: `sudo systemctl status r2d2-audio-notification.service` → `active (running)`
- [ ] Topics: `ros2 topic list | grep r2d2/audio` → Both topics visible
- [ ] Recognition: Publish person_id → "Hello!" plays + status = RED
- [ ] Loss: Wait 20s → "Oh, I lost you!" plays + status = BLUE
- [ ] Jitter: Brief interruption ignored + no extra beeps
- [ ] LED: Status changes visible on RGB LED (if GPIO available)
- [ ] Logs: `journalctl -u r2d2-audio-notification.service -n 10` → No errors
- [ ] Parameters: All 8 parameters readable via `ros2 param list`
- [ ] Post-Reboot: Service auto-starts and works after full system reboot

---

## 11. Common Issues & Troubleshooting

### Issue 1: No Audio Heard After Reboot

**Symptom:** Service running but no audio plays

**Diagnosis:**
```bash
# Check service is running
sudo systemctl status r2d2-audio-notification.service

# Check ALSA configuration
cat ~/.asoundrc | grep -i "hw:1,0"

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

### Issue 2: Service Crashes on Startup

**Symptom:** Service fails with `NameError` or `ModuleNotFoundError`

**Diagnosis:**
```bash
# Check service logs
journalctl -u r2d2-audio-notification.service -n 50

# Look for: NameError, ModuleNotFoundError, ImportError
```

**Solutions:**
1. Verify environment sourcing in startup script
2. Ensure Python packages installed: `pip list | grep rclpy`
3. Rebuild package: `colcon build --packages-select r2d2_audio`
4. Restart service: `sudo systemctl restart r2d2-audio-notification.service`

### Issue 3: Loss Alert Fires Too Early/Too Late

**Symptom:** "Oh, I lost you!" plays at wrong time

**Diagnosis:**
```bash
# Monitor timing in logs
journalctl -u r2d2-audio-notification.service -f | grep -i "loss\|confirmation"

# Check parameters
ros2 param get /audio_notification_node jitter_tolerance_seconds
ros2 param get /audio_notification_node loss_confirmation_seconds
```

**Expected Timing:** ~20 seconds total (5s jitter + 15s confirmation)

**Solutions:**
1. Adjust jitter tolerance: `ros2 param set /audio_notification_node jitter_tolerance_seconds 3.0`
2. Adjust confirmation window: `ros2 param set /audio_notification_node loss_confirmation_seconds 10.0`
3. Restart service to apply: `sudo systemctl restart r2d2-audio-notification.service`

### Issue 4: LED Not Changing Color

**Symptom:** Service running but LED stays same color

**Diagnosis:**
```bash
# Check if status_led_node is running
ros2 node list | grep status_led

# Monitor status messages
ros2 topic echo /r2d2/audio/person_status --no-arr -n 5

# Check GPIO access
ls -l /dev/gpiochip* 2>&1
```

**Solutions:**
1. Launch LED node: `python3 -m r2d2_audio.status_led_node`
2. Verify GPIO permissions: `sudo usermod -aG gpio $USER`
3. Reboot and retry: `sudo reboot`
4. Check GPIO pin connections on hardware

### Issue 5: Audio Plays Multiple Times (Not Anti-Spam)

**Symptom:** Same alert plays multiple times rapidly

**Diagnosis:**
```bash
# Check cooldown parameter
ros2 param get /audio_notification_node cooldown_seconds

# Monitor alerts in logs
journalctl -u r2d2-audio-notification.service -f | grep "Playing"
```

**Solutions:**
1. Increase cooldown: `ros2 param set /audio_notification_node cooldown_seconds 3.0`
2. Check for rapid person_id messages: `ros2 topic echo /r2d2/perception/person_id -n 10`
3. Verify face recognition is stable (check perception node)

---

## 12. Integration with Dialogue System (STT-LLM-TTS)

### 10.1 How Status Drives Dialogue Context

The person status system provides real-time context for the STT-LLM-TTS pipeline:

```
Audio Status System              Dialogue Pipeline
    ├─ RED (recognized)    ──→  STT context: "Speaking with Severin"
    │                            (High-confidence conversation mode)
    │
    ├─ BLUE (lost)         ──→  TTS output: "I've lost sight of you!"
    │                            (System acknowledges loss)
    │
    └─ GREEN (unknown)     ──→  STT context: "Unknown person present"
                                 (Possible side conversation)
```

### 10.2 JSON Status for Dialogue Integration

Subscribe to `/r2d2/audio/person_status` and parse JSON:

```python
import rclpy
from std_msgs.msg import String
import json

def status_callback(msg):
    status_data = json.loads(msg.data)
    
    if status_data['status'] == 'red':
        # Start dialogue with recognized person
        dialogue_context = {
            'person': status_data['person_identity'],
            'confidence': status_data['confidence'],
            'mode': 'active_conversation'
        }
        start_dialogue(dialogue_context)
        
    elif status_data['status'] == 'blue':
        # Loss acknowledged
        announce_loss()
        
    elif status_data['status'] == 'green':
        # Unknown person - optional interaction
        pause_dialogue()
```

### 10.3 Conversation Tagging with Database

The database_logger_node structure supports tagging conversations:

```python
# Future database implementation
conversation_tags = {
    'conversation_id': uuid.uuid4(),
    'person_identity': status['person_identity'],
    'status_timeline': [
        {'time': 0, 'status': 'red', 'duration': 15.3},
        {'time': 15.3, 'status': 'blue', 'duration': 8.2}
    ],
    'dialogue_context': '...'
}
```

---

## 13. Future Enhancements

### 11.1 Planned Features

- [ ] **SQLite Database:** Persistent storage of recognition events
- [ ] **Conversation Tagging:** Link conversations to person_identity + status timeline
- [ ] **Multi-Person Tracking:** Support for multiple known people simultaneously
- [ ] **Confidence Threshold:** Adjust beep trigger based on confidence score
- [ ] **Audio Customization:** Different audio files for different people
- [ ] **LED Patterns:** Blinking or pulsing patterns for special states
- [ ] **Voice Feedback:** TTS confirmation ("Hello Severin!" instead of beep)

### 11.2 Enhancement Checklist

Before implementing new features:

1. [ ] Verify current system is production-stable
2. [ ] Document new requirements in 010_PERSON_RECOGNITION_STATUS.md
3. [ ] Implement changes following coding standards
4. [ ] Add comprehensive logging
5. [ ] Test with checklist from Section 8.2
6. [ ] Update relevant documentation (000-060)
7. [ ] Commit with descriptive message
8. [ ] Deploy and verify on production Jetson

---

## 14. Parameter Reference (Quick Lookup)

```bash
# List all parameters for audio_notification_node
ros2 param list /audio_notification_node

# Example output:
/audio_notification_node.audio_volume: 0.05
/audio_notification_node.cooldown_seconds: 2.0
/audio_notification_node.enabled: True
/audio_notification_node.jitter_tolerance_seconds: 5.0
/audio_notification_node.loss_audio_file: 'Voicy_R2-D2 - 5.mp3'
/audio_notification_node.loss_confirmation_seconds: 15.0
/audio_notification_node.recognition_audio_file: 'Voicy_R2-D2 - 2.mp3'
/audio_notification_node.recognition_cooldown_after_loss_seconds: 5.0
/audio_notification_node.target_person: 'severin'
```

**Quick Change Templates:**

```bash
# Increase volume (hearing issues)
ros2 param set /audio_notification_node audio_volume 0.1

# Faster loss detection (5s jitter + 10s confirmation = 15s total)
ros2 param set /audio_notification_node loss_confirmation_seconds 10.0

# Slower loss detection (5s jitter + 20s confirmation = 25s total)
ros2 param set /audio_notification_node loss_confirmation_seconds 20.0

# Change quiet period after loss alert (calm down rapid transitions)
ros2 param set /audio_notification_node recognition_cooldown_after_loss_seconds 3.0

# Different target person
ros2 param set /audio_notification_node target_person "alice"

# Disable system temporarily
ros2 param set /audio_notification_node enabled false
```

---

## 15. Related Documentation Map

```
System Overview
└─ 001_ARCHITECTURE_OVERVIEW.md
   └─ Full system architecture (camera, perception, audio, control)

Camera Setup
└─ 011_CAMERA_SETUP_DOCUMENTATION.md
   └─ OAK-D Lite camera + DepthAI SDK + ROS 2 integration

Speaker Hardware (Physical Setup)
└─ 012_SPEAKER_AUDIO_SETUP_DOCUMENTATION.md
   ├─ ALSA configuration
   ├─ PAM8403 amplifier wiring
   └─ Audio hardware testing

↓
THIS DOCUMENT - 010_PERSON_RECOGNITION_STATUS.md (Comprehensive Master Reference)
   ├─ Complete state machine documentation
   ├─ JSON message format
   ├─ RGB LED feedback
   ├─ Testing procedures
   ├─ Troubleshooting guide
   └─ Dialogue system integration

Agent Instructions
└─ 000_INTERNAL_AGENT_NOTES.md
   ├─ Development guidelines
   ├─ Git workflow
   ├─ Build procedures
   └─ Debugging tips
```

---

## 16. Quick Reference Commands

### Check System Status
```bash
# Is service running?
sudo systemctl status r2d2-audio-notification.service

# What's the current status?
ros2 topic echo /r2d2/audio/person_status --no-arr -n 1

# Are there any errors?
journalctl -u r2d2-audio-notification.service -n 20 | grep -i error
```

### Test Recognition
```bash
# Send recognition message
ros2 topic pub --once /r2d2/perception/person_id std_msgs/String "{data: severin}"

# Monitor response
ros2 topic echo /r2d2/audio/person_status --no-arr -n 3
```

### Adjust Parameters
```bash
# Increase volume
ros2 param set /audio_notification_node audio_volume 0.5

# Faster loss detection
ros2 param set /audio_notification_node loss_confirmation_seconds 10.0

# Apply changes (restart service)
sudo systemctl restart r2d2-audio-notification.service
```

### View Logs
```bash
# Last 20 lines
journalctl -u r2d2-audio-notification.service -n 20

# Follow in real-time
journalctl -u r2d2-audio-notification.service -f

# Just errors
journalctl -u r2d2-audio-notification.service -n 50 | grep ERROR
```

---

## Appendix: System Specifications

### Hardware
- **Platform:** NVIDIA Jetson AGX Orin 64GB
- **OS:** Ubuntu 22.04 Jammy (Jetson-specific)
- **ROS 2:** Humble
- **Audio Output:** J511 Pin 9 (HPO_L) via I2S → PAM8403 Amplifier → 8Ω Speaker
- **Camera:** OAK-D Lite AF (Face Recognition Source)
- **LED Controller:** GPIO pins (17=RED, 27=GREEN, 22=BLUE)

### Software Stack
```
ROS 2 Humble
├── r2d2_audio (Audio notifications + LED + logging)
├── r2d2_perception (Face recognition)
└── r2d2_bringup (Camera + launch infrastructure)

Linux Audio Stack
├── ALSA (Audio driver)
├── ffplay (Audio player)
└── PAM8403 (Amplifier driver)

Python 3.10.6
├── rclpy (ROS 2 client)
├── OpenCV (Face detection)
├── NumPy (Numerical computing)
└── RPi.GPIO (GPIO control for LEDs)
```

### Performance Baselines
| Metric | Expected | Issue When |
|--------|----------|-----------|
| Status publish rate | 10 Hz | <5 Hz (CPU issue) |
| Audio latency | <2s start | >5s (performance) |
| LED response | <100ms | >500ms (GPIO issue) |
| Memory usage | ~60 MB | >150 MB (leak) |
| CPU usage | 5-10% | >30% (sustained) |

---

**Document Version:** 2.0 (Comprehensive Merged Documentation)  
**Last Updated:** December 8, 2025  
**Status:** ✅ Production Ready  
**Next Review:** After major system changes or user feedback

---

## Appendix: Complete Implementation Workflow

### Step-by-Step Setup Guide

**1. Camera Setup** (See [`011_CAMERA_SETUP_DOCUMENTATION.md`](011_CAMERA_SETUP_DOCUMENTATION.md))
```bash
# Verify camera is working
ros2 launch r2d2_camera camera.launch.py
# In another terminal:
ros2 topic echo /oak/rgb/image_raw
```

**2. Speaker Hardware Setup** (See [`012_SPEAKER_AUDIO_SETUP_DOCUMENTATION.md`](012_SPEAKER_AUDIO_SETUP_DOCUMENTATION.md))
```bash
# Verify ALSA configuration
aplay -l  # Should show Card 1 (APE)
# Test audio
aplay -D hw:1,0 /path/to/test.wav
```

**3. Train Face Recognition Model**
```bash
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8

# Capture training data
python3 1_capture_training_data.py

# Train model
python3 2_train_recognizer.py

# Verify model exists
ls -la ~/dev/r2d2/data/face_recognition/models/severin_lbph.xml
```

**4. Build ROS 2 Packages**
```bash
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_perception r2d2_audio
source install/setup.bash
```

**5. Launch Complete System**

**Terminal 1 - Camera + Face Recognition:**
```bash
cd ~/dev/r2d2/ros2_ws
source ~/depthai_env/bin/activate
source ~/.bashrc
source install/setup.bash

ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true
```

**Terminal 2 - Audio Notifications:**
```bash
cd ~/dev/r2d2/ros2_ws
source install/setup.bash

ros2 launch r2d2_audio audio_notification.launch.py
```

**Terminal 3 - Monitor System:**
```bash
# Watch person recognition
ros2 topic echo /r2d2/perception/person_id

# Watch status messages
ros2 topic echo /r2d2/audio/person_status --no-arr

# Check service status (if using systemd)
sudo systemctl status r2d2-audio-notification.service
```

**6. Verify System is Working**

- ✅ Camera publishing frames: `ros2 topic hz /oak/rgb/image_raw` → ~30 Hz
- ✅ Face recognition working: `ros2 topic echo /r2d2/perception/person_id` → Shows "severin" or "unknown"
- ✅ Audio alerts playing: You hear "Hello!" when face is recognized
- ✅ Status messages: `ros2 topic echo /r2d2/audio/person_status` → Shows JSON with status
- ✅ LED feedback: RGB LED changes color (if GPIO available)

**Expected Behavior:**
- When your face enters frame → 🔊 "Hello!" plays → Status = RED
- While continuously recognized → Silent (no beeps)
- When you leave for >20 seconds → 🔔 "Oh, I lost you!" plays → Status = BLUE
- When you return → 🔊 "Hello!" plays → Status = RED

---

**This comprehensive document merges content from:**
- `040_FACE_RECOGNITION_COMPLETE.md` - Face recognition system
- `060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md` - Audio ROS 2 integration
- `070_PERSON_RECOGNITION_STATUS.md` - State machine and status system

**For hardware setup details, see:**
- `011_CAMERA_SETUP_DOCUMENTATION.md` - Camera hardware and software
- `012_SPEAKER_AUDIO_SETUP_DOCUMENTATION.md` - Speaker and amplifier hardware
