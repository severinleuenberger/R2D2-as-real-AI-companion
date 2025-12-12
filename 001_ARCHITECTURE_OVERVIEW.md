# R2D2 System Architecture Overview
**Date:** December 9, 2025 (Comprehensive Update)  
**Platform:** NVIDIA Jetson AGX Orin 64GB + ROS 2 Humble  
**Phase:** 1 - Perception, Face Recognition & Audio Notifications (Complete)

---

## Executive Summary

The R2D2 system is a modular ROS 2-based pipeline that captures video from an OAK-D Lite camera, processes frames in real-time, detects human faces, recognizes specific individuals, and provides audio/visual feedback through a sophisticated state machine. The system prioritizes efficiency (15-25% CPU usage) and extensibility (easy to add new components).

**Current Processing Chain:**
```
OAK-D Lite → r2d2_camera node → /oak/rgb/image_raw (30 Hz)
             ↓
             r2d2_perception node:
             ├─ Downscale (1920×1080 → 640×360)
             ├─ Brightness computation → /r2d2/perception/brightness (13 Hz)
             ├─ Haar Cascade face detection → /r2d2/perception/face_count (13 Hz)
             └─ LBPH face recognition → /r2d2/perception/person_id (6.5 Hz, optional)
             ↓
             r2d2_audio package:
             ├─ audio_notification_node: State machine (RED/BLUE/GREEN)
             ├─ status_led_node: RGB LED visual feedback
             ├─ database_logger_node: Event logging
             └─ audio_beep_node: Simple beep demo
             ↓
             Downstream consumers (Phase 2: speech, Phase 3: navigation)
```

---

## 1. System-Level Architecture

### 1.1 Hardware Components

```
┌─────────────────────────────────────────────────────────────────┐
│                     PHYSICAL HARDWARE                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────────┐              ┌──────────────────────┐   │
│  │  OAK-D Lite      │              │  NVIDIA Jetson       │   │
│  │  Camera          │──USB 3.0────▶│  AGX Orin 64GB       │   │
│  │                  │              │                      │   │
│  │ • RGB Sensor     │              │ • 12-core ARM CPU    │   │
│  │   1920×1080@30Hz │              │ • 504-core GPU       │   │
│  │ • Stereo Pair    │              │ • 64GB LPDDR5X RAM   │   │
│  │ • Depth Engine   │              │ • 100W TDP (variable)│   │
│  │ • Auto Focus     │              │ • Ubuntu 22.04       │   │
│  │                  │              │ • JetPack 6.x        │   │
│  │ Serial:          │              │ • ROS 2 Humble       │   │
│  │ 19443010E1D30C7E00               │                      │   │
│  └──────────────────┘              └──────────────────────┘   │
│                                                                 │
│  Connection: USB 3.0 (direct to Jetson, not through hub)      │
│  Power: Bus-powered from Jetson (500mA @ 5V)                  │
│  Status: Fully detected and operational                        │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

**Key Hardware Specs:**
- **Camera:** OAK-D Lite Auto Focus
  - RGB: 1280×1080 actual, 1920×1080 reported (includes padding)
  - Depth: Stereo depth with OV9782 pair
  - FPS: 30 (native camera rate)
  - Interface: USB 3.0 with USB-C connector
  - Processor: Intel Movidius MyriadX

- **Jetson AGX Orin:** NVIDIA's flagship edge AI platform
  - CPU: 12-core ARM Cortex-A78 @ 2.4 GHz
  - GPU: 504-core NVIDIA (can accelerate deep learning)
  - RAM: 64 GB LPDDR5X
  - Storage: Internal eMMC (~32GB usable after JetPack)
  - Thermal: Passive/active cooling (can sustain 100W)

### 1.2 Hardware Fixed Constants Reference

**Critical system constants (do not change without documentation update):**

| Constant | Value | Location/Usage | Notes |
|----------|-------|---------------|-------|
| **Platform** | NVIDIA Jetson AGX Orin 64GB | Hardware | ARM64 architecture (not x86) |
| **OS** | Ubuntu 22.04 Jammy | System | Jetson-specific L4T image |
| **ROS 2 Version** | Humble | Software stack | Required version for compatibility |
| **Python Version** | 3.10.6 | Runtime | System or venv |
| **Camera Model** | OAK-D Lite Auto Focus | Hardware | Luxonis depth camera |
| **Camera Serial** | 19443010E1D30C7E00 | Hardware identifier | Specific unit identifier |
| **Camera Resolution** | 1920×1080 | Camera config | Reported resolution (includes padding) |
| **Camera FPS** | 30 Hz | Camera config | Native camera rate |
| **Project Root** | `~/dev/r2d2` | File system | NOT /opt, NOT /home/user |
| **ROS 2 Workspace** | `~/dev/r2d2/ros2_ws` | File system | Standard ROS 2 workspace location |
| **Audio Output Pin** | J511 Pin 9 (HPO_L) | Hardware | I2S interface for audio output |
| **Audio Device** | `hw:1,0` | ALSA config | Audio hardware device identifier |
| **LED GPIO Pins** | 17 (RED), 27 (GREEN), 22 (BLUE) | Hardware | GPIO pin assignments for RGB LED |
| **Power Button GPIO** | Pin 32 (40-pin header) | Hardware | Shutdown control |
| **Boot/Wake Pin** | J42 Pin 4 (POWER) | Hardware | Boot/wake control |
| **OPENBLAS_CORETYPE** | ARMV8 | Environment | Critical for ARM64 (prevents "Illegal instruction") |
| **Face Recognition Model Path** | `~/dev/r2d2/data/face_recognition/models/severin_lbph.xml` | File system | Default model location |
| **Audio Assets Path** | `ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/` | File system | MP3 audio files location |

**Critical Environment Variables:**
- `OPENBLAS_CORETYPE=ARMV8` - **MUST** be set before ROS 2 commands (prevents crashes on ARM64)
- `ROS_DOMAIN_ID` - Optional, for ROS 2 network isolation

**For detailed hardware setup and troubleshooting, see:** [`000_INTERNAL_AGENT_NOTES.md`](000_INTERNAL_AGENT_NOTES.md)

---

### 1.3 Software Stack (Layered)

```
┌─────────────────────────────────────────────────────────────────┐
│                      APPLICATION LAYER                         │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  r2d2_perception (Python ROS 2 Node)                    │  │
│  │  └─ image_listener.py: 354 lines                        │  │
│  │     • Brightness computation                            │  │
│  │     • Haar Cascade face detection                       │  │
│  │     • LBPH face recognition (optional)                  │  │
│  │     • Topic publishing (6 channels)                     │  │
│  └──────────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  r2d2_camera (Python ROS 2 Node)                        │  │
│  │  └─ OAK-D camera driver                                 │  │
│  │     • Initializes DepthAI pipeline                      │  │
│  │     • Streams RGB frames at 30 FPS                      │  │
│  │     • Publishes /oak/rgb/image_raw                      │  │
│  └──────────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  r2d2_hello (ROS 2 Nodes) [Basic Infrastructure]       │  │
│  │  ├─ heartbeat_node: Publish /r2d2/heartbeat (1 Hz)     │  │
│  │  └─ beep_node: Alive signal demo                       │  │
│  └──────────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  r2d2_bringup (Launch Files)                           │  │
│  │  └─ r2d2_camera_perception.launch.py                   │  │
│  │     • Orchestrates camera + perception nodes           │  │
│  │     • Passes parameters to perception node             │  │
│  │     • Enables/disables face recognition                │  │
│  └──────────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  r2d2_audio (ROS 2 Package) [Audio & Status System]    │  │
│  │  ├─ audio_notification_node: Person recognition alerts │  │
│  │  │  • Subscribes to /r2d2/perception/person_id         │  │
│  │  │  • 3-state machine (RED/BLUE/GREEN)                 │  │
│  │  │  • MP3 audio alerts (recognition/loss)               │  │
│  │  │  • Publishes /r2d2/audio/person_status (JSON)       │  │
│  │  ├─ status_led_node: RGB LED control (GPIO)             │  │
│  │  │  • Visual feedback for recognition state             │  │
│  │  │  • RED=recognized, BLUE=lost, GREEN=unknown         │  │
│  │  │  • GPIO pins 17, 27, 22                              │  │
│  │  ├─ database_logger_node: Event logging                │  │
│  │  │  • Tracks state transitions                          │  │
│  │  │  • Future: SQLite database integration              │  │
│  │  └─ audio_beep_node: Simple beep demo                  │  │
│  └──────────────────────────────────────────────────────────┘  │
├─────────────────────────────────────────────────────────────────┤
│                    ROS 2 HUMBLE LAYER                          │
│  • rclpy (Python client library)                              │
│  • sensor_msgs (Image message type)                           │
│  • std_msgs (Float32, Int32, String, Bool types)             │
│  • cv_bridge (ROS Image ↔ OpenCV conversion)                 │
│  • roslaunch (launch file system)                            │
├─────────────────────────────────────────────────────────────────┤
│                  COMPUTER VISION & PROCESSING                  │
│  • OpenCV (face detection, image manipulation)                │
│  • OpenCV contrib (LBPH face recognizer)                      │
│  • NumPy (numerical computing)                                │
│  • cv_bridge (image format conversion)                        │
├─────────────────────────────────────────────────────────────────┤
│                    HARDWARE ABSTRACTION                        │
│  • DepthAI SDK 2.31.0.0 (OAK-D camera interface)             │
│  • DepthAI Python bindings                                    │
│  • USB driver (kernel-level USB 3.0 support)                │
├─────────────────────────────────────────────────────────────────┤
│                    OPERATING SYSTEM                            │
│  • Ubuntu 22.04 Jammy (ARM64 aarch64)                        │
│  • Linux kernel 5.10.192-tegra (custom Jetson kernel)        │
│  • JetPack 6.x (NVIDIA system image)                         │
│  • CUDA 12.x + cuDNN (GPU acceleration, optional)            │
├─────────────────────────────────────────────────────────────────┤
│                  HARDWARE (Jetson AGX Orin)                    │
│  • 12-core ARM CPU + 504-core GPU                            │
│  • 64 GB LPDDR5X memory                                      │
│  • Internal eMMC storage                                      │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Data Flow Architecture

### 2.1 Complete Message Flow (Simplified)

```
OAK-D Camera (30 Hz)
      ↓
r2d2_camera node
      ↓
/oak/rgb/image_raw (30 Hz)
      ↓
r2d2_perception node
      ├─ Downscale + Grayscale
      ├─ Brightness calculation → /r2d2/perception/brightness (13 Hz)
      ├─ Face detection → /r2d2/perception/face_count (13 Hz)
      └─ Face recognition (optional) → /r2d2/perception/person_id (6.5 Hz)
      ↓
r2d2_audio package
      ├─ audio_notification_node: State machine → /r2d2/audio/person_status (10 Hz)
      ├─ status_led_node: LED control (GPIO)
      └─ database_logger_node: Event logging
      ↓
Downstream consumers (Phase 2-4)
```

### 2.2 Component Interaction Diagram

**Complete System Flow with Timing and Dependencies:**

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    R2D2 SYSTEM ARCHITECTURE                             │
│                    Component Interaction & Data Flow                   │
└─────────────────────────────────────────────────────────────────────────┘

HARDWARE LAYER:
┌──────────────┐
│  OAK-D Lite  │ 30 FPS RGB frames (1920×1080)
│   Camera     │ USB 3.0 connection
└──────┬───────┘
       │
       ↓ [30 Hz raw frames]
       
ROS 2 NODE LAYER:
┌─────────────────────┐
│  r2d2_camera        │ Node: camera_node
│  (camera driver)    │ CPU: 2-3% (one core)
│                     │ Memory: ~50 MB
│  Publishes:         │ Frequency: 30 Hz
│  /oak/rgb/image_raw │ Message: sensor_msgs/Image
└──────┬──────────────┘
       │
       ↓ [30 Hz image stream]
       
┌─────────────────────┐
│  r2d2_perception    │ Node: image_listener
│  (image processing) │ CPU: 8-15% (one core)
│                     │ Memory: ~200 MB
│  Processing:        │ Frequency: 13 Hz (downscaled from 30 Hz)
│  • Downscale        │
│  • Grayscale        │
│  • Brightness       │
│  • Face detection   │
│  • Face recognition │ (optional, 6.5 Hz when enabled)
│                     │
│  Publishes:         │
│  • /r2d2/perception/brightness (13 Hz)      │
│  • /r2d2/perception/face_count (13 Hz)      │
│  • /r2d2/perception/person_id (6.5 Hz*)     │
│  • /r2d2/perception/face_confidence (6.5 Hz*)│
│  • /r2d2/perception/is_target_person (6.5 Hz*)     │
└──────┬──────────────┘
       │
       ↓ [6.5 Hz person_id stream]
       
┌─────────────────────┐
│  r2d2_audio          │ Package: r2d2_audio
│  (audio & status)    │
│                      │
│  ┌─────────────────────────────────────┐
│  │ audio_notification_node             │
│  │ CPU: 2-4% | Memory: ~50 MB          │
│  │ Frequency: 10 Hz (status publishing) │
│  │                                     │
│  │ Subscribes:                         │
│  │ • /r2d2/perception/person_id        │
│  │                                     │
│  │ State Machine:                      │
│  │ • RED (recognized)                 │
│  │ • BLUE (lost/idle)                 │
│  │ • GREEN (unknown person)            │
│  │                                     │
│  │ Publishes:                          │
│  │ • /r2d2/audio/person_status (10 Hz)│
│  │ • /r2d2/audio/notification_event   │
│  │                                     │
│  │ Actions:                            │
│  │ • MP3 audio alerts (ffplay)         │
│  └─────────────────────────────────────┘
│           │
│           ↓ [10 Hz status JSON]
│           
│  ┌─────────────────────────────────────┐
│  │ status_led_node                     │
│  │ CPU: <0.1% | Memory: ~20 MB         │
│  │ Frequency: 10 Hz (LED updates)      │
│  │                                     │
│  │ Subscribes:                         │
│  │ • /r2d2/audio/person_status         │
│  │                                     │
│  │ Controls:                           │
│  │ • GPIO Pin 17 (RED LED)            │
│  │ • GPIO Pin 27 (GREEN LED)          │
│  │ • GPIO Pin 22 (BLUE LED)           │
│  └─────────────────────────────────────┘
│           │
│           ↓ [10 Hz status JSON]
│           
│  ┌─────────────────────────────────────┐
│  │ database_logger_node                │
│  │ CPU: <0.1% | Memory: ~30 MB         │
│  │ Frequency: 10 Hz (event logging)     │
│  │                                     │
│  │ Subscribes:                         │
│  │ • /r2d2/audio/person_status         │
│  │                                     │
│  │ Logs:                               │
│  │ • State transitions                 │
│  │ • Recognition events                │
│  │ • (Future: SQLite database)         │
│  └─────────────────────────────────────┘
└─────────────────────────────────────────┘
       │
       ↓ [Status & events available]
       
FUTURE INTEGRATION POINTS (Phase 2-4):
┌─────────────────────┐
│  Phase 2: Speech     │ • Subscribe to /r2d2/audio/person_status
│  (STT-LLM-TTS)       │ • Subscribe to /r2d2/perception/person_id
│                     │ • Publish to /r2d2/cmd/* (commands)
└─────────────────────┘
┌─────────────────────┐
│  Phase 3: Navigation │ • Subscribe to /r2d2/perception/face_count
│  (SLAM, movement)    │ • Subscribe to /r2d2/cmd/* (commands)
│                     │ • Publish to /r2d2/cmd_vel
└─────────────────────┘

* Only published if enable_face_recognition=true
```

**Key Timing Relationships:**
- **Camera → Perception:** 30 Hz → 13 Hz (frame skipping for CPU efficiency)
- **Perception → Audio:** 6.5 Hz → 10 Hz (audio node publishes status faster than recognition rate)
- **Audio → LED/Logger:** 10 Hz → 10 Hz (synchronized updates)

---

### 2.3 ROS 2 Topic Reference

Complete list of all topics published:

```
TOPIC                                  TYPE                  FREQ   NOTES
─────────────────────────────────────────────────────────────────────────
PERCEPTION TOPICS:
/oak/rgb/image_raw                     sensor_msgs/Image     30 Hz  Raw camera
/r2d2/perception/brightness            std_msgs/Float32      13 Hz  Mean brightness
/r2d2/perception/face_count            std_msgs/Int32        13 Hz  Number of faces
/r2d2/perception/person_id             std_msgs/String       6.5 Hz* Person name
/r2d2/perception/face_confidence       std_msgs/Float32      6.5 Hz* Confidence score
/r2d2/perception/is_target_person            std_msgs/Bool         6.5 Hz* Target person present?

AUDIO & STATUS TOPICS:
/r2d2/audio/person_status              std_msgs/String       10 Hz  JSON status (RED/BLUE/GREEN)
/r2d2/audio/notification_event        std_msgs/String       Event  Recognition/loss events
/r2d2/audio/status                    std_msgs/String       Event  Audio system status
/r2d2/audio/beep_count                std_msgs/UInt32        Event  Beep counter
/r2d2/audio/last_frequency             std_msgs/Float32       Event  Last beep frequency

SYSTEM TOPICS:
/r2d2/heartbeat                        std_msgs/String       1 Hz   Health indicator

* Only published if enable_face_recognition=true
```

---

## 3. Node Architecture

### 3.1 Node Details

| Node | Package | Type | FPS In | FPS Out | CPU | Status |
|------|---------|------|--------|---------|-----|--------|
| **camera_node** | r2d2_camera | Sensor driver | N/A | 30 Hz | 2-3% | ✅ |
| **image_listener** | r2d2_perception | Computer vision | 30 Hz | 6 topics | 8-15% | ✅ |
| **heartbeat_node** | r2d2_hello | Health monitor | N/A | 1 Hz | <0.1% | ✅ |
| **audio_notification_node** | r2d2_audio | State machine | 6.5 Hz | 10 Hz | 2-4% | ✅ |
| **status_led_node** | r2d2_audio | GPIO control | 10 Hz | N/A | <0.1% | ✅ |
| **database_logger_node** | r2d2_audio | Event logging | 10 Hz | N/A | <0.1% | ✅ |
| **audio_beep_node** | r2d2_audio | Audio demo | N/A | Event | <0.1% | ✅ |

### 3.2 Launch Sequence

```
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py
        ↓
r2d2_camera_perception.launch.py loads
        ↓
   ┌────┴─────────────────────┐
   ↓                           ↓
camera.launch.py      perception.launch.py
   ↓                           ↓
camera_node started    image_listener started
(1-2 sec to ready)     (waits for /oak/rgb/image_raw)
   ↓                           ↓
   └────┬─────────────────────┘
        ↓
   System Ready (5-7 sec total)
   All topics publishing
   Ready for subscribers
```

---

## 4. Audio Notification System & State Machine

### 4.1 Person Recognition State Machine

The `r2d2_audio` package implements a sophisticated 3-state recognition system:

```
┌─────────────────────────────────────────────────────────────┐
│              PERSON RECOGNITION STATE MACHINE                │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  🔴 RED STATE (Recognized)                                   │
│     • Target person is currently visible                     │
│     • Audio: "Hello!" MP3 plays on transition               │
│     • LED: Solid RED (GPIO pin 17)                          │
│     • Status: Active engagement                             │
│     • Transitions: → BLUE (after loss confirmation)          │
│                    → GREEN (if unknown person appears)      │
│                                                              │
│  🔵 BLUE STATE (Lost/Idle)                                   │
│     • No target person visible                              │
│     • Audio: "Oh, I lost you!" MP3 plays on transition     │
│     • LED: Solid BLUE (GPIO pin 22)                         │
│     • Status: Idle, waiting for recognition                  │
│     • Timing: 5s jitter tolerance + 15s confirmation        │
│     • Transitions: → RED (when target person detected)      │
│                    → GREEN (if unknown person appears)       │
│                                                              │
│  🟢 GREEN STATE (Unknown Person)                            │
│     • Face detected but not the target person               │
│     • Audio: Silent (no alerts)                             │
│     • LED: Solid GREEN (GPIO pin 27)                        │
│     • Status: Caution mode                                   │
│     • Transitions: → RED (if target person appears)         │
│                    → BLUE (if unknown person leaves)        │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

**State Machine Features:**
- **Jitter Tolerance:** 5-second window for brief interruptions (prevents false loss alerts)
- **Loss Confirmation:** 15-second confirmation window after jitter (total ~20s to loss alert)
- **Cooldown Periods:** 2s between recognition alerts, 5s quiet period after loss alert
- **Status Publishing:** JSON messages at 10 Hz for LED, database, and future dialogue system

**Status Message Format (JSON):**
```json
{
  "status": "red|blue|green",
  "person_identity": "target_person|no_person|unknown",
  "timestamp_sec": 1765212914,
  "timestamp_nanosec": 949382424,
  "confidence": 0.95,
  "duration_seconds": 15.3,
  "is_loss_state": false,
  "audio_event": "recognition|loss|none"
}
```

**For complete person recognition and status system setup, see:** [`100_PERSON_RECOGNITION_AND_STATUS.md`](100_PERSON_RECOGNITION_AND_STATUS.md) ⭐ **Complete Setup Guide**

### 4.2 Audio Notification Components

**audio_notification_node:**
- Subscribes to `/r2d2/perception/person_id`
- Implements 3-state machine (RED/BLUE/GREEN)
- Plays MP3 audio alerts via ffplay
- Publishes status JSON for downstream consumers
- Configurable parameters: volume, timing, audio files

**status_led_node:**
- Subscribes to `/r2d2/audio/person_status`
- Controls RGB LED via GPIO (pins 17, 27, 22)
- Real-time visual feedback synchronized with audio
- Auto-detects GPIO availability (simulation mode fallback)

**database_logger_node:**
- Subscribes to `/r2d2/audio/person_status`
- Logs state transitions to console (structure ready for SQLite)
- Tracks recognition events for conversation history
- Future: SQLite database for analytics and memory

**For complete system setup including audio, see:** [`100_PERSON_RECOGNITION_AND_STATUS.md`](100_PERSON_RECOGNITION_AND_STATUS.md)

---

## 5. Processing Pipeline

### 5.1 Step-by-Step Image Processing

```
Each frame (30 FPS):
  1. Receive /oak/rgb/image_raw
  2. Convert ROS Image → OpenCV (BGR array)
  3. Downscale 1920×1080 → 640×360 (4× size reduction)
  4. Convert to grayscale
  5. Compute brightness (mean pixel value)
  6. Detect faces (Haar Cascade)
  7. (If face_count > 0 AND recognition enabled AND skip counter met)
     → Extract face, resize to 100×100, run LBPH recognizer
  8. Publish all results on respective topics

Frame processing time: ~10 ms (without recognition)
                      ~25 ms (with recognition, when triggered)
```

### 5.2 Performance Characteristics & Resource Allocation

**Current Resource Usage (All Components Running):**

| Component | CPU Usage | Memory | GPU | Frequency | Notes |
|-----------|-----------|--------|-----|-----------|-------|
| **camera_node** | 2-3% (1 core) | ~50 MB | 0% | 30 Hz | Camera driver |
| **image_listener** | 8-15% (1 core) | ~200 MB | 0% | 13 Hz | Image processing |
| **audio_notification_node** | 2-4% (1 core) | ~50 MB | 0% | 10 Hz | State machine |
| **status_led_node** | <0.1% (1 core) | ~20 MB | 0% | 10 Hz | GPIO control |
| **database_logger_node** | <0.1% (1 core) | ~30 MB | 0% | 10 Hz | Event logging |
| **heartbeat_node** | <0.1% (1 core) | ~10 MB | 0% | 1 Hz | Health monitor |
| **System overhead** | 5-10% | ~140 MB | 0% | N/A | ROS 2, OS |
| **TOTAL** | **~15-25%** | **~500 MB** | **0%** | — | **All Phase 1 components** |

**Resource Headroom Available (for Phase 2-4):**

```
CPU:
  Total cores: 12 (ARM Cortex-A78)
  Used: ~1-2 cores (15-25% total)
  Available: ~10-11 cores (75-85% headroom)
  
Memory:
  Total: 64 GB LPDDR5X
  Used: ~500 MB (0.8%)
  Available: ~63.5 GB (99.2% headroom)
  
GPU:
  Total: 504 CUDA cores (Ampere architecture)
  Used: 0% (not accelerated yet)
  Available: 100% (full GPU available for Phase 2 LLM, Phase 3 SLAM)
  
Storage:
  Total: ~32 GB usable (eMMC)
  Used: ~5-10 GB (OS, ROS 2, models)
  Available: ~22-27 GB (sufficient for Phase 2-3 models)
```

**Performance Baselines (Expected Values):**

| Metric | Expected Value | Issue When | Notes |
|--------|---------------|------------|-------|
| Camera FPS | 30 Hz | <25 Hz | USB bandwidth or camera issue |
| Perception rate | 13 Hz | <10 Hz | CPU overload or frame processing issue |
| Face recognition rate | 6.5 Hz | <5 Hz | Recognition enabled but slow |
| Audio status rate | 10 Hz | <5 Hz | Audio node performance issue |
| CPU usage (total) | 15-25% | >50% | System overload, check processes |
| Memory usage | ~500 MB | >2 GB | Memory leak, check nodes |
| GPU usage | 0% | N/A | Not used in Phase 1 (available for Phase 2) |

**Timing Guarantees:**

| Data Flow | Latency | Notes |
|-----------|---------|-------|
| Camera → Perception | <33 ms | Frame-to-frame processing |
| Perception → Audio | <100 ms | Topic subscription + state update |
| Audio → LED | <100 ms | GPIO response time |
| Recognition → Alert | <200 ms | State transition to audio playback start |
| End-to-end (camera → alert) | <500 ms | Total system latency |

**For detailed performance analysis, see:** `_ANALYSIS_AND_DOCUMENTATION/COMPUTE_COST_ANALYSIS.md`

---

## 6. Launch Configuration

### 6.1 Launch Parameters (Perception Pipeline)

```
LAUNCH COMMAND:
  ros2 launch r2d2_bringup r2d2_camera_perception.launch.py [ARGS]

AVAILABLE PARAMETERS:
  enable_face_recognition        (bool, default: false)
  recognition_frame_skip         (int, default: 2)
  recognition_confidence_threshold (float, default: 70.0)
  face_recognition_model_path    (string, default: ~/dev/r2d2/data/...)
  log_every_n_frames            (int, default: 30)
  log_face_detections           (bool, default: false)
  save_debug_gray_frame         (bool, default: false)

EXAMPLES:
  # Default (no recognition)
  ros2 launch r2d2_bringup r2d2_camera_perception.launch.py
  
  # With recognition enabled
  ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
    enable_face_recognition:=true
  
  # With verbose logging
  ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
    enable_face_recognition:=true \
    log_every_n_frames:=10 \
    log_face_detections:=true
```

### 6.2 Launch Parameters (Audio Notification System)

**Audio Notification Node Parameters:**

```
LAUNCH COMMAND:
  ros2 launch r2d2_audio audio_notification.launch.py [ARGS]

AVAILABLE PARAMETERS:
  target_person                        (string, default: "target_person")
    → Person name to recognize and alert on (should match training data)
  
  audio_volume                         (float, default: 0.05)
    → Global volume control (0.0=silent, 0.05=5%, 1.0=max)
    → Controls ALL audio alerts (recognition + loss)
  
  jitter_tolerance_seconds             (float, default: 5.0)
    → Brief gap tolerance (ignores gaps < 5s during recognition)
    → Prevents false loss alerts from brief interruptions
  
  loss_confirmation_seconds            (float, default: 15.0)
    → Confirmation window AFTER jitter tolerance exceeded
    → Total time to loss alert: ~20s (5s jitter + 15s confirmation)
  
  cooldown_seconds                     (float, default: 2.0)
    → Minimum time between same alert type (prevents spam)
  
  recognition_cooldown_after_loss_seconds (float, default: 5.0)
    → Quiet period after loss alert (prevents rapid re-recognition beeps)
  
  recognition_audio_file               (string, default: "Voicy_R2-D2 - 2.mp3")
    → MP3 file for recognition alert ("Hello!")
  
  loss_audio_file                      (string, default: "Voicy_R2-D2 - 5.mp3")
    → MP3 file for loss alert ("Oh, I lost you!")
  
  enabled                              (bool, default: true)
    → Enable/disable audio notification system

EXAMPLES:
  # Default settings (5% volume, 15s loss confirmation)
  ros2 launch r2d2_audio audio_notification.launch.py
  
  # Higher volume, faster loss detection
  ros2 launch r2d2_audio audio_notification.launch.py \
    audio_volume:=0.3 \
    loss_confirmation_seconds:=10.0
  
  # Different target person (must match training data)
  ros2 launch r2d2_audio audio_notification.launch.py \
    target_person:=alice
```

**For complete parameter documentation, see:** [`100_PERSON_RECOGNITION_AND_STATUS.md`](100_PERSON_RECOGNITION_AND_STATUS.md) (Section: Configuration & Tuning)

### 6.3 Complete Parameter Reference

**All Launch Parameters Across All Packages:**

| Package | Node | Parameter | Type | Default | Purpose |
|---------|------|-----------|------|---------|---------|
| **r2d2_perception** | image_listener | `enable_face_recognition` | bool | false | Enable LBPH recognition |
| | | `recognition_frame_skip` | int | 2 | Process every Nth frame (6.5 Hz) |
| | | `recognition_confidence_threshold` | float | 70.0 | Recognition threshold (lower = higher confidence) |
| | | `face_recognition_model_path` | string | ~/dev/r2d2/data/... | Path to LBPH model XML |
| | | `log_every_n_frames` | int | 30 | Logging frequency |
| | | `log_face_detections` | bool | false | Verbose face detection logging |
| | | `save_debug_gray_frame` | bool | false | Save one-time debug frame |
| **r2d2_audio** | audio_notification_node | `target_person` | string | "target_person" | Person to recognize |
| | | `audio_volume` | float | 0.05 | Global volume (0.0-1.0) |
| | | `jitter_tolerance_seconds` | float | 5.0 | Brief gap tolerance |
| | | `loss_confirmation_seconds` | float | 15.0 | Loss confirmation window |
| | | `cooldown_seconds` | float | 2.0 | Min between alerts |
| | | `recognition_cooldown_after_loss_seconds` | float | 5.0 | Quiet period after loss |
| | | `recognition_audio_file` | string | "Voicy_R2-D2 - 2.mp3" | Recognition MP3 |
| | | `loss_audio_file` | string | "Voicy_R2-D2 - 5.mp3" | Loss MP3 |
| | | `enabled` | bool | true | Enable/disable system |

**Parameter Tuning Guidelines:**

- **CPU Optimization:** Increase `recognition_frame_skip` (e.g., 3 or 4) to reduce CPU usage
- **Accuracy vs Speed:** Lower `recognition_confidence_threshold` (e.g., 60.0) for more detections but more false positives
- **Audio Responsiveness:** Lower `loss_confirmation_seconds` (e.g., 10.0) for faster loss alerts
- **False Alarm Prevention:** Increase `jitter_tolerance_seconds` (e.g., 7.0) for noisy environments

---

## 7. Hardware Control Systems

### 7.1 Power Button System

The R2D2 system includes a power button control system for graceful shutdown and wake:

**Button 1 (Shutdown):**
- **GPIO Pin:** 32 (40-pin expansion header)
- **Function:** Press to initiate graceful shutdown
- **Action:** `shutdown -h now`
- **Status:** ✅ Tested and operational
- **Service:** `r2d2-powerbutton.service` (systemd)

**Button 2 (Boot/Wake):**
- **Location:** J42 Automation Header
- **Pins:** Pin 4 (POWER) + Pin 1 (GND)
- **Function:** Short pins to wake from low-power or boot from shutdown
- **Status:** Ready for testing

**For detailed power button documentation, see:** [`080_POWER_BUTTON_FINAL_DOCUMENTATION.md`](080_POWER_BUTTON_FINAL_DOCUMENTATION.md)

### 7.2 Audio Hardware

**Audio Output:**
- **Amplifier:** PAM8403 (3W stereo)
- **Speaker:** 8Ω speaker
- **Interface:** I2S via Jetson J511 Pin 9 (HPO_L)
- **ALSA Device:** `hw:1,0`
- **Status:** ✅ Operational

**Audio Input (Phase 2):**
- **Hardware:** ReSpeaker 2-Mic HAT
- **Status:** ⏳ Ordered, pending integration
- **For setup documentation, see:** [`050_AUDIO_SETUP_AND_CONFIGURATION.md`](050_AUDIO_SETUP_AND_CONFIGURATION.md)

---

## 8. Integration Points for Future Features

### 8.1 Integration Points for Phase 2-4

**Where to Hook In New Components:**

The architecture provides clear integration points for future development:

1. **Perception Topics** (for context-aware features):
   - `/r2d2/perception/person_id` - Who is present?
   - `/r2d2/perception/face_count` - How many people?
   - `/r2d2/perception/brightness` - Lighting conditions
   - `/r2d2/audio/person_status` - Recognition state (RED/BLUE/GREEN)

2. **Command Topics** (for robot actions):
   - `/r2d2/cmd/*` - Future command topics (follow_person, navigate_to, etc.)
   - `/r2d2/cmd_vel` - Movement commands (geometry_msgs/Twist)

3. **Status Topics** (for system health):
   - `/r2d2/heartbeat` - System health indicator

**Template for Adding New Nodes:**

```python
from rclpy.node import Node
from std_msgs.msg import String, Float32
import json

class MyNewPhase2Node(Node):
    def __init__(self):
        super().__init__('my_phase2_node')
        
        # Subscribe to existing perception/audio topics
        self.person_sub = self.create_subscription(
            String, '/r2d2/perception/person_id',
            self.person_callback, 10)
        
        self.status_sub = self.create_subscription(
            String, '/r2d2/audio/person_status',
            self.status_callback, 10)
        
        # Publish new results
        self.output_pub = self.create_publisher(
            String, '/r2d2/my_subsystem/output', 10)
    
    def person_callback(self, msg):
        """Handle person recognition updates"""
        person_id = msg.data
        # Your logic here
    
    def status_callback(self, msg):
        """Handle status updates (RED/BLUE/GREEN)"""
        status_data = json.loads(msg.data)
        # Your logic here
```

**For detailed integration guide, see:** `_ANALYSIS_AND_DOCUMENTATION/INTEGRATION_GUIDE.md`

### 8.2 Adding Phase 2 Components (Speech/Conversation)

**Phase 2 Architecture (Hybrid Approach):**

Phase 2 uses a **hybrid architecture** where the speech pipeline runs as a **non-ROS daemon** for low latency, and only discrete commands are published to ROS 2:

```
ReSpeaker 2-Mic HAT
    ↓
Microphone Manager (Python daemon, NOT ROS)
    ├─ Wake word detection: "Hey R2D2" (Porcupine)
    ├─ Audio recording: ring buffer
    ↓
Speech-to-Text (STT): Whisper (local, GPU-accelerated)
    ↓
LLM Processing: Grok API (cloud) or Ollama (local fallback)
    ├─ Parse user intent
    ├─ Generate conversational response
    └─ Detect commands: "come to me", "go to X"
    ↓
Text-to-Speech (TTS): gTTS or Coqui TTS
    ↓
COMMAND EXTRACTION (only discrete actions go to ROS)
    ├─ IF "come to me" → publish /r2d2/cmd/follow_person
    ├─ IF "go to living room" → publish /r2d2/cmd/navigate_to
    └─ OTHERWISE: just speak response (no ROS action)
```

**Rationale:**
- ✅ Minimal ROS overhead for continuous speech processing
- ✅ Low latency (no ROS message serialization)
- ✅ Easy to test components independently
- ✅ ROS only for discrete robot commands

**Integration Points:**
- Subscribe to `/r2d2/audio/person_status` (who's speaking? context)
- Subscribe to `/r2d2/perception/person_id` (recognition status)
- Publish to `/r2d2/cmd/*` topics (discrete commands)
- Follow naming convention: `/r2d2/<subsystem>/<metric>`

**For detailed Phase 2 architecture, see:** [`200_SPEECH_ARCHITECTURE_RECOMMENDATION.md`](200_SPEECH_ARCHITECTURE_RECOMMENDATION.md)

### 8.3 Adding Phase 3 Components (Navigation)

New nodes should:
- Subscribe to `/r2d2/perception/face_count` (obstacle avoidance)
- Subscribe to `/r2d2/perception/person_id` (follow person)
- Publish to `/r2d2/cmd_vel` (movement commands, geometry_msgs/Twist)

### 8.4 General Pattern for New Nodes

```python
from rclpy.node import Node
from std_msgs.msg import String, Float32

class MyNewNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # Subscribe to perception results
        self.person_sub = self.create_subscription(
            String, '/r2d2/perception/person_id',
            self.person_callback, 10)
        
        # Publish new results
        self.output_pub = self.create_publisher(
            String, '/r2d2/subsystem/metric', 10)
    
    def person_callback(self, msg):
        if msg.data == self.target_person_name:
            # React to target person
            pass
```

---

## 9. Key Files Reference

```
MAIN LAUNCH:
  ~/dev/r2d2/ros2_ws/src/r2d2_bringup/launch/r2d2_camera_perception.launch.py

NODES:
  ~/dev/r2d2/ros2_ws/src/r2d2_camera/r2d2_camera/oak_camera_node.py
  ~/dev/r2d2/ros2_ws/src/r2d2_perception/r2d2_perception/image_listener.py
  ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/audio_notification_node.py
  ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/status_led_node.py
  ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/database_logger_node.py

FACE RECOGNITION MODEL:
  ~/dev/r2d2/data/face_recognition/models/severin_lbph.xml

AUDIO ASSETS:
  ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/
    ├─ Voicy_R2-D2 - 2.mp3 (recognition alert)
    └─ Voicy_R2-D2 - 5.mp3 (loss alert)

HARDWARE CONTROL:
  ~/dev/r2d2/r2d2_power_button_simple.py
  /etc/systemd/system/r2d2-powerbutton.service

DOCUMENTATION:
  ~/dev/r2d2/041_CAMERA_SETUP_DOCUMENTATION.md
  ~/dev/r2d2/100_PERSON_RECOGNITION_AND_STATUS.md (⭐ Complete setup guide)
  ~/dev/r2d2/102_CAMERA_SETUP_DOCUMENTATION.md (Camera hardware - prerequisite)
  ~/dev/r2d2/101_SPEAKER_AUDIO_SETUP_DOCUMENTATION.md (Audio hardware - prerequisite)
  ~/dev/r2d2/050_AUDIO_SETUP_AND_CONFIGURATION.md (Alternative audio documentation)
  ~/dev/r2d2/080_POWER_BUTTON_FINAL_DOCUMENTATION.md
  ~/dev/r2d2/200_SPEECH_ARCHITECTURE_RECOMMENDATION.md
```

---

## 10. Monitoring Commands

```
# Watch person identification
ros2 topic echo /r2d2/perception/person_id

# Check publication rate (should be 6.5 Hz with recognition)
ros2 topic hz /r2d2/perception/person_id

# Monitor CPU usage
watch -n 1 'top -bn1 | grep -E "python|Cpu" | head -10'

# List all active topics
ros2 topic list

# View node information
ros2 node info /image_listener

# Monitor audio notification system
ros2 topic echo /r2d2/audio/person_status --no-arr
ros2 topic echo /r2d2/audio/notification_event

# Check audio notification node status
ros2 node info /audio_notification_node
ros2 param list /audio_notification_node

# Monitor LED status
ros2 node info /status_led_controller

# Check power button service
sudo systemctl status r2d2-powerbutton.service
```

---

## Next Steps

1. **Understand**: Read this document, visualize the data flow
2. **Verify**: Run system and confirm all topics are publishing
3. **Extend**: Add Phase 2 features using integration patterns above
4. **Monitor**: Use provided commands to track performance

**Next Documents:**
- `100_PERSON_RECOGNITION_AND_STATUS.md` - ⭐ Complete setup guide for person recognition and status system
- `102_CAMERA_SETUP_DOCUMENTATION.md` - Camera hardware setup (prerequisite for 100_)
- `101_SPEAKER_AUDIO_SETUP_DOCUMENTATION.md` - Audio hardware setup (prerequisite for 100_)
- `200_SPEECH_ARCHITECTURE_RECOMMENDATION.md` - Phase 2 planning
- `QUICK_START.md` - Quick reference guide

---

## Related Documentation

**Component-Specific Documentation:**
- **Complete Person Recognition & Status System:** [`100_PERSON_RECOGNITION_AND_STATUS.md`](100_PERSON_RECOGNITION_AND_STATUS.md) ⭐ **Complete Setup Guide**
- **Camera Setup:** [`102_CAMERA_SETUP_DOCUMENTATION.md`](102_CAMERA_SETUP_DOCUMENTATION.md)
- **Audio Hardware:** [`101_SPEAKER_AUDIO_SETUP_DOCUMENTATION.md`](101_SPEAKER_AUDIO_SETUP_DOCUMENTATION.md)
- **Audio Hardware (Alternative):** [`050_AUDIO_SETUP_AND_CONFIGURATION.md`](050_AUDIO_SETUP_AND_CONFIGURATION.md)
- **Power Button:** [`080_POWER_BUTTON_FINAL_DOCUMENTATION.md`](080_POWER_BUTTON_FINAL_DOCUMENTATION.md)
- **Phase 2 Architecture:** [`200_SPEECH_ARCHITECTURE_RECOMMENDATION.md`](200_SPEECH_ARCHITECTURE_RECOMMENDATION.md)

**Analysis & Planning:**
- `ARCHITECTURE_ANALYSIS.md` - Comprehensive architecture analysis
- `ARCHITECTURE_CONSISTENCY_ANALYSIS.md` - Documentation consistency review

---

*Architecture Overview created: December 7, 2025*  
*Last updated: December 9, 2025 (Comprehensive update: Added audio system, state machine, all nodes, Phase 2 hybrid architecture, hardware control, complete topic reference)*
