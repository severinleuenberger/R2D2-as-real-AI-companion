# Face Recognition System: Quick Start Guide

**Platform:** R2D2 on NVIDIA Jetson AGX Orin 64GB  
**Status:** ✅ Production Ready  
**Last Updated:** December 8, 2025

---

## What Is This?

R2D2's **face recognition system** enables personal identification in real-time. When activated, the perception pipeline recognizes "Severin" vs. unknown people and publishes this information to ROS 2 topics.

This is a **quick start guide** for getting started. See **[`040_FACE_RECOGNITION_COMPLETE.md`](040_FACE_RECOGNITION_COMPLETE.md)** for complete documentation.

---

## Quick Start (5 Minutes)

### Prerequisites

- ✅ Camera setup complete (see [`020_CAMERA_SETUP_DOCUMENTATION.md`](020_CAMERA_SETUP_DOCUMENTATION.md))
- ✅ Perception pipeline running (see [`030_PERCEPTION_PIPELINE_SETUP.md`](030_PERCEPTION_PIPELINE_SETUP.md))
- ✅ Environment configured

### 1. Activate Environment

```bash
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8
```

### 2. Capture Training Images

```bash
cd ~/dev/r2d2/tests/face_recognition
python3 1_capture_training_data.py

# Follow on-screen prompts:
# - Look at camera in bright light (20 sec)
# - Look at camera in bright light from farther away (20 sec)
# - Look at camera in low light (20 sec)
# - Look at camera in low light from farther away (20 sec)

# Result: ~80 images in ~/dev/r2d2/data/face_recognition/severin/
```

### 3. Train Recognition Model

```bash
python3 2_train_recognizer.py

# Result: Model saved to ~/dev/r2d2/data/face_recognition/models/severin_lbph.xml
```

### 4. Test Recognition (Optional)

```bash
python3 3_test_recognizer_demo.py

# Shows accuracy statistics
```

### 5. Launch with Face Recognition Enabled

```bash
# Terminal 1: Start perception with recognition
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py enable_face_recognition:=true

# Terminal 2: Monitor person ID
ros2 topic echo /r2d2/perception/person_id
```

**Expected Output:**
```
data: severin    # When you look at camera
---
data: unknown    # When someone else is visible
---
```

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Camera (OAK-D Lite)                      │
│                    1920×1080 @ 30 FPS                       │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ↓
         ┌───────────────────────────────────┐
         │  Perception Pipeline (ROS 2)      │
         │  image_listener.py                │
         │  13 Hz frame processing           │
         └────────────────┬────────────────────┘
                          │
                          ↓
          ┌───────────────────────────────────┐
          │   Face Detection (Haar Cascade)   │
          │   640×360 grayscale               │
          │   13 Hz processing                │
          └────────────────┬────────────────────┘
                           │
                           ↓
          ┌───────────────────────────────────┐
          │  Face Recognition (LBPH Model)    │
          │  Trained on 80+ images            │
          │  6-13 Hz (configurable)           │
          └────────────────┬────────────────────┘
                           │
                           ↓
        ┌──────────────────────────────────────┐
        │     ROS 2 Topics (Published)         │
        ├──────────────────────────────────────┤
        │ /r2d2/perception/person_id           │
        │   → "severin" or "unknown"           │
        │                                      │
        │ /r2d2/perception/face_confidence     │
        │   → Float32 (0-100+)                 │
        │                                      │
        │ /r2d2/perception/is_severin          │
        │   → Bool (true/false)                │
        └──────────────────────────────────────┘
                           │
                           ↓
        ┌──────────────────────────────────────┐
        │   Downstream Integration             │
        ├──────────────────────────────────────┤
        │ • Audio notifications                │
        │ • LED control                        │
        │ • Dialogue system                    │
        │ • Custom ROS 2 nodes                 │
        └──────────────────────────────────────┘
```

---

## Key Topics

| Topic | Type | Content | Frequency |
|-------|------|---------|-----------|
| `/r2d2/perception/person_id` | `std_msgs/String` | `"severin"` or `"unknown"` | 6-13 Hz |
| `/r2d2/perception/face_confidence` | `std_msgs/Float32` | Confidence 0-100+ (lower=better) | 6-13 Hz |
| `/r2d2/perception/is_severin` | `std_msgs/Bool` | Boolean convenience topic | 6-13 Hz |

---

## Configuration

### Launch Parameters

```bash
# Default (recognition disabled)
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py

# With recognition enabled
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true

# With custom threshold (lower=stricter)
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true \
  recognition_confidence_threshold:=75.0

# With reduced CPU usage (process every 3rd frame)
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true \
  recognition_frame_skip:=3
```

### CPU Usage by Configuration

| Configuration | CPU | Frequency | Use Case |
|---------------|-----|-----------|----------|
| frame_skip=1 | 18-25% | 13 Hz | Maximum accuracy |
| frame_skip=2 | 10-15% | 6.5 Hz | **Recommended** |
| frame_skip=3 | 8-10% | 4.3 Hz | Minimal CPU |
| frame_skip=6 | 3-5% | 2.5 Hz | Very minimal |

**Recommended:** `frame_skip=2` (10-12% CPU, 6.5 Hz frequency)

---

## Monitoring in ROS 2

### Watch Person Identification

```bash
ros2 topic echo /r2d2/perception/person_id
```

**Output:**
```
data: severin
---
data: unknown
---
data: severin
```

### Watch Confidence Scores

```bash
ros2 topic echo /r2d2/perception/face_confidence
```

**Output:**
```
data: 35.2    # High confidence (Severin)
---
data: 92.1    # Low confidence (unknown)
---
data: 40.1    # High confidence (Severin)
```

### Check Publishing Frequency

```bash
ros2 topic hz /r2d2/perception/person_id
```

**Output:**
```
average rate: 6.50
min: 0.150s max: 0.160s std dev: 0.0050s count: 13
```

### Monitor System Resource Usage

```bash
watch -n 1 'top -bn1 | head -10'
```

---

## Service Management (Background)

The face recognition service runs in background and updates a JSON status file:

### Start Service

```bash
cd ~/dev/r2d2/tests/face_recognition
python3 face_recognition_service.py start severin ~/dev/r2d2/data/face_recognition
```

### Check Service Status

```bash
python3 face_recognition_service.py status
```

**Output:**
```
======================================================================
FACE RECOGNITION SERVICE STATUS
======================================================================
Timestamp: 2025-12-06T16:47:28.123456
✅ RECOGNIZED: severin
Frames: 1234
Threshold: 70
======================================================================
```

### View Logs

```bash
python3 face_recognition_service.py logs 50
```

### Stop Service

```bash
pkill -f face_recognition_service.py
```

---

## Troubleshooting

### Issue: Always returns "unknown"

**Cause:** Model not well-trained or threshold too strict  
**Solution:**
1. Capture more diverse images (different angles, lighting)
2. Lower confidence threshold (less strict)
3. Retrain model

### Issue: Faces not detected

**Cause:** Face detection disabled or camera issue  
**Solution:**
1. Verify camera working: `ros2 topic echo /oak/rgb/image_raw`
2. Check `/r2d2/perception/face_count` is non-zero
3. See [`04_FACE_DETECTION_SETUP.md`](_ARCHIVED_FACE_RECOGNITION_DOCS_v1/04_FACE_DETECTION_SETUP.md) for detection troubleshooting

### Issue: CPU usage too high

**Cause:** Frame skip set too low  
**Solution:**
```bash
# Use frame_skip=3 instead of default 2
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true \
  recognition_frame_skip:=3
```

### Issue: Model not found

**Cause:** Training model doesn't exist  
**Solution:**
```bash
# Run training process
cd ~/dev/r2d2/tests/face_recognition
python3 1_capture_training_data.py
python3 2_train_recognizer.py

# Verify model exists
ls -la ~/dev/r2d2/data/face_recognition/models/severin_lbph.xml
```

---

## Key Files

### Training & Service

```
~/dev/r2d2/tests/face_recognition/
├── 1_capture_training_data.py     # Capture training images
├── 2_train_recognizer.py          # Train LBPH model
├── 3_test_recognizer_demo.py      # Test model accuracy
├── face_recognition_service.py    # Background service
└── train_manager.py               # Training menu
```

### Data & Models

```
~/dev/r2d2/data/face_recognition/
├── severin/                       # Training images (~80)
└── models/
    └── severin_lbph.xml          # Trained model (33 MB)
```

### ROS 2 Integration

```
~/dev/r2d2/ros2_ws/src/r2d2_perception/
├── r2d2_perception/
│   └── image_listener.py          # Perception node with recognition
└── launch/
    └── perception.launch.py       # Recognition launch config
```

---

## Complete Documentation

For detailed information, see **[`040_FACE_RECOGNITION_COMPLETE.md`](040_FACE_RECOGNITION_COMPLETE.md)** which includes:

- ✅ Complete ROS 2 integration guide
- ✅ Launch configuration reference
- ✅ CPU and performance optimization
- ✅ Training best practices
- ✅ Service monitoring and logging
- ✅ LED controller integration
- ✅ Comprehensive troubleshooting
- ✅ LBPH recognizer explanation
- ✅ Audio integration examples
- ✅ Code integration examples

---

## Related Documentation

- **Camera Setup:** [`020_CAMERA_SETUP_DOCUMENTATION.md`](020_CAMERA_SETUP_DOCUMENTATION.md)
- **Perception Pipeline:** [`030_PERCEPTION_PIPELINE_SETUP.md`](030_PERCEPTION_PIPELINE_SETUP.md)
- **Face Detection (Archived):** [`_ARCHIVED_FACE_RECOGNITION_DOCS_v1/04_FACE_DETECTION_SETUP.md`](_ARCHIVED_FACE_RECOGNITION_DOCS_v1/04_FACE_DETECTION_SETUP.md)
- **Audio Integration:** [`060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md`](060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md)
- **Audio Quick Reference:** [`AUDIO_QUICK_REFERENCE.md`](AUDIO_QUICK_REFERENCE.md)

---

## Command Cheat Sheet

```bash
# ==== SETUP ====
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8
cd ~/dev/r2d2/tests/face_recognition

# ==== TRAINING ====
python3 1_capture_training_data.py  # Capture 80 images
python3 2_train_recognizer.py       # Train model
python3 3_test_recognizer_demo.py   # Test accuracy

# ==== ROS 2 LAUNCH ====
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py enable_face_recognition:=true

# ==== MONITORING ====
ros2 topic echo /r2d2/perception/person_id
ros2 topic hz /r2d2/perception/person_id
ros2 topic echo /r2d2/perception/face_confidence

# ==== SERVICE ====
python3 face_recognition_service.py start severin ~/dev/r2d2/data/face_recognition
python3 face_recognition_service.py status
python3 face_recognition_service.py logs 50
pkill -f face_recognition_service.py
```

---

**Status:** ✅ Production Ready  
**Last Updated:** December 8, 2025  
**See Also:** [`040_FACE_RECOGNITION_COMPLETE.md`](040_FACE_RECOGNITION_COMPLETE.md) for complete documentation
