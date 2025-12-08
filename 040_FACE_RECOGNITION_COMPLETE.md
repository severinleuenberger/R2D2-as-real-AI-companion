# Face Recognition: Integration, Training & Service

**Date:** December 8, 2025 (Consolidated)  
**Version:** 2.0 - Complete System  
**Status:** ✅ Production Ready  
**Platform:** NVIDIA Jetson AGX Orin 64GB with ROS 2 Humble

---

## Executive Summary

The R2D2 face recognition system operates at two levels:

1. **ROS 2 Level:** Real-time person identification published on `/r2d2/perception/person_id` topic with confidence scores
2. **Service Level:** Background service for training, status monitoring, and LED control integration

This document covers both levels with complete setup, configuration, training, monitoring, and troubleshooting.

**Prerequisites:**
- Face detection working (see archived documentation in [`_ARCHIVED_FACE_RECOGNITION_DOCS_v1/`](_ARCHIVED_FACE_RECOGNITION_DOCS_v1/))
- OAK-D camera configured (see [`020_CAMERA_SETUP_DOCUMENTATION.md`](020_CAMERA_SETUP_DOCUMENTATION.md))
- ROS 2 perception pipeline running (see [`030_PERCEPTION_PIPELINE_SETUP.md`](030_PERCEPTION_PIPELINE_SETUP.md))

---

## Part 1: ROS 2 Integration

### Overview

The R2D2 perception pipeline includes **personal face recognition** using OpenCV's LBPH (Local Binary Pattern Histograms) recognizer. The LBPH model is trained on captured face images and runs at ~6-13 Hz depending on CPU budget.

**System Flow:**
```
Camera Frame (30 FPS)
    ↓
Face Detection (13 Hz) [from 04_FACE_DETECTION_SETUP.md]
    ↓
Face Recognition (6-13 Hz depending on frame_skip)
    ↓
Publishes: person_id, face_confidence, is_severin
    ↓
Consumed by: ROS 2 nodes, audio notifications, LED control
```

### New ROS 2 Topics

When face recognition is enabled, the pipeline publishes to three new topics:

#### 1. `/r2d2/perception/person_id` (String)

```
Message Type: std_msgs/String
Content: "severin" or "unknown"
Frequency: Every 2-3 frames (~4-6 Hz with frame_skip=2)
Purpose: High-level person identification
```

**Example consumption:**
```python
# Subscribe to person identification
self.person_sub = self.create_subscription(
    String,
    '/r2d2/perception/person_id',
    self.person_callback,
    10
)

def person_callback(self, msg: String):
    if msg.data == "severin":
        self.get_logger().info('Recognized Severin!')
    else:
        self.get_logger().info(f'Unknown person: {msg.data}')
```

#### 2. `/r2d2/perception/face_confidence` (Float32)

```
Message Type: std_msgs/Float32
Content: Confidence score (0-100+)
Frequency: Every 2-3 frames (~4-6 Hz with frame_skip=2)
Purpose: Confidence metric for recognition result
Interpretation: Lower is higher confidence (0-40=strong match, 70+=weak match)
```

**Example:** A confidence of 35.2 means "very confident this is Severin", 92.1 means "likely unknown"

#### 3. `/r2d2/perception/is_severin` (Bool)

```
Message Type: std_msgs/Bool
Content: true/false
Frequency: Every 2-3 frames (~4-6 Hz with frame_skip=2)
Purpose: Boolean convenience topic for downstream logic
```

**Existing topics** (unchanged):
- `/r2d2/perception/brightness` (Float32)
- `/r2d2/perception/face_count` (Int32)
- `/oak/rgb/image_raw` (sensor_msgs/Image)

### Launch Configuration

#### Launch Without Face Recognition (Default)

```bash
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py
```

This launches camera + perception but **no LBPH recognition**.

#### Launch With Face Recognition (After training)

```bash
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true
```

**Requirements:**
- Trained model must exist at `~/dev/r2d2/data/face_recognition/models/severin_lbph.xml`
- If model not found, node logs warning and continues without recognition

#### Launch With Custom Settings

```bash
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true \
  face_recognition_model_path:=/path/to/custom_model.xml \
  recognition_confidence_threshold:=75.0 \
  recognition_frame_skip:=3
```

### Launch Parameters Reference

| Parameter | Default | Type | Purpose |
|-----------|---------|------|---------|
| `enable_face_recognition` | `false` | bool | Enable/disable LBPH recognition |
| `face_recognition_model_path` | `~/dev/r2d2/data/face_recognition/models/severin_lbph.xml` | string | Path to trained LBPH model |
| `recognition_confidence_threshold` | `70.0` | float | Threshold for "Severin" classification (lower=stricter) |
| `recognition_frame_skip` | `2` | int | Process recognition every Nth frame (manages CPU load) |

### CPU & Performance

#### Compute Budget

- **Brightness + face detection only:** ~8-10% CPU (one core)
- **With recognition (frame_skip=1):** ~18-25% CPU (processes at ~13 Hz)
- **With recognition (frame_skip=2):** ~10-15% CPU (processes at ~6.5 Hz)
- **With recognition (frame_skip=3):** ~8-10% CPU (processes at ~4.3 Hz)

#### Recommended Configuration

```bash
# Balanced: 10-12% CPU, good recognition frequency
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true \
  recognition_frame_skip:=2
```

#### Troubleshooting High CPU

```bash
# If CPU usage is too high, increase frame skip
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true \
  recognition_frame_skip:=3  # Process every 3rd frame (~4 Hz)
```

### Monitoring Recognition in ROS 2

#### Monitor Person ID Topic

```bash
# Terminal 1: Start the pipeline
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true

# Terminal 2: Watch person identification
ros2 topic echo /r2d2/perception/person_id

# Expected output:
# data: severin
# ---
# data: unknown
# ---
# data: severin
```

#### Monitor Confidence Scores

```bash
# Terminal 2: Watch confidence values
ros2 topic echo /r2d2/perception/face_confidence

# Expected output (lower is better):
# data: 35.2  # High confidence Severin
# ---
# data: 92.1  # Low confidence, likely unknown
```

#### Monitor Frequency

```bash
# Check topic publishing rate
ros2 topic hz /r2d2/perception/person_id

# Expected (with frame_skip=2):
# average rate: 6.50
# min: 0.150s max: 0.160s std dev: 0.0050s count: 13
```

#### Monitor Resource Usage

```bash
# Check CPU/memory on Jetson
watch -n 1 'top -bn1 | grep -E "python|Cpu" | head -5'

# Or use tegrastats for full system status
tegrastats
```

#### Code Integration Example

If you want to consume the face recognition results in another ROS 2 node:

```python
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool

class MyPerceptionNode(Node):
    def __init__(self):
        super().__init__('my_perception_node')
        
        # Subscribe to person identification
        self.person_sub = self.create_subscription(
            String,
            '/r2d2/perception/person_id',
            self.person_callback,
            10
        )
        
        # Subscribe to confidence
        self.confidence_sub = self.create_subscription(
            Float32,
            '/r2d2/perception/face_confidence',
            self.confidence_callback,
            10
        )
        
        # Subscribe to boolean convenience topic
        self.is_person_sub = self.create_subscription(
            Bool,
            '/r2d2/perception/is_severin',
            self.is_severin_callback,
            10
        )
    
    def person_callback(self, msg: String):
        if msg.data == "severin":
            self.get_logger().info('Recognized Severin!')
        else:
            self.get_logger().info(f'Unknown person: {msg.data}')
    
    def confidence_callback(self, msg: Float32):
        self.get_logger().info(f'Confidence: {msg.data:.1f}')
    
    def is_severin_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Severin is present!')
        else:
            self.get_logger().info('Severin is not present')
```

---

## Part 2: Face Recognition Service

### Overview

The **face recognition service** is a background process that:
- Runs continuously on 10-15% CPU
- Processes frames at ~15 FPS with configurable frame skip
- Updates status in real-time (JSON file)
- Provides training pipeline for LBPH model
- Integrates with LED control systems
- Supports multi-user capability (can train multiple people)

### Quick Start

#### Start the Face Recognition Service

```bash
# Navigate to the service directory
cd /home/severin/dev/r2d2/tests/face_recognition

# Activate environment
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8

# Start the service (background)
python3 face_recognition_service.py start severin ~/dev/r2d2/data/face_recognition &
```

#### Check Service Status

```bash
# Quick status check
python3 face_recognition_service.py status

# Output:
# ======================================================================
# FACE RECOGNITION SERVICE STATUS
# ======================================================================
# Timestamp: 2025-12-06T16:47:28.123456
# ✅ RECOGNIZED: severin
# Frames: 1234
# Threshold: 70
# ======================================================================
```

#### Monitor in Real-Time

```bash
# Watch status continuously
watch -n 0.2 'python3 face_recognition_service.py status'
```

### Service Commands

#### Start Service

```bash
python3 face_recognition_service.py start <person_name> <data_dir>

# Example:
python3 face_recognition_service.py start severin ~/dev/r2d2/data/face_recognition
```

**Parameters:**
- `person_name`: Who to recognize (e.g., "severin")
- `data_dir`: Base directory with training data and models

#### Stop Service

```bash
python3 face_recognition_service.py stop

# Or kill the process
pkill -f "face_recognition_service.py"
```

#### Check Status

```bash
python3 face_recognition_service.py status

# Shows current recognition state, timestamp, frame count
```

#### View Logs

```bash
# Last 50 lines
python3 face_recognition_service.py logs 50

# Last 20 lines
python3 face_recognition_service.py logs 20

# Full log file
cat ~/.r2d2_face_recognition.log
```

### Status Monitoring

#### Real-Time Status File

The service writes status to a JSON file that updates every frame:

```
Location: ~/.r2d2_face_recognition_status.json
```

**Status File Format:**

```json
{
  "timestamp": "2025-12-06T16:47:28.123456",
  "recognized_person": "severin",
  "confidence_threshold": 70,
  "frame_count": 1234
}
```

**Fields:**
- `timestamp`: ISO 8601 timestamp of last status update
- `recognized_person`: Person name if recognized, `null` if not
- `confidence_threshold`: Current confidence threshold (lower = stricter)
- `frame_count`: Total frames processed since service start

#### Monitoring Methods

**Method 1: Command Line Status**
```bash
python3 face_recognition_service.py status
```
Best for: Quick checks, scripts

**Method 2: Watch JSON File**
```bash
watch -n 0.1 'cat ~/.r2d2_face_recognition_status.json | jq'

# Output updates every 100ms
{
  "timestamp": "2025-12-06T16:47:28.123456",
  "recognized_person": "severin",
  "confidence_threshold": 70,
  "frame_count": 1234
}
```
Best for: Detailed monitoring, debugging

**Method 3: Monitor Logs**
```bash
# Watch logs in real-time
tail -f ~/.r2d2_face_recognition.log

# Or with filtering
tail -f ~/.r2d2_face_recognition.log | grep "RECOGNIZED\|Error"
```
Best for: Troubleshooting, debugging

**Method 4: Visual Test Script**
```bash
# Run the quick visual test
python3 quick_status_test.py

# Shows status changes as they happen
# [16:47:51.820] ✅ DETECTED: SEVERIN
# [16:47:56.150] ❌ LOST: Recognition ended
```
Best for: Interactive testing, verification

### Recognition Behavior & State Management

#### Immediate Recognition
- Face appears in frame
- Status changes to `"recognized_person": "severin"` instantly (~67ms)
- No confirmation delay
- Per-frame detection

#### 5-Second Loss Persistence
- Face leaves frame
- Status stays `"recognized_person": "severin"` for 5 seconds
- Prevents flickering on brief movements
- After 5 seconds: changes to `"recognized_person": null`

**State Machine Timeline Example:**

```
T=0.0s:  Face enters frame
T=0.07s: recognized_person = "severin"          ← INSTANT
         Status shows: ✅ RECOGNIZED: SEVERIN

T=1.0s:  Face still visible
T=1.0s:  recognized_person = "severin"          ← UNCHANGED
         Status shows: ✅ RECOGNIZED: SEVERIN

T=2.0s:  Face leaves frame
T=2.0s:  recognized_person = "severin"          ← PERSISTENCE BEGINS
         Status shows: ✅ RECOGNIZED: SEVERIN

T=5.0s:  Still no face (at timeout edge)
T=5.0s:  recognized_person = "severin"          ← STILL RECOGNIZED
         Status shows: ✅ RECOGNIZED: SEVERIN

T=5.1s:  5 seconds have elapsed
T=5.1s:  recognized_person = null               ← TIMEOUT TRIGGERED
         Status shows: ❌ No one recognized

T=6.0s:  Face reappears
T=6.07s: recognized_person = "severin"          ← INSTANT AGAIN!
         Status shows: ✅ RECOGNIZED: SEVERIN
```

---

## Part 3: Training Pipeline

### Training Requirements

Before using face recognition, you need a trained LBPH model:

```
Required files:
├── ~/dev/r2d2/data/face_recognition/
│   ├── severin/              (training images directory)
│   │   ├── image_001.jpg
│   │   ├── image_002.jpg
│   │   └── ... (~80 images)
│   └── models/
│       └── severin_lbph.xml  (trained model)
```

### Training Process

#### Step 1: Capture Training Data

```bash
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8

python3 1_capture_training_data.py
```

**What it does:**
- Captures images from camera
- Saves to ~/dev/r2d2/data/face_recognition/severin/
- 4 tasks with different lighting/distances:
  - Task 1: Bright light, 1 meter (20 sec)
  - Task 2: Bright light, 2 meters (20 sec)
  - Task 3: Low light, 3 meters (20 sec)
  - Task 4: Low light, 5 meters (20 sec)
- Total: ~80 images, diverse angles

**Menu options:**
```
1. Train new person
2. Add more pictures to existing person
3. Retrain model from existing images
4. Test accuracy at different distances
5. Real-time recognition test (30 sec, instant feedback)
6. List all people and models
7. Delete person (safe deletion)
```

#### Step 2: Train the Model

```bash
# Auto-trains from captured images
python3 train_manager.py

# Select option 1 to train
```

Or directly:

```bash
python3 2_train_recognizer.py
```

**What it does:**
- Reads all images from ~/dev/r2d2/data/face_recognition/severin/
- Extracts faces using Haar Cascade
- Trains LBPH recognizer
- Saves model to ~/dev/r2d2/data/face_recognition/models/severin_lbph.xml
- Duration: 30-60 seconds

#### Step 3: Test the Model

```bash
python3 3_test_recognizer_demo.py
```

**What it shows:**
- Recognition accuracy statistics
- Confidence scores for test images
- Suggestions for improving accuracy

### Training Best Practices

#### Image Capture Tips

- **Lighting:** Vary lighting conditions (bright, low light, natural, artificial)
- **Angles:** Capture from different angles (straight, 45°, 90°)
- **Distance:** Vary distance from camera (1m, 2m, 3m, 5m)
- **Expressions:** Include different facial expressions (neutral, smile, looking down)
- **Occlusions:** Some images with partial occlusions (turned head, glasses)

#### Improving Recognition Accuracy

1. **Capture more images:** 80+ images is good, 150+ is better
2. **Diverse conditions:** Different lighting, angles, distances
3. **Lower threshold:** If too strict, adjust confidence_threshold downward
4. **Retrain regularly:** Add new images and retrain monthly

### Configuration Parameters

#### Confidence Threshold

```python
# Lower = stricter (requires higher confidence for match)
# Higher = more lenient (accepts weaker matches)

# Default: 70
confidence_threshold = 70

# Recommended ranges:
# 50-60:  Very strict (few false positives, some false negatives)
# 70:     Balanced (default)
# 80-90:  Lenient (may accept unknowns as severin)
```

Edit in `face_recognition_service.py` line ~71:

```python
def __init__(self, person_name, data_dir, confidence_threshold=70, cpu_limit=0.15):
```

#### CPU Limit & Frame Skip

```python
# Default: 0.15 (15%)
# Adjustable via frame skip

cpu_limit = 0.15  # 15% CPU usage

# This translates to:
skip_frames = int(1 / cpu_limit)  # 6 frames skip
# Processing at ~2.5 Hz (15 FPS * 1/6)

# Tuning:
# cpu_limit = 0.10  → skip = 10  → ~1.5 FPS recognition
# cpu_limit = 0.15  → skip = 6   → ~2.5 FPS recognition
# cpu_limit = 0.25  → skip = 4   → ~3.75 FPS recognition
```

---

## Part 4: LED Integration

### Status to LED Mapping

The service provides status via JSON file that can be consumed by LED controller:

```
recognized_person = "severin"  →  LED: Green (solid)
recognized_person = null       →  LED: Red (off) or Blue (idle)
```

### Reading Status from LED Controller

**Python Example:**

```python
import json
from pathlib import Path

# Read status file
status_file = Path.home() / '.r2d2_face_recognition_status.json'

def get_recognition_status():
    try:
        with open(status_file) as f:
            status = json.load(f)
            person = status.get('recognized_person')
            
            if person == 'severin':
                return 'RECOGNIZED'
            else:
                return 'NOT_RECOGNIZED'
    except:
        return 'ERROR'

# Use in LED control
if get_recognition_status() == 'RECOGNIZED':
    led.set_color('green')
else:
    led.set_color('red')
```

**Bash Example:**

```bash
#!/bin/bash

# Read recognized person from JSON
PERSON=$(cat ~/.r2d2_face_recognition_status.json | jq -r '.recognized_person')

if [ "$PERSON" = "severin" ]; then
    echo "GREEN"  # Send to LED controller
else
    echo "RED"    # Send to LED controller
fi
```

### LED Controller Architecture

The service includes a pluggable LED controller architecture:

```
led_controller.py:
├── Text Backend (current - for testing)
├── GPIO RGB Backend (ready to implement)
└── HTTP Backend (ready to implement)
```

See `led_controller.py` for implementation details.

---

## Performance Characteristics

### CPU Usage by Configuration

| Configuration | CPU | FPS | Use Case |
|---------------|-----|-----|----------|
| skip=1 | 18-25% | 13 Hz | Maximum accuracy, high CPU |
| skip=2 | 10-15% | 6.5 Hz | **Recommended** |
| skip=3 | 8-10% | 4.3 Hz | Minimal CPU, less responsive |
| skip=6 | 3-5% | 2.5 Hz | Very minimal, slow response |

### Recognition Latency

- **Detection latency:** ~67ms (1 frame at 15 FPS)
- **Loss confirmation:** ~5 seconds (configurable)
- **Status update frequency:** Every 500ms for display, every frame for JSON

### Memory Usage

- Service memory: ~200-300 MB
- Model file: 33.1 MB (severin_lbph.xml)
- Status file: <1 KB (JSON)

---

## Service Files & Locations

### Main Service

```
~/dev/r2d2/tests/face_recognition/face_recognition_service.py
├── FaceRecognitionService class
├── Commands: start, stop, status, logs
├── Configuration: threshold, CPU limit
└── Status file: ~/.r2d2_face_recognition_status.json
```

### Supporting Files

```
~/dev/r2d2/tests/face_recognition/
├── led_controller.py              (LED integration)
├── train_manager.py               (training menu)
├── interactive_training_simple.py (training script)
├── realtime_recognition_test_headless.py
├── quick_status_test.py           (visual test)
└── test_*.py                      (validation tests)
```

### Data Files

```
~/dev/r2d2/data/face_recognition/
├── severin/                       (training images)
│   ├── image_001.jpg
│   ├── image_002.jpg
│   └── ... (~80 images)
└── models/
    └── severin_lbph.xml          (trained model)
```

---

## Troubleshooting

### Issue: LBPH model not found

**Cause:** Trained model doesn't exist at specified path  
**Solution:**
1. Run training script first:
   ```bash
   cd ~/dev/r2d2/tests/face_recognition
   python3 1_capture_training_data.py
   python3 2_train_recognizer.py
   ```
2. Verify model exists:
   ```bash
   ls -la ~/dev/r2d2/data/face_recognition/models/
   ```

### Issue: ROS 2 Topics not publishing

**Cause:** Face recognition disabled or no faces detected  
**Solution:**
1. Check that recognition is enabled:
   ```bash
   ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
     enable_face_recognition:=true
   ```
2. Verify faces are being detected:
   ```bash
   ros2 topic echo /r2d2/perception/face_count
   ```

### Issue: Always returning "unknown" even for Severin

**Cause:** Training data not representative of test conditions  
**Solution:**
1. Retrain with more diverse images (different angles, lighting)
2. Lower the threshold to accept weaker matches:
   ```bash
   ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
     enable_face_recognition:=true \
     recognition_confidence_threshold:=80.0
   ```

### Issue: Service won't start

**Symptom:** Command hangs or shows errors

**Solution:**
1. Check environment is activated:
   ```bash
   source ~/depthai_env/bin/activate
   export OPENBLAS_CORETYPE=ARMV8
   ```

2. Verify model exists:
   ```bash
   ls -la ~/dev/r2d2/data/face_recognition/models/severin_lbph.xml
   ```

3. Check camera access:
   ```bash
   python3 -c "import depthai; print('Camera OK')"
   ```

### Issue: Service always shows "No one recognized"

**Symptom:** Service runs but never recognizes severin

**Solutions:**
1. **Check confidence threshold is not too strict:**
   ```bash
   # Try lower threshold (less strict)
   # Edit face_recognition_service.py, change default from 70 to 80-85
   ```

2. **Check training data quality:**
   ```bash
   python3 3_test_recognizer_demo.py
   ```

3. **Add more training images:**
   ```bash
   python3 train_manager.py  # Select option 2
   ```

4. **Retrain with new images:**
   ```bash
   python3 train_manager.py  # Select option 3
   ```

### Issue: CPU usage too high

**Symptom:** Service using 20%+ CPU

**Solution: Increase frame skip**

```bash
# Current default: skip_frames = 6 (15 FPS)
# Edit face_recognition_service.py line ~72:

self.skip_frames = int(1 / cpu_limit)  # Adjust cpu_limit

# For 10% CPU: set cpu_limit = 0.10 (skip = 10)
# For 5% CPU: set cpu_limit = 0.05 (skip = 20)
```

### Issue: Status file not updating

**Symptom:** `~/.r2d2_face_recognition_status.json` is stale

**Solution:**
1. Check service is running:
   ```bash
   ps aux | grep face_recognition_service.py
   ```

2. Check file permissions:
   ```bash
   ls -la ~/.r2d2_face_recognition_status.json
   ```

3. Restart service:
   ```bash
   pkill -f face_recognition_service.py
   python3 face_recognition_service.py start severin ~/dev/r2d2/data/face_recognition
   ```

### Issue: Logs not being written

**Symptom:** `~/.r2d2_face_recognition.log` is empty or missing

**Solution:**
1. Check permissions:
   ```bash
   touch ~/.r2d2_face_recognition.log
   chmod 666 ~/.r2d2_face_recognition.log
   ```

2. Restart service and verify logging:
   ```bash
   pkill -f face_recognition_service.py
   python3 face_recognition_service.py start severin ~/dev/r2d2/data/face_recognition
   
   # Check logs
   tail -f ~/.r2d2_face_recognition.log
   ```

---

## Quick Command Reference

### ROS 2 Integration

```bash
# Launch without recognition (default)
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py

# Launch with recognition
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py enable_face_recognition:=true

# Monitor person ID
ros2 topic echo /r2d2/perception/person_id

# Monitor confidence
ros2 topic echo /r2d2/perception/face_confidence

# Check frequency
ros2 topic hz /r2d2/perception/person_id
```

### Service Management

```bash
# Start service
python3 face_recognition_service.py start severin ~/dev/r2d2/data/face_recognition

# Stop service
pkill -f face_recognition_service.py

# Check status
python3 face_recognition_service.py status

# View logs
python3 face_recognition_service.py logs 50
```

### Training

```bash
# Capture images
python3 1_capture_training_data.py

# Train model
python3 2_train_recognizer.py

# Test model
python3 3_test_recognizer_demo.py

# Full menu
python3 train_manager.py
```

### Monitoring

```bash
# Visual test
python3 quick_status_test.py

# Watch service status
watch -n 0.2 'python3 face_recognition_service.py status'

# Watch JSON status
watch -n 0.1 'cat ~/.r2d2_face_recognition_status.json | jq'

# Watch logs
tail -f ~/.r2d2_face_recognition.log
```

---

## LBPH (Local Binary Pattern Histograms) Recognizer

### Why LBPH?

**Comparison: LBPH vs. Deep Learning Detectors**

```
┌─────────────────┬──────────────────────┬──────────────┬──────────────┐
│ Recognizer      │ Speed (ARM/Jetson)   │ Accuracy     │ Model Size   │
├─────────────────┼──────────────────────┼──────────────┼──────────────┤
│ LBPH            │ ~5-10ms per face     │ Good (80%)   │ 33 MB        │
│ FaceNet         │ ~100-150ms per face  │ Better (92%) │ 100+ MB      │
│ ArcFace         │ ~150-200ms per face  │ Excellent    │ 80+ MB       │
│ VGGFace2        │ ~200-300ms per face  │ Excellent    │ 200+ MB      │
└─────────────────┴──────────────────────┴──────────────┴──────────────┘
```

**Chosen for:**
- ✅ CPU-efficient (no GPU needed)
- ✅ Fast enough for real-time on ARM (Jetson)
- ✅ Works on standard Jetson without CUDA
- ✅ Acceptable accuracy for personal recognition
- ✅ Mature, stable OpenCV implementation
- ✅ Small model size (33 MB)
- ✅ Works well with limited training data

### Recognition Process

1. **Face Detection:** Haar Cascade on 640×360 grayscale image (~13 Hz)
2. **Face Extraction:** Crop detected face to ROI
3. **Face Resize:** Standardize to 100×100 (training requirement)
4. **LBPH Recognition:** Predict label + confidence
5. **Publishing:** Output to `/r2d2/perception/person_id`, etc. (~6 Hz with frame_skip=2)

---

## Integration with Other Systems

### Audio Notifications Integration

The face recognition system integrates with audio notifications:

```
Face Recognition (/r2d2/perception/person_id)
    ↓
Audio Notification Node (subscribes to person_id)
    ↓
Triggers beeps (see [`060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md`](060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md))
```

**Example:** When person changes from "unknown" → "severin", audio node triggers recognition beep.

### LED Control Integration

The service provides JSON status for LED integration:

```
face_recognition_service.py
    ↓
~/.r2d2_face_recognition_status.json
    ↓
LED Controller
    ↓
RGB LED (Green = recognized, Red = unknown)
```

---

## Future Enhancements

### Short Term
- Test recognition at various distances (1m-5m)
- Fine-tune threshold based on real performance
- Collect more training data for improved accuracy
- Multi-person recognition (not just Severin)

### Medium Term
- Store recognition confidence history
- Add simple motion/tracking to link detections across frames
- Emotion recognition integration
- Voice-based person identification

### Long Term
- Migrate to deep learning (FaceNet, ArcFace) for better accuracy
- GPU acceleration via CUDA
- Integration with dialogue/interaction system
- Multi-modal identification (face + voice + gesture)

---

## Files Modified/Created

### Created:
```
~/dev/r2d2/tests/face_recognition/
├── 1_capture_training_data.py
├── 2_train_recognizer.py
├── 3_test_recognizer_demo.py
├── face_recognition_service.py
├── led_controller.py
├── train_manager.py
├── quick_status_test.py
└── interactive_training_simple.py

~/dev/r2d2/data/face_recognition/
├── severin/  (training images - created during capture)
└── models/   (trained models - created during training)
```

### Modified:
```
~/dev/r2d2/ros2_ws/src/r2d2_perception/r2d2_perception/image_listener.py
  - Added: LBPH recognition logic
  - Added: person_id, face_confidence, is_severin publishers
  - Added: face_recognition parameters

~/dev/r2d2/ros2_ws/src/r2d2_perception/launch/perception.launch.py
  - Added: enable_face_recognition argument
  - Added: recognition configuration parameters

~/dev/r2d2/ros2_ws/src/r2d2_bringup/launch/r2d2_camera_perception.launch.py
  - Added: enable_face_recognition argument declaration
  - Added: pass-through of recognition parameters
```

---

## References

**Related Documentation:**
- [`020_CAMERA_SETUP_DOCUMENTATION.md`](020_CAMERA_SETUP_DOCUMENTATION.md) - OAK-D setup
- [`030_PERCEPTION_PIPELINE_SETUP.md`](030_PERCEPTION_PIPELINE_SETUP.md) - Perception pipeline
- Archived face detection documentation in `_ARCHIVED_FACE_RECOGNITION_DOCS_v1/`
- [`060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md`](060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md) - Audio integration

**External References:**
- OpenCV LBPH: https://docs.opencv.org/master/df/d25/classcv_1_1face_1_1LBPHFaceRecognizer.html
- ROS 2 Humble: https://docs.ros.org/en/humble/

---

## Summary

Face recognition on R2D2 is **production-ready** with two complementary systems:

1. **ROS 2 Integration:** Real-time person identification in perception pipeline, published on standard ROS 2 topics
2. **Service System:** Background training and monitoring service with JSON status file for external integrations

Together, these provide:
- ✅ Real-time personal recognition at 6-13 Hz
- ✅ Training pipeline for model generation
- ✅ Status monitoring and LED integration
- ✅ Comprehensive error handling
- ✅ Integration with audio and other systems

**Ready to use!** See "Quick Start" and "Training Process" sections above.

---

**Document Version:** 2.0 (Consolidated)  
**Last Updated:** December 8, 2025  
**Status:** ✅ Production Ready
