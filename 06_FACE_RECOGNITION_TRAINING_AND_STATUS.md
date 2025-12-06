# Face Recognition Training and Status Monitoring

**Date:** December 6, 2025  
**Status:** ✅ Production Ready  
**Platform:** NVIDIA Jetson AGX Orin 64GB

---

## Table of Contents

1. [Quick Start](#quick-start)
2. [Service Initialization](#service-initialization)
3. [Status Monitoring](#status-monitoring)
4. [Training Pipeline](#training-pipeline)
5. [LED Integration](#led-integration)
6. [Troubleshooting](#troubleshooting)

---

## Quick Start

### Start the Face Recognition Service

```bash
# Navigate to the service directory
cd /home/severin/dev/r2d2/tests/face_recognition

# Activate environment
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8

# Start the service (background)
python3 face_recognition_service.py start severin ~/dev/r2d2/data/face_recognition &
```

### Check Service Status

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

### Monitor in Real-Time

```bash
# Watch status continuously
watch -n 0.2 'python3 face_recognition_service.py status'
```

---

## Service Initialization

### Understanding the Service Architecture

The `face_recognition_service.py` is a **background service** that:
- Runs continuously on 10-15% CPU
- Processes frames at ~15 FPS with 6-frame skip (2.5 FPS recognition)
- Updates status in real-time (every 500ms for display)
- Stores status in JSON file for inter-process communication

### Service Startup Sequence

```
1. Initialize service
   └─ Load configuration (person_name, data_dir, thresholds)

2. Initialize camera
   └─ 1280x720 resolution
   └─ 15 FPS frame rate
   └─ Frame skip: 6 (to limit CPU to 15%)

3. Load face detector
   └─ Haar Cascade (haarcascade_frontalface_default.xml)
   └─ OpenCV built-in

4. Load recognizer model
   └─ LBPH (Local Binary Pattern Histograms)
   └─ Path: ~/dev/r2d2/data/face_recognition/models/severin_lbph.xml

5. Start main loop
   └─ Capture frames
   └─ Skip frames for CPU efficiency
   └─ Detect faces
   └─ Recognize faces
   └─ Update status file
   └─ Display status (every 500ms)
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

---

## Status Monitoring

### Real-Time Status File

The service writes status to a JSON file that updates every frame:

```
Location: ~/.r2d2_face_recognition_status.json
```

### Status File Format

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

### Monitoring Methods

#### Method 1: Command Line Status

```bash
python3 face_recognition_service.py status
```

Best for: Quick checks, scripts

#### Method 2: Watch JSON File

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

#### Method 3: Monitor Logs

```bash
# Watch logs in real-time
tail -f ~/.r2d2_face_recognition.log

# Or with filtering
tail -f ~/.r2d2_face_recognition.log | grep "RECOGNIZED\|Error"
```

Best for: Troubleshooting, debugging

#### Method 4: Visual Test Script

```bash
# Run the quick visual test
python3 quick_status_test.py

# Shows status changes as they happen
# [16:47:51.820] ✅ DETECTED: SEVERIN
# [16:47:56.150] ❌ LOST: Recognition ended
```

Best for: Interactive testing, verification

### Recognition Behavior

#### Immediate Recognition (NEW!)
- Face appears in frame
- Status changes to `"recognized_person": "severin"` instantly (~67ms)
- No confirmation delay
- Per-frame detection

#### 5-Second Loss Persistence
- Face leaves frame
- Status stays `"recognized_person": "severin"` for 5 seconds
- Prevents flickering on brief movements
- After 5 seconds: changes to `"recognized_person": null`

#### Timeline Example

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

## Training Pipeline

### Training Requirements

Before using the service, you need a trained LBPH model:

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

#### CPU Limit

```python
# Default: 0.15 (15%)
# Adjustable via frame skip

cpu_limit = 0.15  # 15% CPU usage

# This translates to:
skip_frames = int(1 / cpu_limit)  # 6 frames skip
# Processing at ~2.5 Hz (15 FPS * 1/6)
```

---

## LED Integration

### Status to LED Mapping

The service provides status via JSON file that can be consumed by LED controller:

```
recognized_person = "severin"  →  LED: Green (solid)
recognized_person = null       →  LED: Red (off) or Blue (idle)
```

### Reading Status from LED Controller

#### Python Example

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

#### Bash Example

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

### LED Controller Integration

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

### CPU Usage

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

## Service Files

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

### Issue: Always shows "No one recognized"

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

### Start/Stop

```bash
# Start
python3 face_recognition_service.py start severin ~/dev/r2d2/data/face_recognition

# Stop
pkill -f face_recognition_service.py

# Status
python3 face_recognition_service.py status
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

# Watch status
watch -n 0.2 'python3 face_recognition_service.py status'

# Watch JSON
watch -n 0.1 'cat ~/.r2d2_face_recognition_status.json | jq'

# Watch logs
tail -f ~/.r2d2_face_recognition.log
```

---

## Next Steps

1. **Train the model:** Run `1_capture_training_data.py` and `2_train_recognizer.py`
2. **Test the service:** Run `quick_status_test.py` and verify behavior
3. **Integrate with LED:** Use status JSON to drive LED colors
4. **Deploy to production:** Install systemd service for auto-start

---

**Questions?** Check the other documentation files:
- `04_FACE_RECOGNITION_INTEGRATION.md` - ROS 2 integration
- `QUICK_START.md` - Getting started guide
- `SYSTEM_DOCUMENTATION.md` - Overall system architecture
