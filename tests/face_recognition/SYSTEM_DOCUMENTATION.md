# Face Recognition Service - Complete System Documentation

## Overview

This is a complete face recognition system for R2D2 that:
- Runs as a background service with 10-15% CPU usage
- Provides real-time recognition feedback
- Maintains a 5-second recognition timeout
- Architecture ready for LED control integration
- Includes multiple menu-driven training interfaces
- Protects training data from accidental overwrites

## Architecture

```
┌─────────────────────────────────────────────────────┐
│          face_recognition_service.py                │
│      (Background continuous recognition)            │
└──────────────┬──────────────────────────────────────┘
               │
               ├─ Camera: OAK-D Lite
               │           (1280×720 @ 15 FPS)
               │
               ├─ Face Detection: OpenCV Haar Cascade
               │
               ├─ Face Recognition: LBPH Model
               │
               ├─ Status Output: LED Controller
               │                 ├─ Text (console)
               │                 ├─ GPIO (RGB LED)
               │                 └─ HTTP (network LED)
               │
               └─ IPC Status File: ~/.r2d2_face_recognition_status.json
                                    (for external processes, LED integration)

┌─────────────────────────────────────────────────────┐
│            train_manager.py                         │
│        (Menu-driven training & testing)             │
└──────────────────────────────────────────────────────┘
   │
   ├─ [1] Train new person
   │       └─ interactive_training_simple.py
   │           (4 tasks: 1m, 2m, 3m, 5m)
   │
   ├─ [2] Add additional training pictures
   │
   ├─ [3] Train model from existing images
   │
   ├─ [4] Test trained model (accuracy)
   │
   ├─ [5] Real-time recognition test
   │       └─ realtime_recognition_test_headless.py
   │           (30-second live feedback)
   │
   ├─ [6] List all people and models
   │
   └─ [7] Delete person
```

## System Components

### 1. Face Recognition Service (`face_recognition_service.py`)

**Purpose:** Continuous background face recognition with optional LED control

**Key Features:**
- CPU limiting via frame skipping (10-15% configurable)
- 5-second recognition timeout
- JSON status file for inter-process communication
- Comprehensive logging
- Multiple LED backend support (text, GPIO, HTTP)
- Thread-safe status management

**Usage:**
```bash
# Start service
python3 face_recognition_service.py start severin ~/dev/r2d2/data/face_recognition

# Check status
python3 face_recognition_service.py status

# View logs
python3 face_recognition_service.py logs 50

# Stop service
python3 face_recognition_service.py stop
```

**Camera Settings:**
- Resolution: 1280×720 (reduced from 1920×1080 for CPU efficiency)
- FPS: 15 (reduced from 30 for CPU efficiency)
- Frame skip: 6 (process every 6th frame at 15% CPU limit)

**Status File Format:**
```json
{
  "timestamp": "2024-01-15T10:30:45.123456",
  "recognized_person": "severin",
  "confidence_threshold": 70,
  "frame_count": 1234
}
```

When no one is recognized, `recognized_person` is `null`.

### 2. Training Manager (`train_manager.py`)

**Purpose:** Central hub for all training and testing operations

**Menu Options:**

```
[1] Train new person
    - Run interactive training (4 tasks)
    - Automatically train model
    - Run real-time test
    - Save person data
    
[2] Add additional training pictures
    - Add more images to existing person
    - Retrain model
    - Test new model
    
[3] Train model from existing images
    - Retrain model from disk images
    - Useful after adding more pictures
    
[4] Test trained model (accuracy)
    - Shows recognition metrics
    - Tests at different distances (1m, 2m, 3m, 5m)
    
[5] Real-time recognition test (INSTANT FEEDBACK!)
    - 30-second live test
    - Shows "RECOGNIZED: person" or "Unknown" every frame
    - Uses confidence threshold 70
    
[6] List all people and models
    - Shows all trained people
    - Shows model sizes
    
[7] Delete person
    - Removes all training images
    - Removes trained model
    - Requires confirmation
```

**Data Protection:**
- Training images are NOT tracked by git (protected by `.gitignore`)
- Models ARE tracked by git (for easy deployment)
- Overwrite confirmation when reusing person names
- Prevents accidental data loss

### 3. Interactive Training (`interactive_training_simple.py`)

**Purpose:** Simple 4-task training system with different distances and lighting

**Tasks:**
1. **Bright Light - 1 meter**: Head movements right, left, up, down (20 sec)
2. **Bright Light - 2 meters**: Head movements right, left, up, down (20 sec)
3. **Low Light - 3 meters**: Head movements right, left, up, down (20 sec)
4. **Low Light - 5 meters**: Head movements right, left, up, down (20 sec)

**Total:** ~80 images per person, diverse distances and lighting

### 4. Real-time Recognition Test (`realtime_recognition_test_headless.py`)

**Purpose:** 30-second continuous test with instant visual feedback

**Features:**
- Shows "RECOGNIZED: person" instantly when detected
- Shows "Unknown" when not recognized
- Configurable duration and threshold
- Updates every frame (no delay)

**Usage:**
```bash
python3 realtime_recognition_test_headless.py \
  --person severin \
  --data-dir ~/dev/r2d2/data/face_recognition \
  --duration 30 \
  --threshold 70
```

### 5. LED Controller (`led_controller.py`)

**Purpose:** Pluggable architecture for different LED backends

**Supported Backends:**

#### Text LED (Current)
```python
led = create_led_controller('text')
led.set_recognized('severin')  # Prints: ✅ RECOGNIZED: SEVERIN
led.set_unrecognized()         # Prints: ❌ No one recognized
```

#### GPIO-based RGB LED (Planned)
```python
led = create_led_controller('gpio', 
                            red_pin=17, 
                            green_pin=27, 
                            blue_pin=22)
led.set_recognized('severin')  # Green LED
led.set_unrecognized()         # Red LED
led.set_error()                # Yellow LED
```

#### Network-based LED (Planned)
```python
led = create_led_controller('http', 
                            base_url='http://192.168.1.100:5000')
led.set_recognized('severin')  # GET /api/led?status=recognized&person=severin
```

### 6. Systemd Service (`r2d2-face-recognition.service`)

**Purpose:** Auto-start face recognition service on system boot

**Installation:**
```bash
sudo cp r2d2-face-recognition.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable r2d2-face-recognition
sudo systemctl start r2d2-face-recognition
```

**Management:**
```bash
# Check status
sudo systemctl status r2d2-face-recognition

# View logs
sudo journalctl -u r2d2-face-recognition -f

# Restart
sudo systemctl restart r2d2-face-recognition

# Stop
sudo systemctl stop r2d2-face-recognition
```

## Quick Start

### 1. Start the Menu System
```bash
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8
python3 train_manager.py
```

### 2. Train a New Person
```
Enter option: 1
Enter person name: severin
[Runs interactive training with 4 tasks]
[Automatically trains model]
[Runs 30-second real-time test]
```

### 3. Start Background Service
```bash
python3 face_recognition_service.py start severin ~/dev/r2d2/data/face_recognition
```

### 4. Check Service Status
In another terminal:
```bash
python3 face_recognition_service.py status
```

### 5. View Service Logs
```bash
python3 face_recognition_service.py logs 50
```

## Configuration

### CPU Usage Control
Edit `face_recognition_service.py` and change the `cpu_limit` parameter:
```python
service = FaceRecognitionService(
    person_name='severin',
    data_dir='~/dev/r2d2/data/face_recognition',
    cpu_limit=0.15  # 10-15% (adjust 0.10-0.20)
)
```

### Confidence Threshold
The system uses confidence threshold 70 (empirically determined to be optimal).
To adjust:
```python
service = FaceRecognitionService(
    confidence_threshold=70  # Lower = stricter, 40-100
)
```

### Recognition Timeout
Default is 5 seconds. To change:
```python
self.recognition_timeout = 5  # seconds
```

## Data Structure

```
~/dev/r2d2/
├── data/
│   └── face_recognition/
│       ├── severin/              ← Training images (not tracked by git)
│       │   ├── task1_bright_1m_*.jpg
│       │   ├── task2_bright_2m_*.jpg
│       │   ├── task3_low_3m_*.jpg
│       │   └── task4_low_5m_*.jpg
│       └── models/
│           └── severin_lbph.xml  ← Trained model (tracked by git)
├── tests/
│   └── face_recognition/
│       ├── face_recognition_service.py
│       ├── led_controller.py
│       ├── train_manager.py
│       ├── interactive_training_simple.py
│       ├── realtime_recognition_test_headless.py
│       ├── r2d2-face-recognition.service
│       └── [analysis tools]
└── .gitignore
    ├── /data/face_recognition/*/   ← Exclude training images
    └── *.pyc
```

## Performance Characteristics

### Current Setup (387 training images)
- **Recognition Accuracy:** ~45% (at optimal threshold 70)
- **CPU Usage:** 10-15%
- **Response Time:** ~500ms (status updates every 500ms)
- **Recognition Timeout:** 5 seconds
- **Model Size:** 33.9 MB

### Optimization Tips
1. **More training images** → Higher accuracy
2. **Varied lighting/distance** → Better generalization
3. **Confidence threshold 70** → Optimal balance (empirically tested)
4. **Lower FPS** → Lower CPU usage (currently 15 FPS, could go to 10)
5. **Smaller resolution** → Lower CPU (currently 1280×720, could go to 640×360)

## Troubleshooting

### Service not starting?
```bash
# Check logs
python3 face_recognition_service.py logs 100

# Verify model exists
ls -la ~/dev/r2d2/data/face_recognition/models/severin_lbph.xml

# Test camera
python3 -c "import depthai as dai; print('Camera OK')"
```

### High CPU usage?
- Increase frame skip: `cpu_limit=0.10` → skip more frames
- Reduce resolution further: 640×360
- Reduce FPS: 10 instead of 15

### Low recognition accuracy?
- Train with more images (target 200+)
- Add more variety (different angles, lighting)
- Adjust confidence threshold lower (60-65)

### LED not working?
- Check `led_controller.py` has no errors
- Verify GPIO pins if using GPIO backend
- Check network connectivity if using HTTP backend

## Future Enhancements

### Phase 1: GPIO LED (Next)
- [x] Architecture ready
- [ ] Test with actual RGB LED
- [ ] GPIO pin configuration
- [ ] Color state mapping

### Phase 2: ROS 2 Integration
- [ ] Published recognition topic
- [ ] Subscribed to LED control
- [ ] Integration with robot decision-making

### Phase 3: Advanced Features
- [ ] Multiple person recognition
- [ ] Confidence score reporting
- [ ] Web dashboard
- [ ] Cloud backup of models
- [ ] Automatic retraining

## Key Learnings

1. **Frame skipping > resolution reduction** for CPU efficiency
   - Frame skip: Minimal latency, smooth operation
   - Resolution: Affects accuracy, noticeable quality loss

2. **Confidence threshold 70** is optimal
   - Tested thresholds 40-100
   - 70 gives best precision/recall balance

3. **5-second timeout** feels natural
   - Not too fast: Avoids flickering
   - Not too slow: Responds quickly to person leaving

4. **More diverse training data > More images**
   - Different distances (1m, 2m, 3m, 5m)
   - Different lighting (bright, low)
   - Different angles (right, left, up, down)

5. **OPENBLAS_CORETYPE=ARMV8** is critical
   - Without it: Illegal instruction errors
   - With it: Works perfectly on Jetson ARM

## Environment Setup

```bash
# Activate environment
source ~/depthai_env/bin/activate

# Set critical environment variable
export OPENBLAS_CORETYPE=ARMV8

# Run system
python3 train_manager.py
```

## Support

For issues or questions, check:
1. Logs: `python3 face_recognition_service.py logs 50`
2. Camera: `python3 -c "import depthai as dai; print('OK')"`
3. Model: `ls -la ~/dev/r2d2/data/face_recognition/models/`
4. Status: `python3 face_recognition_service.py status`

---

**Created:** 2024-01-15
**System:** NVIDIA Jetson AGX Orin 64GB, Ubuntu 22.04, JetPack 6.x
**Camera:** OAK-D Lite
**Python:** 3.10.6 in ~/depthai_env
