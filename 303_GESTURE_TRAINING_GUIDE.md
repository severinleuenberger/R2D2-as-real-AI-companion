# Gesture Training Guide - Person-Specific Gesture Recognition

**Date:** December 17, 2025  
**Status:** Production Ready  
**System:** R2D2 Perception Pipeline  
**Purpose:** Train person-specific gesture classifiers for conversation triggering

**Related Documentation:**
- [300_GESTURE_SYSTEM_OVERVIEW.md](300_GESTURE_SYSTEM_OVERVIEW.md) - Complete system architecture
- [250_PERSON_MANAGEMENT_SYSTEM_REFERENCE.md](250_PERSON_MANAGEMENT_SYSTEM_REFERENCE.md) - Person entity management

---

## Overview

The gesture training system allows you to train person-specific gesture recognition models for two gestures:
- **Index Finger Up**: Pointing upward (triggers conversation start)
- **Fist**: All fingers closed (triggers conversation stop)

Gesture models are person-specific and organized the same way as face recognition models.

---

## Quick Start

### Prerequisites

1. **Virtual Environment:**
   ```bash
   source ~/depthai_env/bin/activate
   export OPENBLAS_CORETYPE=ARMV8
   ```

2. **Dependencies:**
   - MediaPipe: `pip install mediapipe`
   - scikit-learn: `pip install scikit-learn`
   - OpenCV, DepthAI (already installed from face recognition)

### Launch Training Manager

```bash
cd ~/dev/r2d2/tests/face_recognition
python3 train_manager.py
```

Select option **[8]** for complete gesture training workflow.

---

## Directory Structure

```
~/dev/r2d2/data/
├── face_recognition/              # Face recognition data
│   ├── {person_name}/
│   └── models/
│       └── {person_name}_lbph.xml
└── gesture_recognition/          # NEW - Gesture recognition data
    ├── {person_name}/            # Person-specific gestures
    │   ├── index_finger_up/     # Gesture 1 images
    │   └── fist/                # Gesture 2 images
    └── models/
        └── {person_name}_gesture_classifier.pkl
```

---

## Menu Options

### Face Recognition (Options 1-7)
- Same as before, unchanged

### Gesture Recognition (Options 8-13)

**[8] Train gestures for person (capture + train + test)**
- Complete workflow: capture → train → test
- Recommended for first-time training
- Takes ~15-20 minutes total

**[9] Add additional gesture pictures**
- Add more images to existing dataset
- Improves model accuracy
- Useful after initial training

**[10] Train gesture model from existing images**
- Retrain classifier on existing images
- Use after adding more pictures

**[11] Test gesture classifier (real-time)**
- 30-second real-time test
- Shows recognition accuracy
- Verifies model performance

**[12] List all gesture datasets and models**
- View all people with gesture data
- Check image counts per gesture
- View model sizes

**[13] Delete person gestures (images + model)**
- Remove all gesture data for a person
- Requires YES confirmation
- Leaves face recognition data untouched

---

## Complete Training Workflow

### Step 1: Train Face Recognition (Optional but Recommended)

```bash
python3 train_manager.py
# Select [1] Train new person
# Follow prompts to capture face images
```

**Why first?** Face recognition identifies who is present. Gestures are person-specific and only recognized when the target person is detected.

### Step 2: Train Gestures

```bash
python3 train_manager.py
# Select [8] Train gestures for person
# Enter person name (must match face recognition if used)
```

**Capture Process (2 gestures × 15 seconds each):**

1. **Index Finger Up:**
   - Hold index finger pointing upward
   - Keep other fingers closed or relaxed
   - Move hand slowly (left/right, up/down)
   - MediaPipe validates gesture in real-time
   - Saves ~30-40 validated images

2. **Fist:**
   - Close all fingers into fist
   - Keep fist closed throughout
   - Move hand slowly (left/right, up/down)
   - MediaPipe validates gesture in real-time
   - Saves ~30-40 validated images

**Training Process (~2-3 minutes):**
- Loads captured images
- Extracts hand landmarks using MediaPipe (21 landmarks × 3 coords = 63 features)
- Normalizes features (scale/position invariant)
- Trains SVM classifier
- Saves model: `{person_name}_gesture_classifier.pkl`

**Testing Process (30 seconds):**
- Real-time gesture recognition
- Shows detected gestures with confidence
- Reports accuracy statistics
- Recommendations for improvement

---

## Usage Examples

### Example 1: Train Gestures for "target_person"

```bash
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8
python3 train_manager.py

# Select [8] Train gestures for person
# Enter: target_person
# Follow capture instructions
# Review training results
# Test in real-time
```

**Result:**
```
~/dev/r2d2/data/gesture_recognition/
├── target_person/
│   ├── index_finger_up/  (35 images)
│   └── fist/            (38 images)
└── models/
    └── target_person_gesture_classifier.pkl (12.5 KB)
```

### Example 2: Add More Pictures (Improve Accuracy)

```bash
python3 train_manager.py
# Select [9] Add additional gesture pictures
# Select person from list
# Capture more images (adds to existing)
# Select [10] Train gesture model from existing images
# Select [11] Test gesture classifier
```

---

## Technical Details

### Gesture Capture Module (`_gesture_capture_module.py`)

**Features:**
- Uses MediaPipe Hands for real-time hand detection
- Validates gestures during capture:
  - Index finger up: Index tip above MCP, other fingers down
  - Fist: All finger tips below MCPs
- Saves only validated gesture images
- Person-specific directory structure
- ~30-40 images per gesture (15 seconds × 2-3 FPS save rate)

### Gesture Training Module (`_gesture_train_module.py`)

**Features:**
- MediaPipe landmark extraction (21 landmarks × 3D coords = 63 features)
- Normalization:
  - Translate to wrist origin (position invariant)
  - Scale by hand size (scale invariant)
- SVM classifier (RBF kernel, probability estimates)
- StandardScaler for feature normalization
- Model saved with metadata (person name, training date, gestures)

### Gesture Test Module (`_gesture_test_module.py`)

**Features:**
- Real-time gesture recognition (30 seconds)
- MediaPipe Hands + trained SVM classifier
- Confidence threshold: 0.6 (60%)
- Displays detected gestures with confidence
- Statistics: detection rate, recognition rate, gesture distribution
- Recommendations for improvement

---

## Troubleshooting

### Issue: MediaPipe not installed

```bash
source ~/depthai_env/bin/activate
pip install mediapipe
```

### Issue: scikit-learn not installed

```bash
pip install scikit-learn
```

### Issue: Low recognition accuracy (<50%)

**Solutions:**
1. Add more training images (option [9])
2. Ensure clear gestures during capture
3. Vary hand positions/angles during capture
4. Retrain model after adding images (option [10])

### Issue: Hand not detected during capture

**Solutions:**
1. Move hand closer to camera (arm's length)
2. Ensure good lighting
3. Keep hand in camera frame
4. Make gesture more pronounced

### Issue: Gesture validation fails

**Solutions:**
1. For index finger up: Make sure ONLY index finger is extended
2. For fist: Close ALL fingers tightly into palm
3. Hold gesture steady (don't rush)

---

## Integration with ROS 2 Perception Node

Once trained, gesture models will be loaded by the perception node:

**Model Path:**
```
~/dev/r2d2/data/gesture_recognition/models/{person_name}_gesture_classifier.pkl
```

**Usage in Perception Node:**
```python
# Load model
with open(model_path, 'rb') as f:
    model_data = pickle.load(f)

classifier = model_data['classifier']
scaler = model_data['scaler']
label_to_gesture = model_data['label_to_gesture']

# Extract landmarks from frame
landmarks = extract_hand_landmarks(frame)
landmarks_normalized = normalize_landmarks(landmarks)

# Predict gesture
features_scaled = scaler.transform([landmarks_normalized])
prediction = classifier.predict(features_scaled)[0]
confidence = max(classifier.predict_proba(features_scaled)[0])

gesture_name = label_to_gesture[prediction]
```

**Gating:**
- Gesture recognition only active when `person_id == person_name`
- Prevents false triggers from other people

---

## Performance Metrics

### Expected Values

| Metric | Expected | Notes |
|--------|----------|-------|
| Images per gesture | 30-40 | 15 seconds capture |
| Training time | 2-3 minutes | Depends on image count |
| Model size | 10-15 KB | SVM + scaler + metadata |
| Recognition accuracy | 70-90% | With good training data |
| Real-time performance | ~15 FPS | MediaPipe + SVM inference |

### CPU Usage (Jetson AGX Orin)

| Operation | CPU Usage | Notes |
|-----------|-----------|-------|
| Capture | ~10-15% | MediaPipe Hands |
| Training | ~30-40% | SVM training (brief) |
| Testing | ~10-15% | Real-time recognition |

---

## File Reference

### Modules

```
~/dev/r2d2/tests/face_recognition/
├── train_manager.py                    # Main manager (UPDATED)
├── _gesture_capture_module.py          # NEW - Gesture capture
├── _gesture_train_module.py            # NEW - Gesture training
└── _gesture_test_module.py             # NEW - Gesture testing
```

### Data Directories

```
~/dev/r2d2/data/gesture_recognition/
├── {person_name}/
│   ├── index_finger_up/
│   └── fist/
└── models/
    └── {person_name}_gesture_classifier.pkl
```

---

## Next Steps

1. **Train gestures for target person** (option [8])
2. **Test gesture classifier** (option [11])
3. **Verify accuracy** (>70% recommended)
4. **Deploy to ROS 2 perception node** (see main plan)
5. **Integrate with gesture intent node** (triggers conversation)

---

## Person Registry Integration

### Automatic Registration

Trained gestures are automatically integrated with the Person Registry system:

**After Training:**
- Person is automatically registered in SQLite database (`data/persons.db`)
- Gesture model path is linked to person entity
- Associated with existing face recognition models
- Unified person entity for all recognition systems

**Benefits:**
- Centralized person management
- Consistent person identification across systems
- Extensible for future features (Google account, preferences)
- Forward-compatible database schema

### Person Management

Access the Person Registry via:

**Option 1: Training Manager**
```bash
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
python3 train_manager.py
# Choose option [14] Manage persons
```

**Option 2: Direct CLI**
```bash
cd ~/dev/severin/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
python3 person_manager.py
```

**Features:**
- List all persons with their trained models
- View person details (face model, gesture model, metadata)
- Create/delete persons
- Migrate existing models

### For More Information

See [250_PERSON_MANAGEMENT_SYSTEM_REFERENCE.md](250_PERSON_MANAGEMENT_SYSTEM_REFERENCE.md) for complete Person Management System documentation, including:
- Database schema and design
- API reference
- Integration guide
- Future roadmap (Google account integration, cloud sync)

---

## Production Deployment

After training gestures, deploy to the auto-start system:

### Step 1: Update Camera Service Model Path

```bash
# Edit camera service
sudo nano /etc/systemd/system/r2d2-camera-perception.service

# Verify gesture_recognition_model_path points to your trained model
# Default: /home/severin/dev/r2d2/data/gesture_recognition/models/severin_gesture_classifier.pkl
```

### Step 2: Restart Services

```bash
sudo systemctl daemon-reload
sudo systemctl restart r2d2-camera-perception.service
sudo systemctl restart r2d2-gesture-intent.service
```

### Step 3: Verify Auto-Start

```bash
# Check both services
sudo systemctl status r2d2-camera-perception.service
sudo systemctl status r2d2-gesture-intent.service

# Watch logs
sudo journalctl -u r2d2-gesture-intent.service -f
```

### Step 4: Test After Reboot

```bash
sudo reboot
# Wait 60 seconds
# System should auto-start gesture recognition
# Test gestures in front of camera
```

**Model Storage:**
- Location: `data/gesture_recognition/models/{person_name}_gesture_classifier.pkl`
- Registered in: Person Registry database (`data/persons.db`)
- Used by: ROS 2 perception pipeline (`image_listener` node)
- Controlled by: Gesture intent node (auto-starts on boot)

For complete deployment details, see [300_GESTURE_SYSTEM_OVERVIEW.md](300_GESTURE_SYSTEM_OVERVIEW.md).

---

**Document Version:** 1.1  
**Last Updated:** December 17, 2025  
**Status:** Production Ready

