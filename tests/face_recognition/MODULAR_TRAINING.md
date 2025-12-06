# Modular Face Training System

## Overview

The face recognition training system has been refactored from a rigid **sequential** approach to a flexible **modular** architecture.

### Old Approach (Sequential)
```bash
python3 1_capture_training_data.py      # Fixed 4 stages, Severin only
python3 2_train_recognizer.py           # Trains fixed model
python3 3_test_recognizer_demo.py       # Tests fixed distances
```

**Issues:**
- Only works for Severin
- All stages must be completed in order
- Hard to modify or extend
- No reusability

### New Approach (Modular & Interactive)
```bash
python3 train_manager.py                # Central hub with menu
# Menu options:
# [1] Train new person
# [2] Capture images
# [3] Train model
# [4] Test model
# [5] List people
# [6] Delete person
```

**Features:**
- ✅ **Multi-person**: Train any person, not just Severin
- ✅ **Flexible workflow**: Capture, train, test in any order
- ✅ **Interactive prompts**: ENTER key between stages
- ✅ **Clear instructions**: Bold, explicit user guidance
- ✅ **Modular design**: Reusable components (_capture_module, _train_module, _test_module)
- ✅ **Status tracking**: View progress anytime

## Architecture

### Components

```
train_manager.py (Main Hub)
├── _capture_module.py   (Captures training images)
├── _train_module.py     (Trains LBPH model)
└── _test_module.py      (Tests recognition at distances)
```

### Directory Structure

```
~/dev/r2d2/data/face_recognition/
├── models/                      (Trained models)
│   ├── severin_lbph.xml        (LBPH model for Severin)
│   ├── john_doe_lbph.xml       (LBPH model for John)
│   └── ...
├── severin/                     (Training images for Severin)
│   ├── 20251206_143000_bright_direct_000.jpg
│   ├── 20251206_143001_bright_direct_001.jpg
│   └── ... (92 images)
├── john_doe/                    (Training images for John)
│   └── ... (to be captured)
└── ...
```

## Usage

### Activate Virtual Environment

```bash
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8
```

### Start Training Manager

```bash
cd ~/dev/r2d2/tests/face_recognition
python3 train_manager.py
```

### Complete Workflow: Train New Person

1. **Start Manager**
   ```
   python3 train_manager.py
   ```

2. **Select [1] Train new person**

3. **Enter person name**
   - Examples: `john`, `alice_smith`, `bob`
   - Use lowercase, no spaces
   
4. **Step 1: Capture Training Images**
   - Follow interactive prompts
   - 4 stages (bright, dim, 45°, varied distance)
   - 10 seconds each stage
   - Automatic face detection
   - Target: 75-100 images
   
   **Instructions appear:**
   ```
   ======================================================================
   STAGE: BRIGHT DIRECT LIGHT
   ======================================================================
   
   Position yourself in bright, direct light (e.g., sunny window).
   Stand 1 meter from camera, facing forward.
   Move slowly: left/right, up/down to vary angles.
   Try different expressions: neutral, smiling, surprised.
   
   ======================================================================
   Press ENTER when ready to capture...
   ```

5. **Step 2: Train Model**
   - Runs LBPH (Local Binary Pattern Histograms)
   - Takes ~3-5 seconds
   - Saves model to `models/{person}_lbph.xml`
   
6. **Step 3: Test Model**
   - Tests at 4 distances: 1m, 2m, 3m, 5m
   - Reports accuracy and confidence
   - Takes ~10 seconds per distance

### Quick Operations

#### Capture Only
```
python3 train_manager.py
→ [2] Capture images for person
→ Select person
→ Follow prompts
```

#### Train Only
```
python3 train_manager.py
→ [3] Train model from existing images
→ Select person with images
```

#### Test Only
```
python3 train_manager.py
→ [4] Test trained model
→ Select person with trained model
```

#### List Status
```
python3 train_manager.py
→ [5] List all people and models
→ View training progress for each person
```

#### Delete Person
```
python3 train_manager.py
→ [6] Delete person (images + model)
→ Confirm deletion
```

## User Interaction Model

### ENTER Prompts

Each stage waits for you to press ENTER before starting:

```
======================================================================
STAGE: BRIGHT DIRECT LIGHT
======================================================================

[Instructions displayed]

======================================================================
Press ENTER when ready to capture...
_
```

This gives you time to:
- Position yourself correctly
- Prepare your expression
- Ensure good lighting
- Mentally prepare

### Clear Feedback

Progress is shown in real-time:

```
Capturing... (10 seconds)

Stage Results:
  ✓ Frames captured: 300
  ✓ Faces detected: 287
  ✓ Images saved: 45
  ✓ Total so far: 120
```

## Training Data

### Recommended Amounts

| Images | Status | Recommendation |
|--------|--------|-----------------|
| < 50 | Poor | Add more training data |
| 50-75 | Fair | Ready to train, consider more |
| 75-100 | Good | Excellent for training |
| > 100 | Excellent | Very robust model |

### Training Stages

Each stage captures images under different conditions:

1. **Bright Direct Light**
   - Purpose: Well-lit conditions
   - Position: 1 meter, facing forward
   - Movement: Side-to-side, up-down, expressions

2. **Dim Indoor Light**
   - Purpose: Normal indoor lighting
   - Position: 1 meter, facing forward
   - Movement: Side-to-side, up-down

3. **Side Profile (45°)**
   - Purpose: Angle recognition
   - Position: 45° angle to camera
   - Movement: Forward-back, left-right profiles

4. **Varied Distance**
   - Purpose: Distance robustness
   - Stages: 1m → 2m → 3m
   - Movement: Head movement at each distance

## Technical Details

### LBPH Algorithm

- **Name**: Local Binary Pattern Histograms
- **Type**: Face recognition (not detection)
- **Speed**: ~5-10ms per face on Jetson
- **CPU**: Minimal (runs on single core)
- **Memory**: ~50-100 KB per model
- **Accuracy**: Depends heavily on training data quality

### Parameters

```python
# In launch files and ROS 2:
enable_face_recognition: true
face_recognition_model_path: ~/dev/r2d2/data/face_recognition/models/severin_lbph.xml
recognition_confidence_threshold: 70        # 0-255 scale, lower = stricter
recognition_frame_skip: 2                    # Process every 2nd frame
```

## Troubleshooting

### Problem: "No faces detected during capture"

**Causes:**
- Poor lighting (too dark or backlighting)
- Face too small or too close
- Camera obstruction

**Solutions:**
- Ensure good lighting (bright window or lamp)
- Stand 1 meter from camera
- Clean camera lens
- Try again with different positioning

### Problem: "Low recognition accuracy"

**Causes:**
- Too few training images
- Poor lighting during training
- Limited angle coverage
- Large differences between capture and testing conditions

**Solutions:**
- Capture additional images (add 50+)
- Use consistent lighting similar to deployment
- Include more profile and angle variations
- Retrain with expanded dataset

### Problem: "Module not found errors"

**Causes:**
- Virtual environment not activated
- Scripts in wrong directory
- Python path issues

**Solutions:**
```bash
source ~/depthai_env/bin/activate
cd ~/dev/r2d2/tests/face_recognition
python3 train_manager.py
```

### Problem: "Import errors for cv2.face"

**Causes:**
- OpenCV contrib not installed

**Solution:**
```bash
source ~/depthai_env/bin/activate
pip install opencv-contrib-python
```

## Integration with ROS 2

Once you have a trained model, integrate it with the robot:

```bash
# Update launch file with model path
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true \
  face_recognition_model_path:=~/dev/r2d2/data/face_recognition/models/severin_lbph.xml

# Monitor recognition output
ros2 topic echo /r2d2/perception/person_id
ros2 topic echo /r2d2/perception/face_confidence
```

## Files Created/Modified

### New Files
- `train_manager.py` - Main hub for all training operations
- `_capture_module.py` - Interactive image capture with prompts
- `_train_module.py` - LBPH training pipeline
- `_test_module.py` - Recognition testing at multiple distances
- `MODULAR_TRAINING.md` - This documentation

### Old Files (Still Available)
- `1_capture_training_data.py` - Original sequential capture
- `2_train_recognizer.py` - Original training script
- `3_test_recognizer_demo.py` - Original testing script
- `README.md` - Original quick-start guide

## Development Notes

### Modular Design Benefits

1. **Testability**: Each module can be tested independently
2. **Reusability**: Modules can be called from other scripts
3. **Maintainability**: Changes isolated to specific modules
4. **Extensibility**: Easy to add new features or algorithms
5. **Debugging**: Easier to isolate and fix issues

### Adding New Algorithms

To add a new face recognition algorithm (e.g., FisherFaces, EigenFaces):

1. Create `_algorithm_module.py` with new training/recognition logic
2. Update `train_manager.py` to offer algorithm selection
3. Maintain consistent interface (person_name, data_dir parameters)

### Multi-Person Recognition

Current implementation:
- Single person per model (one LBPH per person)
- Simple linear search through models
- Confidence score per person

Future improvements:
- Multi-person single model (person ID encoding)
- Parallel model testing
- Weighted ensemble results

---

**Last Updated**: December 6, 2025  
**Version**: 2.0 (Modular)  
**Author**: R2D2 Perception Pipeline
