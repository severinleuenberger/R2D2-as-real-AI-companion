# Quick Start: Modular Face Training System

## Setup (One-Time)

```bash
# Activate virtual environment
source ~/depthai_env/bin/activate

# Set ARM optimization
export OPENBLAS_CORETYPE=ARMV8

# Navigate to training directory
cd ~/dev/r2d2/tests/face_recognition
```

## Start Training Manager

```bash
python3 train_manager.py
```

## Menu Options

```
📦 TRAINED MODELS: (shows trained models)
📷 TRAINING DATASETS: (shows captured images)

MENU OPTIONS:
  [1] Train new person (capture + train + test)
  [2] Capture images for person
  [3] Train model from existing images
  [4] Test trained model
  [5] List all people and models
  [6] Delete person (images + model)
  [0] Exit
```

## Typical Workflow: Train Someone New

### Step 1: Start Manager
```bash
python3 train_manager.py
```

### Step 2: Select [1] Train new person

### Step 3: Enter person name
```
Enter person name: john_doe
```

### Step 4: Capture Training Images (Stage 1 of 3)
- Bright direct light
- Stand 1 meter from camera
- Move slowly: left/right, up-down
- **Press ENTER when ready**
- System captures automatically for 10 seconds
- Target: 30-40 images per stage

### Step 5: Repeat for other stages
- Dim indoor light
- Side profile (45°)
- Varied distance (1m → 2m → 3m)

### Step 6: Train Model (Stage 2 of 3)
- System runs LBPH training
- Takes ~3-5 seconds
- Saves model: `john_doe_lbph.xml`

### Step 7: Test Model (Stage 3 of 3)
- Tests at 4 distances: 1m, 2m, 3m, 5m
- Reports accuracy and confidence
- Takes ~10-15 seconds total

## Results

After completion:
- ✅ Training images saved: `~/dev/r2d2/data/face_recognition/john_doe/`
- ✅ Trained model saved: `~/dev/r2d2/data/face_recognition/models/john_doe_lbph.xml`
- ✅ Recognition ready for ROS 2 deployment

## Key Points

### ENTER Prompts
The system will ask you to press ENTER before each capture stage. This gives you time to:
- Position yourself correctly
- Adjust lighting
- Prepare your expression

### Progress Indicators
Real-time feedback shows:
- Frames being captured
- Faces detected
- Images saved
- Total captured so far

### Recommendations
- **Target images**: 75-100 total (across all 4 stages)
- **Less than 50**: Low quality model, capture more
- **50-75**: Fair, can improve with more
- **75-100**: Good model ready for deployment
- **100+**: Excellent robustness

## Troubleshooting

### "No faces detected"
- Check lighting (too dark?)
- Ensure you're 1 meter from camera
- Clean camera lens
- Try different position

### "Low recognition accuracy"
- Capture more images (add another stage)
- Improve lighting conditions
- Include more angles (45°, profile views)
- Retrain model with larger dataset

### Module not found errors
```bash
# Make sure you're in correct directory
cd ~/dev/r2d2/tests/face_recognition

# Verify virtual environment
source ~/depthai_env/bin/activate
which python3  # Should show depthai_env path
```

## File Structure

```
~/dev/r2d2/
├── tests/face_recognition/
│   ├── train_manager.py          ← START HERE
│   ├── _capture_module.py        (called by manager)
│   ├── _train_module.py          (called by manager)
│   ├── _test_module.py           (called by manager)
│   ├── MODULAR_TRAINING.md       (detailed docs)
│   ├── QUICK_START.md            (this file)
│   ├── 1_capture_training_data.py (legacy)
│   ├── 2_train_recognizer.py     (legacy)
│   └── 3_test_recognizer_demo.py (legacy)
│
└── data/face_recognition/
    ├── models/
    │   ├── severin_lbph.xml      (existing model)
    │   └── john_doe_lbph.xml     (new model)
    ├── severin/
    │   └── (92 training images)
    └── john_doe/
        └── (your captured images)
```

## Example Session

```bash
$ python3 train_manager.py

📦 TRAINED MODELS:
   ✓ severin (50.2 KB)

📷 TRAINING DATASETS:
   • severin (92 images)

MENU OPTIONS:
  [1] Train new person (capture + train + test)
  ...

Enter choice (0-6): 1

Complete workflow: capture → train → test

Enter person name: alice_smith

STEP 1: Capture Training Images
------ ... ------
Ready to capture? (y/n): y

[CAPTURE MODULE RUNS...]
... Bright Direct Light stage ...
... Dim Indoor Light stage ...
... Side Profile 45° stage ...
... Varied Distance stage ...

Captured total: 98 images ✓

STEP 2: Train Model
------ ... ------
Found 98 training images.
Ready to train? (y/n): y

[TRAINING MODULE RUNS...]
✓ Model trained successfully
✓ File size: 68.3 KB
✓ Model saved: alice_smith_lbph.xml

STEP 3: Test Model
------ ... ------
Ready to test? (y/n): y

[TESTING MODULE RUNS...]
Results at 1m: 85.2% accuracy [✓ GOOD]
Results at 2m: 72.5% accuracy [✓ GOOD]
Results at 3m: 45.3% accuracy [⚠️  FAIR]
Results at 5m: 12.1% accuracy [❌ POOR]

✅ TRAINING COMPLETE

Person: alice_smith
Images: 98
Model: alice_smith_lbph.xml

Your model is ready for deployment!
```

## Next Steps

1. **Deploy to ROS 2**
   ```bash
   ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
     enable_face_recognition:=true \
     face_recognition_model_path:=~/dev/r2d2/data/face_recognition/models/alice_smith_lbph.xml
   ```

2. **Monitor Recognition**
   ```bash
   ros2 topic echo /r2d2/perception/person_id
   ros2 topic echo /r2d2/perception/face_confidence
   ```

3. **Add More People**
   - Repeat the workflow with different names
   - Each person gets their own model file
   - All models can be used simultaneously

---

**Questions?** See `MODULAR_TRAINING.md` for detailed documentation.
