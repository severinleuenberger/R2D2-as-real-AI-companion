# Interactive Face Training System - User Guide

## Quick Start

### Setup
```bash
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8
cd ~/dev/r2d2/tests/face_recognition
```

### Run Interactive Training
```bash
python3 interactive_training.py
```

## What is This?

This is a **step-by-step interactive training system** that guides you through capturing face images under different conditions:

- **8 structured training tasks** (not random!)
- Each task specifies lighting, distance, head angle, and expressions
- You see instructions, then confirm when ready
- System captures images automatically
- Moves to next task when done

## The 8 Training Tasks

### Task 1: Bright Light - Front View
- ☀️ **Lighting**: Bright sunlight or strong lamp
- 📏 **Distance**: 1 meter from camera
- 🧑 **Head**: Face forward, rotate left-right, up-down
- ⏱️ **Duration**: 20 seconds
- **Goal**: Capture your face in ideal lighting conditions

### Task 2: Medium Light - 1.5m Distance
- 💡 **Lighting**: Indoor room light (no direct sun)
- 📏 **Distance**: 1.5 meters from camera
- 🧑 **Head**: Face forward, 45° angles left/right
- ⏱️ **Duration**: 20 seconds
- **Goal**: Capture at normal indoor lighting

### Task 3: Low Light - Front View
- 🌙 **Lighting**: Shadowed area (no direct light)
- 📏 **Distance**: 1.5 meters from camera
- 🧑 **Head**: Small movements, face forward
- ⏱️ **Duration**: 20 seconds
- **Goal**: Train the model to recognize you in dim conditions

### Task 4: Bright Light - Profile (45°)
- ☀️ **Lighting**: Bright sunlight or strong lamp
- 📏 **Distance**: 1.5 meters from camera
- 🧑 **Head**: Turned 45° to side (profile view)
- ⏱️ **Duration**: 20 seconds
- **Goal**: Capture side profile angles

### Task 5: Medium Light - 2 Meters Distance
- 💡 **Lighting**: Indoor room light
- 📏 **Distance**: 2 meters from camera (farther away)
- 🧑 **Head**: Face forward with slight turns
- ⏱️ **Duration**: 20 seconds
- **Goal**: Train at greater distance

### Task 6: Bright Light - Looking Up/Down
- ☀️ **Lighting**: Bright sunlight or strong lamp
- 📏 **Distance**: 1.5 meters from camera
- 🧑 **Head**: Look UP and DOWN (30° angles)
- ⏱️ **Duration**: 20 seconds
- **Goal**: Capture vertical head movements

### Task 7: Low Light - Profile Views
- 🌙 **Lighting**: Shadowed area (no direct light)
- 📏 **Distance**: 1.5 meters from camera
- 🧑 **Head**: Left profile → center → right profile
- ⏱️ **Duration**: 20 seconds
- **Goal**: Profile views in dim lighting

### Task 8: Medium Light - 3 Meters Distance (Final)
- 💡 **Lighting**: Indoor room light
- 📏 **Distance**: 3 meters from camera (farthest)
- 🧑 **Head**: Small movements, face centered
- ⏱️ **Duration**: 20 seconds
- **Goal**: Train at maximum practical distance

## How It Works

### Step-by-Step Workflow

```
1. START SCRIPT
   ↓
2. TASK 1 PRESENTED
   • See detailed instructions
   • Understand what to do
   ↓
3. WAIT FOR YOUR CONFIRMATION
   "Are you ready to START this task? Type 'yes' to begin:"
   ↓
4. AUTOMATIC CAPTURE
   • System captures for 20 seconds
   • Progress shown with dots
   • Faces auto-detected and saved
   ↓
5. RESULTS DISPLAYED
   "✓ Frames: 300"
   "✓ Faces detected: 287"
   "✓ Images saved: 45"
   "✓ Total accumulated: 45"
   ↓
6. CONTINUE?
   "Continue to next task? (yes/no):"
   ↓
7. REPEAT FOR ALL 8 TASKS
   (or skip/stop as needed)
   ↓
8. FINAL SUMMARY
   Shows total images captured
   Provides recommendations
```

## Important Tips

### For Best Results

✅ **DO:**
- Prepare the area before each task
- Read instructions carefully
- Position yourself correctly
- Move slowly and naturally
- Try to fill the frame with your face
- Look directly at camera when specified

❌ **DON'T:**
- Rush through tasks
- Wear sunglasses or hat
- Have another person in frame
- Move out of frame during capture
- Use extremely bright backlighting

### Lighting Setup

**For BRIGHT tasks:**
- Position yourself near a window (sunlight)
- Use a strong lamp or desk light
- Avoid shadows on your face

**For MEDIUM tasks:**
- Use normal room lighting
- Avoid harsh shadows
- Indirect light is fine

**For LOW LIGHT tasks:**
- Move away from windows/lamps
- Go to a shadowed corner
- Still enough to see the camera

### Distance Tips

- Use a tape measure or mark on the ground
- Measure from your face to camera lens
- Stay at the specified distance throughout the task
- Small movements forward/back are okay

## What Happens to Your Images?

### Local Storage (Safe)
```
~/dev/r2d2/data/face_recognition/severin/
├── 20251206_143000_bright_light_front_view_000.jpg
├── 20251206_143001_bright_light_front_view_001.jpg
├── ... (more images)
└── 20251206_143540_medium_light_3_meters_distance_045.jpg
```

### Files are 100×100 Grayscale JPEG
- Optimized for LBPH face recognizer
- Preprocessed automatically
- Timestamped for tracking

### GitHub Protection
✅ `.gitignore` prevents upload
- Only the models go to GitHub
- Training images stay on your Jetson
- No privacy concerns
- Can safely commit changes

## Target Image Counts

| Total Images | Status | Recommendation |
|--------------|--------|-----------------|
| < 50 | Poor | Run training again |
| 50-75 | Fair | Can work, add more if possible |
| 75-100 | Good | Ready for training |
| 100-150 | Excellent | Very robust model |
| 150+ | Professional | Maximum confidence |

## Example: Full Workflow

```bash
# 1. Start training
$ python3 interactive_training.py

# 2. See task details
📋 INSTRUCTIONS:
  ☀️ BRIGHT LIGHTING (sunlight or strong lamp)
  
  📏 DISTANCE: 1 meter from camera
  🧑 POSITION: Face forward, center of frame
  
  HEAD MOVEMENTS:
    • Start facing directly at camera
    • Slow head turn: LEFT → CENTER → RIGHT
    • Look slightly UP then DOWN
    • Move head side-to-side 2-3 times
  
  ⏱️ Duration: 20 seconds total

===============================================

# 3. You confirm when ready
Are you ready to START this task? Type "yes" to begin: yes

# 4. System captures automatically
Capturing... (20 seconds)
Progress: ..................

# 5. You see results
📊 TASK 1 RESULTS:
  ✓ Frames processed: 300
  ✓ Faces detected: 287
  ✓ Images saved: 45
  ✓ Total accumulated: 45

# 6. Continue to next task
Continue to next task? (yes/no): yes

# (Repeat for all 8 tasks)

# 7. Final summary
TRAINING SUMMARY
================

Tasks completed: 8 of 8
Total images captured: 127

RECOMMENDATIONS:
✓✓ You have 127 images!
   Excellent dataset for face recognition training.
   Ready to proceed with model training.

NEXT STEPS:
1. Train your model:
   python3 _train_module.py severin ~/dev/r2d2/data/face_recognition

2. Test recognition:
   python3 _test_module.py severin ~/dev/r2d2/data/face_recognition
```

## Troubleshooting

### "No faces detected in task"

**Cause**: Face not visible or too small
**Solutions**:
- Move closer to camera
- Ensure good lighting
- Clean camera lens
- Make sure face is centered

### "Only got 5-10 images per task"

**Cause**: Face wasn't detected most of the time
**Solutions**:
- Lighting too dim or too bright
- Face too far from camera
- Head moving too quickly
- Face at wrong angle

**If this happens:**
- Skip that task (type "no" when asked to continue)
- Redo it with better positioning
- Run training again to add more images

### "Camera not working"

**Cause**: OAK-D Lite not connected or driver issue
**Solutions**:
```bash
# Check if camera is detected
lsusb | grep OAK

# Check DepthAI connection
python3 -c "import depthai; print(depthai.__version__)"

# Reconnect camera and try again
```

## After Training

### Train the Model
Once you have 75+ images, train the LBPH recognizer:

```bash
python3 _train_module.py severin ~/dev/r2d2/data/face_recognition
```

This creates: `severin_lbph.xml` (50-100 KB)

### Test Recognition
Test the trained model at various distances:

```bash
python3 _test_module.py severin ~/dev/r2d2/data/face_recognition
```

Shows accuracy at 1m, 2m, 3m, 5m distances.

### Deploy in ROS 2
Use the model with your robot:

```bash
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true \
  face_recognition_model_path:=~/dev/r2d2/data/face_recognition/models/severin_lbph.xml
```

## Total Training Time

- **8 tasks** × **20 seconds** = **160 seconds** total
- Plus time between tasks (instructions, confirmation)
- **Total: ~10-15 minutes** for complete dataset

## Questions?

See these files for more info:
- `MODULAR_TRAINING.md` - Technical details
- `QUICK_START.md` - Quick reference
- `README.md` - Original guide

---

**Ready?** Run `python3 interactive_training.py` and let's capture your training data! 🚀
