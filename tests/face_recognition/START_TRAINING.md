# Starting Fresh Face Recognition Training

**Date:** December 12, 2025  
**Status:** Ready to Begin

---

## Setup Complete ✅

1. ✅ Old images backed up to: `data/face_recognition/severin_backup_20251212/` (847 images)
2. ✅ New directory created: `data/face_recognition/severin/` (empty, ready)
3. ✅ Improved capture script installed: `1_capture_training_data.py`
4. ✅ Launcher script created: `run_improved_capture.sh`

---

## How to Run Training Capture

### Option 1: Using Launcher Script (Recommended)

```bash
cd ~/dev/r2d2/tests/face_recognition
./run_improved_capture.sh
```

### Option 2: Manual

```bash
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8
python3 1_capture_training_data.py
```

---

## What to Expect

The script will guide you through **5 stages** (~50 seconds total):

1. **Stage 1: Bright Direct Light** (10 seconds, max 10 images)
   - Stand 1 meter from camera, look straight ahead
   - Move slowly left/right and nod up/down

2. **Stage 2: Dim Indoor Light** (10 seconds, max 10 images)
   - Move to dimly lit area
   - Stand 1 meter from camera, look straight ahead
   - Move left/right and nod

3. **Stage 3: Side Profile 45°** (10 seconds, max 10 images)
   - Return to bright area
   - Stand at 45-degree angle
   - Move slowly while maintaining angle

4. **Stage 4: Varied Distance** (15 seconds, max 10 images)
   - Stand 1m for 3s, then 2m for 3s, then 3m
   - Look straight ahead at each distance

5. **Stage 5: Facial Expressions** (5 seconds, max 10 images)
   - Stand 1 meter, front-facing
   - Show neutral, then smile, then variations

**Expected Result:** 40-50 high-quality images (not 800+!)

---

## After Capture

Once capture is complete:

1. **Verify images captured:**
   ```bash
   ls -lh ~/dev/r2d2/data/face_recognition/severin/
   # Should show ~40-50 images
   ```

2. **Train the model:**
   ```bash
   cd ~/dev/r2d2/tests/face_recognition
   source ~/depthai_env/bin/activate
   export OPENBLAS_CORETYPE=ARMV8
   python3 2_train_recognizer.py
   ```

3. **Test the model:**
   ```bash
   python3 3_test_recognizer_demo.py
   ```

---

## Quality Improvements

The improved script includes:
- ✅ **Quality filtering:** Only saves sharp, well-lit images
- ✅ **Deduplication:** Avoids redundant/similar images
- ✅ **Image limits:** 10-15 per stage, 40-50 total
- ✅ **Minimum interval:** 0.5s between saves
- ✅ **Real-time feedback:** Shows quality scores during capture

---

**Ready to start!** Run the launcher script when you're in front of the camera.

