# Step-by-Step Training Capture Guide

**Follow these steps when you're ready to capture training data.**

---

## Step 1: Prepare Yourself

✅ Position yourself in front of the camera (about 1 meter away)
✅ Make sure you have good lighting available
✅ Make sure you have a dimly lit area nearby
✅ Clear your schedule (~2 minutes total)

---

## Step 2: Run the Interactive Guide

When you're ready, run this command:

```bash
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8
python3 interactive_capture_guide.py
```

---

## Step 3: Follow the Prompts

The script will guide you through:

### **Stage 1: Bright Direct Light** (10 seconds)
- **When prompted:** Confirm you're ready
- **What to do:** Stand 1 meter from camera, look straight ahead
- **Movement:** Move slowly left/right, nod up/down
- **Result:** Up to 10 high-quality images saved

### **Stage 2: Dim Indoor Light** (10 seconds)
- **When prompted:** Confirm you're ready
- **What to do:** Move to dimly lit area, stand 1 meter from camera
- **Movement:** Move left/right, nod
- **Result:** Up to 10 high-quality images saved

### **Stage 3: Side Profile 45°** (10 seconds)
- **When prompted:** Confirm you're ready
- **What to do:** Return to bright area, stand at 45-degree angle
- **Movement:** Move slowly while maintaining 45° angle
- **Result:** Up to 10 high-quality images saved

### **Stage 4: Varied Distance** (15 seconds)
- **When prompted:** Confirm you're ready
- **What to do:** 
  - Stand 1m for 3 seconds
  - Move to 2m for 3 seconds
  - Move to 3m for remaining time
- **Movement:** Look straight ahead, move side-to-side at each distance
- **Result:** Up to 10 high-quality images saved

### **Stage 5: Facial Expressions** (5 seconds)
- **When prompted:** Confirm you're ready
- **What to do:** Stand 1 meter, front-facing
- **Movement:** Show neutral expression, then smile, then variations
- **Result:** Up to 10 high-quality images saved

---

## Step 4: After Capture

Once all stages complete:

1. **Verify images were captured:**
   ```bash
   ls -lh ~/dev/r2d2/data/face_recognition/severin/
   # Should show ~40-50 images
   ```

2. **Check the count:**
   ```bash
   ls -1 ~/dev/r2d2/data/face_recognition/severin/*.jpg | wc -l
   # Should be around 40-50
   ```

---

## Step 5: Train the Model

Once you have the images:

```bash
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8
python3 2_train_recognizer.py
```

---

## Step 6: Test the Model

Test the trained model:

```bash
python3 3_test_recognizer_demo.py
```

---

## What Makes This Different?

✅ **Quality filtering:** Only saves sharp, well-lit images
✅ **Deduplication:** Avoids redundant/similar images
✅ **Smart limits:** 10-15 images per stage (not 300+)
✅ **Real-time feedback:** Shows quality scores during capture
✅ **Total images:** 40-50 (not 800+)

---

**Ready when you are!** Run the command in Step 2 when you're positioned in front of the camera.

