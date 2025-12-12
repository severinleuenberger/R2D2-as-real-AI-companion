# Face Recognition Training System - Improvement Plan

**Date:** December 9, 2025  
**Status:** Analysis & Recommendations  
**Issue:** Current system captures 847 images (way too many!) - optimal is 20-50 high-quality images

---

## Executive Summary

The current face recognition training system captures **847 images**, which is **17-42x more than optimal** (20-50 images). The capture script saves every detected face in every frame, leading to massive redundancy and potential overfitting. This document provides analysis and recommendations for improvement.

---

## Current System Analysis

### Current Capture Process

**Script:** `1_capture_training_data.py`

**How it works:**
- Captures 4 stages (bright_direct, dim_indoor, side_45deg, varied_distance)
- Each stage runs for 10-15 seconds at 30 FPS
- **Saves EVERY face detected in EVERY frame**
- No quality filtering
- No deduplication
- No selection mechanism

**Current Results:**
- **Total images:** 847
- **bright_dist:** 374 images
- **bright_direct:** 5 images
- **unknown:** 468 images (from old sessions)
- **Storage:** 4.3 MB

**Problems:**
1. **Massive redundancy:** 300-450 images per stage (30 FPS × 10-15 seconds)
2. **No quality control:** Saves blurry, poorly lit, or occluded faces
3. **No deduplication:** Many nearly identical images
4. **Overfitting risk:** Too many similar images can hurt generalization
5. **Training time:** Unnecessarily long (processing 847 images vs 50)
6. **Storage waste:** 4.3 MB of mostly redundant data

---

## Best Practices Research

### Optimal Number of Training Images for LBPH

**Research Findings:**
- **Minimum:** 10-15 high-quality images (basic recognition)
- **Optimal:** 20-50 high-quality, diverse images
- **Maximum:** 100 images (diminishing returns, risk of overfitting)
- **Current:** 847 images ❌ (17-42x too many!)

**Key Principles:**
1. **Quality over quantity:** 20 excellent images > 200 mediocre images
2. **Diversity is critical:** Different lighting, angles, expressions, distances
3. **Avoid redundancy:** Similar images don't add value
4. **Face size:** At least 100×100 pixels (current: 100×100 ✓)
5. **Resolution:** Higher is better, but 100×100 is acceptable for LBPH

### What Makes a Good Training Image?

✅ **Good:**
- Clear, sharp face (no blur)
- Good lighting (even illumination, no harsh shadows)
- Front-facing or slight angle (≤45°)
- Face occupies significant portion of frame
- No obstructions (glasses OK, but not hats/masks)
- Neutral or varied expressions
- Different distances (1m, 2m, 3m)

❌ **Bad:**
- Blurry or out-of-focus
- Poor lighting (too dark, too bright, harsh shadows)
- Extreme angles (>45°)
- Face too small or too large
- Obstructed (hands, objects, partial occlusion)
- Identical to previous images (redundant)

---

## Recommended Improvements

### 1. Smart Capture Script with Quality Filtering

**New Approach:**
- Capture images only when quality criteria are met
- Limit to 10-15 images per stage (total: 40-60 images)
- Implement quality scoring:
  - Face size check (not too small/large)
  - Blur detection (Laplacian variance)
  - Lighting assessment (histogram analysis)
  - Angle estimation (face alignment)
  - Deduplication (compare with recent images)

**Implementation Strategy:**
```python
# Pseudo-code for improved capture
def capture_stage(stage_name, duration_seconds=10, max_images=15):
    saved_images = []
    last_saved_time = 0
    min_interval = 0.5  # Minimum 0.5s between saves
    
    while elapsed < duration_seconds:
        face = detect_face(frame)
        if face:
            quality_score = assess_quality(face)
            similarity = compare_with_recent(face, saved_images)
            
            if (quality_score > threshold and 
                similarity < threshold and
                time_since_last_save > min_interval and
                len(saved_images) < max_images):
                save_image(face)
                saved_images.append(face)
```

### 2. Image Quality Assessment

**Quality Metrics:**
1. **Blur Detection:** Laplacian variance (higher = sharper)
2. **Lighting:** Histogram analysis (even distribution = good)
3. **Face Size:** Check if face is 50-200 pixels (current: 100×100 ✓)
4. **Face Angle:** Estimate pose (prefer frontal, accept ≤45°)
5. **Brightness:** Mean pixel value (avoid too dark/bright)

**Implementation:**
```python
def assess_quality(face_image):
    # Blur detection
    laplacian_var = cv2.Laplacian(face_image, cv2.CV_64F).var()
    blur_score = min(laplacian_var / 100.0, 1.0)  # Normalize
    
    # Lighting assessment
    hist = cv2.calcHist([face_image], [0], None, [256], [0, 256])
    mean_brightness = np.mean(face_image)
    lighting_score = 1.0 - abs(mean_brightness - 128) / 128.0
    
    # Combined score
    quality_score = (blur_score * 0.6 + lighting_score * 0.4)
    return quality_score
```

### 3. Deduplication

**Strategy:**
- Compare new image with last 5-10 saved images
- Use structural similarity (SSIM) or histogram comparison
- Only save if similarity < 0.85 (different enough)

**Implementation:**
```python
def is_duplicate(new_image, recent_images, threshold=0.85):
    for recent in recent_images[-10:]:  # Check last 10
        similarity = compare_images(new_image, recent)
        if similarity > threshold:
            return True
    return False
```

### 4. Optimal Capture Strategy

**Recommended Stages (Total: 40-50 images):**

1. **Frontal, Bright Light (10 images)**
   - 1 meter distance
   - Front-facing, good lighting
   - 10 seconds, save max 10 images

2. **Frontal, Low Light (10 images)**
   - 1 meter distance
   - Dim lighting
   - 10 seconds, save max 10 images

3. **Angled Views (10 images)**
   - 45° left, 45° right
   - 1 meter distance
   - 10 seconds, save max 10 images

4. **Varied Distances (10 images)**
   - 1m, 2m, 3m distances
   - Front-facing
   - 15 seconds, save max 10 images

5. **Expressions (5-10 images)**
   - Neutral, smile, slight variations
   - 5 seconds, save max 10 images

**Total:** 40-50 high-quality, diverse images

### 5. Post-Capture Curation

**Manual Review Script:**
- Display all captured images in a grid
- Allow user to delete poor-quality images
- Show quality scores for guidance
- Final count: 20-50 images

**Implementation:**
```python
def review_and_curate(image_dir):
    images = load_images(image_dir)
    display_grid(images, quality_scores)
    selected = user_select_best(images, min_count=20, max_count=50)
    delete_unselected(images, selected)
```

---

## Implementation Plan

### Phase 1: Improve Capture Script (Priority: High)

**File:** `1_capture_training_data.py`

**Changes:**
1. Add quality assessment function
2. Add deduplication check
3. Limit images per stage (10-15 max)
4. Add minimum interval between saves (0.5s)
5. Display quality score during capture

**Expected Result:**
- Capture 40-60 images instead of 800+
- Higher quality images
- Less redundancy
- Faster training

### Phase 2: Add Post-Capture Curation (Priority: Medium)

**New Script:** `1b_curate_training_data.py`

**Features:**
- Display all captured images
- Show quality scores
- Allow manual selection
- Delete unselected images
- Final dataset: 20-50 images

### Phase 3: Improve Training Script (Priority: Low)

**File:** `2_train_recognizer.py`

**Enhancements:**
- Add data augmentation (rotation, brightness, contrast)
- Validate image quality before training
- Report training statistics
- Cross-validation (if multiple people)

---

## Migration Plan for Existing Data

### Current Situation
- 847 images in `~/dev/r2d2/data/face_recognition/severin/`
- Many redundant, low-quality images

### Recommended Action

**Option A: Start Fresh (Recommended)**
1. Backup current images: `mv severin severin_backup_20251209`
2. Run improved capture script
3. Capture 40-50 new high-quality images
4. Train new model
5. Compare performance with old model

**Option B: Curate Existing Images**
1. Run curation script on existing 847 images
2. Select best 20-50 images
3. Move selected to new directory
4. Retrain model

**Recommendation:** Option A (start fresh) - ensures quality and diversity

---

## Verification: Git Exclusion

✅ **Verified:** Training images are properly excluded from Git

**Check:**
```bash
# .gitignore contains:
data/face_recognition/*/
!data/face_recognition/models/

# Verification:
git check-ignore data/face_recognition/severin/*.jpg
# All images are ignored ✓
```

**Status:** ✅ Images are stored locally only, not in Git

---

## Expected Improvements

### Before (Current)
- **Images:** 847
- **Quality:** Mixed (many redundant, some poor quality)
- **Training time:** ~60 seconds
- **Storage:** 4.3 MB
- **Accuracy:** Potentially overfitted to specific frames

### After (Improved)
- **Images:** 20-50
- **Quality:** High (all meet quality criteria)
- **Training time:** ~5-10 seconds
- **Storage:** ~200-500 KB
- **Accuracy:** Better generalization, less overfitting

---

## Next Steps

1. **Review this plan** and approve approach
2. **Implement improved capture script** (Phase 1)
3. **Test with new capture** (capture 40-50 images)
4. **Compare performance** (old model vs new model)
5. **Implement curation script** (Phase 2, optional)
6. **Update documentation** in `100_PERSON_RECOGNITION_AND_STATUS.md`

---

## References

- OpenCV LBPH Documentation: https://docs.opencv.org/master/df/d25/classcv_1_1face_1_1LBPHFaceRecognizer.html
- Best Practices: Multiple sources (see web search results)
- Current Implementation: `~/dev/r2d2/tests/face_recognition/`

---

**Document Version:** 1.0  
**Last Updated:** December 9, 2025  
**Status:** Ready for Implementation

