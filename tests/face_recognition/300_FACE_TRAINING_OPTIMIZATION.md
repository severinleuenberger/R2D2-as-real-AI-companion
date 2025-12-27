# Face Training Optimization Guide

## Overview

This document describes the automatic dataset optimization feature integrated into the R2D2 Face Recognition Training Manager. The feature automatically detects oversized datasets and offers to select the best high-quality, diverse images for optimal training results.

## Problem Statement

The `_capture_module.py` script captures training images at 30 FPS for approximately 40 seconds across 4 stages. Without filtering, this can result in:

- **300-1000+ images** captured per person
- **Many duplicate/similar images** from consecutive frames
- **Low-quality images** (blurry, poor lighting)
- **Slow training times** due to excessive data
- **Potential overfitting** from redundant training data

**Real Example:** A typical capture session produced 942 images, when optimal training requires only 50-100 high-quality, diverse images.

## Solution

Two fixes were implemented:

### Fix 1: Capture Script Bug Fix

**Problem:** `train_manager.py` was calling a non-existent file `1_capture_training_data.py`

**Solution:** Updated to call the correct modular capture script `_capture_module.py`

### Fix 2: Automatic Dataset Optimization

When a dataset exceeds 100 images, the system automatically:
1. Detects the oversized dataset
2. Prompts the user with optimization options
3. If selected, analyzes all images for quality
4. Selects the best 75 diverse, high-quality images
5. Archives extras to a timestamped backup folder

## How It Works

### Quality Assessment Algorithm

Each image is scored based on three factors:

```
quality_score = blur_score * 0.5 + lighting_score * 0.3 + contrast_score * 0.2
```

| Factor | Weight | How It's Measured |
|--------|--------|-------------------|
| Blur | 50% | Laplacian variance (higher = sharper) |
| Lighting | 30% | Mean brightness deviation from 128 (closer = better) |
| Contrast | 20% | Standard deviation / 128 (higher = more contrast) |

### Diversity Selection

After quality filtering, images are selected for diversity:

1. **Sort by quality** (highest first)
2. **Compare each candidate** to already-selected images using histogram correlation
3. **Accept if diverse** (similarity < 0.7 to existing selections)
4. **Continue until target reached** (default: 75 images)

### Archive Strategy

Extras are moved (not deleted) to a timestamped folder:
```
~/dev/r2d2/data/face_recognition/severin_archive_20251227_131115/
```

This allows recovery if needed while keeping the active dataset optimal.

## Usage

### Option 1: Train New Person (Menu [1])

After capturing images, if more than 100 are captured:

```
⚠️  Captured 312 images
   Optimal training uses 50-100 high-quality, diverse images.

Would you like to optimize the dataset? (recommended)
Optimize now? (y/n): y

======================================================================
OPTIMIZING DATASET
======================================================================
  📊 Analyzing 312 images...
  ✓ 287 images pass quality threshold (>= 0.4)
  🔍 Selecting 75 diverse images...
  📦 Archiving 237 images...
  ✅ Kept 75 best images
  ✅ Archived 237 images to severin_archive_20251227_140523

STEP 2: Train Model
Found 75 training images.
```

### Option 2: Train from Existing Images (Menu [3])

When selecting a person with >100 images:

```
Select person to train:
  [1] Severin (942 images)
  [0] Back

Enter choice: 1

⚠️  Large dataset detected: 942 images
   Optimal training uses 50-100 high-quality, diverse images.
   Too many images can slow training and may include duplicates.

Options:
  [1] Optimize dataset first (recommended) - Select best ~75 images
  [2] Train with all images as-is
  [0] Cancel

Enter choice: 1
```

### Standalone Tool

A standalone optimization tool is also available:

```bash
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8

# Dry run (preview what will happen)
python3 reduce_training_images.py severin --dry-run

# Actual optimization
python3 reduce_training_images.py severin
```

## Technical Reference

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `target_images` | 75 | Number of images to keep |
| `min_quality` | 0.4 | Minimum quality score threshold |
| Diversity threshold | 0.3 | Minimum diversity score to accept |

### Files Modified

- `train_manager.py`: Added optimization methods and integration
- `reduce_training_images.py`: Standalone optimization tool (new)

### Methods Added to TrainingManager

```python
def assess_image_quality(self, image_path):
    """Assess blur, lighting, and contrast of a face image."""
    
def calculate_diversity_score(self, img, existing_images):
    """Calculate how different an image is from existing selections."""
    
def optimize_training_dataset(self, person_name, target_images=75, min_quality=0.4):
    """Select best images and archive the rest."""
```

## Results

**Before Optimization:**
- 942 images
- Training time: ~45 seconds
- Model potentially overfitted to similar poses

**After Optimization:**
- 75 images (92% reduction)
- Training time: ~5 seconds
- Model trained on diverse, high-quality data

## Troubleshooting

### "Dataset already optimal"

If you see this message, your dataset is already at or below the target (75 images). No optimization is needed.

### Recovery from Archive

If you need to restore archived images:

```bash
# Find archive folder
ls ~/dev/r2d2/data/face_recognition/

# Move images back (example)
mv ~/dev/r2d2/data/face_recognition/severin_archive_20251227_131115/*.jpg \
   ~/dev/r2d2/data/face_recognition/severin/
```

### Changing Target Image Count

To optimize to a different number of images, use the standalone tool or modify the `target_images` parameter in `optimize_training_dataset()`.

## Best Practices

1. **Let the system optimize** - The automatic optimization is well-tuned for LBPH face recognition
2. **Don't delete archives immediately** - Keep them until you're satisfied with the trained model
3. **Re-train after optimization** - The model needs to be retrained with the optimized dataset
4. **Verify model accuracy** - Use the real-time recognition test (Menu [5]) to verify results

## See Also

- `_capture_module.py` - Image capture with 4-stage guidance
- `_train_module.py` - LBPH model training
- `_test_module.py` - Model accuracy testing
- `reduce_training_images.py` - Standalone optimization tool

---

**Last Updated:** December 27, 2025
**Feature:** Automatic dataset optimization for face recognition training

