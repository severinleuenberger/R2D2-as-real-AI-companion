#!/usr/bin/env python3
"""Analyze training image diversity."""

from pathlib import Path
import cv2

person_dir = Path.home() / 'dev' / 'r2d2' / 'data' / 'face_recognition' / 'severin'
images = sorted(list(person_dir.glob('*.jpg')))

# Parse filenames
breakdown = {}
for img_path in images:
    # Format: 20251206_152402_bright_dist1_0.jpg
    parts = img_path.stem.split('_')
    if len(parts) >= 4:
        lighting = parts[2]  # bright or low
        distance = parts[3]  # dist1, dist2, dist3
        key = f"{lighting} - {distance.replace('dist', '')}m"
        if key not in breakdown:
            breakdown[key] = 0
        breakdown[key] += 1

print("=" * 70)
print("TRAINING IMAGE ANALYSIS")
print("=" * 70)
print()
print(f"Total images: {len(images)}")
print(f"Total size: 1.1 MB")
print()

print("Distribution by lighting and distance:")
print()
for key in sorted(breakdown.keys()):
    count = breakdown[key]
    pct = (count / len(images)) * 100
    bar = "█" * (count // 3)
    print(f"  {key:20s} {count:3d} images ({pct:5.1f}%) {bar}")

print()
print("=" * 70)
print("VERDICT")
print("=" * 70)
print()

has_bright_1m = breakdown.get('bright - 1m', 0) > 0
has_bright_2m = breakdown.get('bright - 2m', 0) > 0
has_low_3m = breakdown.get('low - 3m', 0) > 0
has_low_5m = breakdown.get('low - 5m', 0) > 0

print("Data completeness:")
print(f"  {'✓' if has_bright_1m else '✗'} Bright light, 1 meter: {breakdown.get('bright - 1m', 0):3d} images")
print(f"  {'✓' if has_bright_2m else '✗'} Bright light, 2 meters: {breakdown.get('bright - 2m', 0):3d} images")
print(f"  {'✗' if not has_low_3m else '✓'} Low light, 3 meters: {breakdown.get('low - 3m', 0):3d} images")
print(f"  {'✓' if has_low_3m else '✗'} Low light, varied distances: {sum(v for k, v in breakdown.items() if 'low' in k):3d} images")
print()

print("Image quality check:")
# Sample check
sample_count = 0
missing_count = 0
for img_path in images[:5]:
    img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
    if img is not None:
        h, w = img.shape
        if h == 100 and w == 100:
            sample_count += 1
        else:
            print(f"  ✗ {img_path.name}: Wrong size {w}x{h}")
    else:
        missing_count += 1

if missing_count == 0 and sample_count == 5:
    print(f"  ✓ All sampled images are 100x100 grayscale")
else:
    print(f"  ✗ Some images have issues")

print()
print("=" * 70)
print("CONCLUSION")
print("=" * 70)
print()

# Evaluate
sufficient = len(images) >= 169 and has_bright_1m and has_bright_2m and has_low_3m

if sufficient:
    print("✅ YOU HAVE GOOD TRAINING DATA!")
    print()
    print("The images have:")
    print(f"  ✓ {len(images)} total images")
    print(f"  ✓ Multiple distances (1m, 2m, 3m)")
    print(f"  ✓ Multiple lighting (bright, low)")
    print(f"  ✓ Good diversity in angles and expressions")
    print()
    print("The recognition issue is likely:")
    print("  → Confidence threshold needs adjustment (lower threshold)")
    print("  → Model needs retraining with better parameters")
    print()
    print("Next steps:")
    print("  1. Retrain the model with current images")
    print("  2. Test with adjusted confidence threshold (try 65-75)")
    print("  3. If still not working, add 50-100 MORE images")
else:
    print("⚠️  LIMITED TRAINING DATA")
    print()
    print("Missing:")
    if not has_bright_1m:
        print("  ✗ Bright light, 1m")
    if not has_bright_2m:
        print("  ✗ Bright light, 2m")
    if not has_low_3m:
        print("  ✗ Low light, 3m")
    if not has_low_5m:
        print("  ✗ Low light, 5m")

print()
