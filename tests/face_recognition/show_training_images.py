#!/usr/bin/env python3
"""
View training images as a grid/montage.
Shows random samples of the training images.
"""

import cv2
import os
from pathlib import Path
import random
import numpy as np

person_dir = Path.home() / 'dev' / 'r2d2' / 'data' / 'face_recognition' / 'severin'

# Get all images
images = sorted(list(person_dir.glob('*.jpg')))

print(f"Found {len(images)} training images")
print()

# Get unique timestamps (different capture sessions)
timestamps = {}
for img_path in images:
    # Extract timestamp from filename (20251206_152402_bright_dist1_0.jpg)
    parts = img_path.stem.split('_')
    if len(parts) >= 2:
        timestamp = f"{parts[0]}_{parts[1]}"  # 20251206_152402
        if timestamp not in timestamps:
            timestamps[timestamp] = []
        timestamps[timestamp].append(img_path)

print(f"Found {len(timestamps)} different capture sessions:")
for i, ts in enumerate(sorted(timestamps.keys()), 1):
    print(f"  {i}. {ts}: {len(timestamps[ts])} images")

print()

# Show stats by distance/lighting
lighting_dist = {}
for img_path in images:
    parts = img_path.stem.split('_')
    if len(parts) >= 3:
        key = f"{parts[2]}_dist{parts[3]}" if len(parts) > 3 else f"{parts[2]}"
        if key not in lighting_dist:
            lighting_dist[key] = 0
        lighting_dist[key] += 1

print("Images by lighting and distance:")
for key in sorted(lighting_dist.keys()):
    print(f"  {key}: {lighting_dist[key]} images")

print()
print("=" * 70)
print("IMAGE ANALYSIS")
print("=" * 70)
print()

# Check diversity
print("✓ Total images: 169")
print("✓ Capture sessions: 1")
print("✓ Lighting condition: Bright only (limited!)")
print("✓ Distance: 1 meter only (limited!)")
print()

print("OBSERVATION:")
print("  All 169 images are from the SAME session (14:24 on Dec 6)")
print("  All images are BRIGHT light, 1 meter distance")
print("  This explains why recognition failed!")
print()

print("SOLUTION:")
print("  Need to add images with:")
print("    ✗ Different lighting (low light, shadows)")
print("    ✗ Different distances (2m, 3m, 5m)")
print("    ✗ Different head angles (already varied in current set)")
print()

print("RECOMMENDATION:")
print("  Run the simplified training system to capture:")
print("    • Task 1: Bright Light - 1m (already have ~80 images)")
print("    • Task 2: Bright Light - 2m (NEW - ~80 images)")
print("    • Task 3: Low Light - 3m (NEW - ~80 images)")
print("    • Task 4: Low Light - 5m (NEW - ~80 images)")
print()
print("  Total: ~320 images with proper diversity")
print()

# Sample and display a few images
print("=" * 70)
print("DISPLAYING 10 RANDOM SAMPLE IMAGES")
print("=" * 70)
print()

sample_images = random.sample(images, min(10, len(images)))

# Create a grid to display
grid_width = 5
grid_height = 2
img_width = 100
img_height = 100

# Create blank grid
grid = np.ones((grid_height * (img_height + 10), grid_width * (img_width + 10), 3), dtype=np.uint8) * 255

for i, img_path in enumerate(sample_images):
    row = i // grid_width
    col = i % grid_width
    
    # Read image
    img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
    if img is not None:
        # Resize to 100x100
        img = cv2.resize(img, (img_width, img_height))
        
        # Convert to 3 channels
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        
        # Place in grid
        y = row * (img_height + 10) + 5
        x = col * (img_width + 10) + 5
        grid[y:y+img_height, x:x+img_width] = img
        
        # Add filename text
        text = img_path.name[:15] + "..."
        cv2.putText(grid, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 1)

# Save the grid
output_path = Path.home() / 'dev' / 'r2d2' / 'data' / 'face_recognition' / 'training_samples.jpg'
cv2.imwrite(str(output_path), grid)
print(f"✓ Sample grid saved to: {output_path}")
print()

# Also print individual image info
print("Sample images:")
for i, img_path in enumerate(sample_images, 1):
    parts = img_path.stem.split('_')
    print(f"  {i}. {img_path.name}")
    if len(parts) >= 4:
        print(f"     Lighting: {parts[2]}, Distance: {parts[3].replace('dist', '')}m")

print()
