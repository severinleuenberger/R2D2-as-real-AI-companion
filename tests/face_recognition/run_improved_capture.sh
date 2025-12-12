#!/bin/bash
# Launcher script for improved face recognition training data capture

set -e

echo "=========================================="
echo "R2D2 Improved Face Training Data Capture"
echo "=========================================="
echo ""
echo "This script will capture 40-50 high-quality training images."
echo "You will be guided through 5 stages:"
echo "  1. Bright direct light (10 images)"
echo "  2. Dim indoor light (10 images)"
echo "  3. Side profile 45° (10 images)"
echo "  4. Varied distances (10 images)"
echo "  5. Facial expressions (10 images)"
echo ""
echo "Total time: ~50 seconds"
echo ""
read -p "Press ENTER when ready to begin (make sure you're in front of the camera)..."

# Activate environment
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8

# Run capture script
cd "$(dirname "$0")"
python3 1_capture_training_data.py

echo ""
echo "=========================================="
echo "Capture complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "  1. Review captured images: ls -lh ~/dev/r2d2/data/face_recognition/severin/"
echo "  2. Train model: python3 2_train_recognizer.py"
echo ""

