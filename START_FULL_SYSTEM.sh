#!/bin/bash
# Start the full R2D2 perception and audio system

echo "🚀 Starting R2D2 Full System"
echo "=============================="
echo ""

cd ~/dev/r2d2/ros2_ws

# Source environments
source ~/depthai_env/bin/activate
source ~/.bashrc
source install/setup.bash

echo "Step 1: Checking if model exists..."
MODEL_FILE="$HOME/dev/r2d2/data/face_recognition/models/severin_lbph.xml"
if [ -f "$MODEL_FILE" ]; then
    echo "  ✅ Model found: $MODEL_FILE"
    ls -lh "$MODEL_FILE"
else
    echo "  ❌ Model NOT found: $MODEL_FILE"
    echo "  Please train a model first using train_manager.py"
    exit 1
fi

echo ""
echo "Step 2: Starting perception pipeline with face recognition..."
echo "  This will run in the foreground. Press Ctrl+C to stop."
echo ""
echo "Launching: ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \\"
echo "  enable_face_recognition:=true \\"
echo "  target_person:=severin"
echo ""

ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true \
  target_person:=severin

