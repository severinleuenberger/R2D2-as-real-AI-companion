#!/bin/bash
# Fix the r2d2-camera-perception.service file to add target_person_name parameter

set -e

echo "========================================"
echo "Fixing Camera Perception Service File"
echo "========================================"
echo ""

SERVICE_FILE="/etc/systemd/system/r2d2-camera-perception.service"

echo "Current ExecStart line:"
grep "ExecStart" "$SERVICE_FILE"
echo ""

echo "Creating fixed version..."
sudo sed -i 's|enable_face_recognition:=true enable_gesture_recognition|enable_face_recognition:=true target_person_name:=severin enable_gesture_recognition|' "$SERVICE_FILE"

echo ""
echo "New ExecStart line:"
grep "ExecStart" "$SERVICE_FILE"
echo ""

echo "Reloading systemd daemon..."
sudo systemctl daemon-reload
echo ""

echo "Restarting camera-perception service..."
sudo systemctl restart r2d2-camera-perception.service
sleep 3
echo ""

if systemctl is-active --quiet r2d2-camera-perception.service; then
    echo "✅ Service restarted successfully"
else
    echo "❌ Service failed to start!"
    exit 1
fi
echo ""

echo "Verifying parameters..."
sleep 2
cd /home/severin/dev/r2d2/ros2_ws
source install/setup.bash
echo ""
echo "Gesture model path:"
ros2 param get /image_listener gesture_model_path
echo ""
echo "Target person gesture name:"
ros2 param get /image_listener target_person_gesture_name
echo ""

echo "========================================"
echo "✅ Fix Complete!"
echo "========================================"
echo ""
echo "The gesture model should now be:"
echo "  severin_gesture_classifier.pkl"
echo ""
echo "Try making gestures again!"

