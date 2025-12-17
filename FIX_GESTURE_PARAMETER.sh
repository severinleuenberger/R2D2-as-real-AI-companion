#!/bin/bash
# Fix: Add target_person_gesture_name parameter to service file

set -e

echo "========================================"
echo "Adding Gesture Person Name Parameter"
echo "========================================"
echo ""

SERVICE_FILE="/etc/systemd/system/r2d2-camera-perception.service"

echo "Current ExecStart line:"
grep "ExecStart" "$SERVICE_FILE"
echo ""

echo "Adding target_person_gesture_name:=severin..."
sudo sed -i 's|target_person_name:=severin enable_gesture_recognition|target_person_name:=severin target_person_gesture_name:=severin enable_gesture_recognition|' "$SERVICE_FILE"

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
sleep 3
cd /home/severin/dev/r2d2/ros2_ws
source install/setup.bash
echo ""
echo "Face recognition person name:"
ros2 param get /image_listener target_person_name
echo ""
echo "Gesture recognition person name:"
ros2 param get /image_listener target_person_gesture_name
echo ""
echo "Gesture model path:"
ros2 param get /image_listener gesture_model_path
echo ""

echo "========================================"
echo "Testing gesture detection..."
echo "========================================"
echo ""
echo "Make an OPEN PALM gesture now!"
echo "Waiting 10 seconds for gesture events..."
echo ""

timeout 10 ros2 topic echo /r2d2/perception/gesture_event || echo "No gestures detected in 10 seconds"
echo ""

echo "========================================"
echo "✅ Setup Complete!"
echo "========================================"
echo ""
echo "Both parameters should now be 'severin':"
echo "  - target_person_name: severin"
echo "  - target_person_gesture_name: severin"
echo ""
echo "Try making gestures - you should hear beeps!"

