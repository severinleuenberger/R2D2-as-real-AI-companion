#!/bin/bash
# Fix gesture model path parameter name in service file

set -e

echo "========================================"
echo "Fixing Gesture Model Path Parameter"
echo "========================================"
echo ""

SERVICE_FILE="/etc/systemd/system/r2d2-camera-perception.service"

echo "Current ExecStart line:"
grep "ExecStart" "$SERVICE_FILE"
echo ""

echo "Fixing parameter name: gesture_recognition_model_path → gesture_model_path"
sudo sed -i 's|gesture_recognition_model_path:=|gesture_model_path:=|' "$SERVICE_FILE"

echo ""
echo "New ExecStart line:"
grep "ExecStart" "$SERVICE_FILE"
echo ""

echo "Rebuilding packages..."
cd /home/severin/dev/r2d2/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select r2d2_perception r2d2_bringup --symlink-install
echo ""

echo "Reloading systemd daemon..."
sudo systemctl daemon-reload
echo ""

echo "Restarting camera-perception service..."
sudo systemctl restart r2d2-camera-perception.service
sleep 5
echo ""

if systemctl is-active --quiet r2d2-camera-perception.service; then
    echo "✅ Service restarted successfully"
else
    echo "❌ Service failed to start!"
    exit 1
fi
echo ""

echo "Verifying parameters..."
source install/setup.bash
echo ""

echo "Face recognition person:"
ros2 param get /image_listener target_person_name
echo ""

echo "Gesture recognition person:"
ros2 param get /image_listener target_person_gesture_name
echo ""

echo "Gesture model path:"
ros2 param get /image_listener gesture_model_path
echo ""

# Check if file exists
MODEL_PATH=$(ros2 param get /image_listener gesture_model_path 2>/dev/null | sed 's/String value is: //')
if [ -f "$MODEL_PATH" ]; then
    echo "✅ Model file exists: $MODEL_PATH"
else
    echo "❌ Model file NOT FOUND: $MODEL_PATH"
fi
echo ""

echo "========================================"
echo "Testing Gesture Detection"
echo "========================================"
echo ""

PERSON_STATUS=$(timeout 2 ros2 topic echo /r2d2/perception/is_target_person --once 2>/dev/null | grep "data:" | awk '{print $2}')
if [ "$PERSON_STATUS" == "true" ]; then
    echo "✅ Target person detected"
elif [ "$PERSON_STATUS" == "false" ]; then
    echo "⚠️  No person detected - stand in front of camera"
else
    echo "⚠️  Cannot check person status"
fi
echo ""

echo "Make an OPEN PALM gesture now!"
echo "Listening for 10 seconds..."
echo ""

timeout 10 ros2 topic echo /r2d2/perception/gesture_event || echo "⚠️  No gestures detected"
echo ""

echo "========================================"
echo "✅ Fix Complete!"
echo "========================================"
echo ""
echo "Expected values:"
echo "  - target_person_name: severin"
echo "  - target_person_gesture_name: severin"
echo "  - gesture_model_path: .../severin_gesture_classifier.pkl"
echo ""
echo "If all values are correct, gestures should now be detected!"

