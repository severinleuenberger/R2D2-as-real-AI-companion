#!/bin/bash
# Rebuild packages and test gesture recognition

set -e

echo "========================================"
echo "Rebuilding ROS2 Packages"
echo "========================================"
echo ""

cd /home/severin/dev/r2d2/ros2_ws
source /opt/ros/humble/setup.bash

echo "Building r2d2_perception..."
colcon build --packages-select r2d2_perception --symlink-install
echo ""

echo "Building r2d2_bringup..."
colcon build --packages-select r2d2_bringup --symlink-install
echo ""

echo "✅ Packages rebuilt"
echo ""

echo "========================================"
echo "Restarting Camera Perception Service"
echo "========================================"
echo ""

sudo systemctl daemon-reload
sudo systemctl restart r2d2-camera-perception.service
sleep 5

if systemctl is-active --quiet r2d2-camera-perception.service; then
    echo "✅ Service restarted successfully"
else
    echo "❌ Service failed to start!"
    exit 1
fi
echo ""

echo "========================================"
echo "Verifying Parameters"
echo "========================================"
echo ""

source install/setup.bash

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
echo "Testing Gesture Detection"
echo "========================================"
echo ""

# Check if person is detected
PERSON_STATUS=$(timeout 2 ros2 topic echo /r2d2/perception/is_target_person --once 2>/dev/null | grep "data:" | awk '{print $2}')
if [ "$PERSON_STATUS" == "true" ]; then
    echo "✅ Target person (severin) is detected"
elif [ "$PERSON_STATUS" == "false" ]; then
    echo "⚠️  No person detected - stand in front of camera"
else
    echo "⚠️  Cannot check person status"
fi
echo ""

echo "Make an OPEN PALM gesture now!"
echo "Listening for gesture events for 10 seconds..."
echo ""

timeout 10 ros2 topic echo /r2d2/perception/gesture_event || echo "No gestures detected"
echo ""

echo "========================================"
echo "✅ Setup Complete!"
echo "========================================"
echo ""
echo "Both parameters should now be 'severin'."
echo "If you see gesture events above, the system is working!"
echo ""
echo "Expected workflow:"
echo "  1. Open palm → Start speech service + beep"
echo "  2. Closed fist → Stop speech service + beep"
echo "  3. No gesture for 35s → Auto-stop + beep"

