#!/bin/bash
# Complete test for audio notification system

echo "🔍 R2D2 Audio Notification System - Complete Test"
echo "=================================================="
echo ""

# Source ROS 2 environment
source /opt/ros/humble/setup.bash 2>/dev/null
if [ -f ~/dev/r2d2/ros2_ws/install/setup.bash ]; then
    source ~/dev/r2d2/ros2_ws/install/setup.bash
fi

echo "Step 1: Check if audio notification node is running..."
if ros2 node list | grep -q audio_notification_node; then
    echo "  ✅ Audio notification node is running"
else
    echo "  ❌ Audio notification node NOT running"
    echo "     Start it with: sudo systemctl start r2d2-audio-notification.service"
    exit 1
fi

echo ""
echo "Step 2: Check target_person parameter..."
TARGET=$(ros2 param get /audio_notification_node target_person 2>&1 | grep "String value is" | awk -F"'" '{print $2}' || echo "unknown")
echo "  Target person: $TARGET"

echo ""
echo "Step 3: Check if perception pipeline is publishing..."
echo "  Monitoring /r2d2/perception/person_id for 5 seconds..."
echo "  (Stand in front of the camera now!)"
timeout 5 ros2 topic echo /r2d2/perception/person_id 2>&1 | head -20 || echo "  ⚠️  No messages received - perception pipeline may not be running"

echo ""
echo "Step 4: Check audio status messages..."
echo "  Monitoring /r2d2/audio/person_status for 3 seconds..."
timeout 3 ros2 topic echo /r2d2/audio/person_status --once 2>&1 | head -10 || echo "  ⚠️  No status messages"

echo ""
echo "Step 5: Check audio notification events..."
echo "  Monitoring /r2d2/audio/notification_event for 3 seconds..."
timeout 3 ros2 topic echo /r2d2/audio/notification_event --once 2>&1 | head -10 || echo "  ⚠️  No event messages"

echo ""
echo "=================================================="
echo "📊 Test Summary:"
echo "  If you saw person_id messages → Perception is working"
echo "  If you saw person_status messages → Audio node is working"
echo "  If you heard audio → System is fully functional!"
echo ""
echo "If no messages, check:"
echo "  1. Is perception pipeline running?"
echo "     ros2 launch r2d2_bringup r2d2_camera_perception.launch.py enable_face_recognition:=true"
echo "  2. Is face recognition model trained?"
echo "     ls ~/dev/r2d2/data/face_recognition/models/"
echo "  3. Does target_person match the published name?"

