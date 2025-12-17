#!/bin/bash
# Comprehensive diagnostic for gesture system

echo "========================================"
echo "Gesture System Diagnostics"
echo "========================================"
echo ""

cd /home/severin/dev/r2d2/ros2_ws
source install/setup.bash

echo "1. Checking ROS2 Nodes"
echo "----------------------"
ros2 node list | grep -E "(image_listener|gesture_intent|camera)"
echo ""

echo "2. Checking image_listener parameters"
echo "--------------------------------------"
echo "Face recognition enabled:"
ros2 param get /image_listener enable_face_recognition 2>/dev/null || echo "  ERROR: Cannot get parameter"
echo ""
echo "Gesture recognition enabled:"
ros2 param get /image_listener enable_gesture_recognition 2>/dev/null || echo "  ERROR: Cannot get parameter"
echo ""
echo "Target person name:"
ros2 param get /image_listener target_person_name 2>/dev/null || echo "  ERROR: Cannot get parameter"
echo ""

echo "3. Checking Topic Publishing Status"
echo "------------------------------------"
echo "Gesture events:"
timeout 1 ros2 topic hz /r2d2/perception/gesture_event 2>&1 | head -3
echo ""
echo "Person detection:"
timeout 1 ros2 topic hz /r2d2/perception/is_target_person 2>&1 | head -3
echo ""
echo "Person ID:"
timeout 1 ros2 topic hz /r2d2/perception/person_id 2>&1 | head -3
echo ""

echo "4. Testing Topic Content (5 second sample)"
echo "-------------------------------------------"
echo "Stand in front of camera and make gestures..."
echo ""

# Monitor both topics simultaneously
echo "Monitoring person_id:"
timeout 5 ros2 topic echo /r2d2/perception/person_id --once 2>&1 &
PID1=$!

echo "Monitoring is_target_person:"
timeout 5 ros2 topic echo /r2d2/perception/is_target_person --once 2>&1 &
PID2=$!

echo "Monitoring gesture_event:"
timeout 5 ros2 topic echo /r2d2/perception/gesture_event --once 2>&1 &
PID3=$!

wait $PID1 $PID2 $PID3
echo ""

echo "========================================"
echo "5. Checking Service Logs (last 30 lines)"
echo "========================================"
echo ""
echo "Requesting sudo access for log viewing..."
sudo journalctl -u r2d2-camera-perception.service -n 30 --no-pager | grep -E "(ERROR|WARN|person|gesture|recognition)" | tail -20

echo ""
echo "========================================"
echo "Diagnostic Complete"
echo "========================================"

