#!/bin/bash
# Check why gesture_intent_node isn't responding

cd /home/severin/dev/r2d2/ros2_ws
source install/setup.bash

echo "========================================"
echo "Gesture Intent Node Diagnostics"
echo "========================================"
echo ""

echo "1. Is gesture_intent_node running?"
echo "-----------------------------------"
ros2 node list | grep gesture_intent || echo "❌ NOT RUNNING!"
echo ""

echo "2. Gesture Intent Parameters:"
echo "-----------------------------"
echo "Enabled:"
ros2 param get /gesture_intent_node enabled
echo ""
echo "Audio feedback enabled:"
ros2 param get /gesture_intent_node audio_feedback_enabled
echo ""
echo "Auto shutdown enabled:"
ros2 param get /gesture_intent_node auto_shutdown_enabled
echo ""

echo "3. Checking Person Detection Status:"
echo "-------------------------------------"
echo "This is CRITICAL - gesture_intent only works if person is detected!"
echo ""
timeout 3 ros2 topic echo /r2d2/perception/is_target_person --once || echo "❌ NOT PUBLISHING!"
echo ""

echo "4. Checking gesture_intent_node logs:"
echo "--------------------------------------"
echo "Recent logs from gesture_intent service..."
sudo journalctl -u r2d2-gesture-intent.service -n 50 --no-pager | tail -30

echo ""
echo "========================================"
echo "Analysis"
echo "========================================"
echo ""
echo "If person detection is NOT publishing or shows 'false',"
echo "then gesture_intent will IGNORE all gestures!"
echo ""
echo "The gesture_intent_node requires:"
echo "  1. Person must be detected (is_target_person = true)"
echo "  2. Gesture detected (index_finger_up or fist)"
echo "  3. Cooldown period elapsed"

