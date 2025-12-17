#!/bin/bash
# Complete Gesture Recognition Workflow Test
# This script guides you through testing the entire gesture system

echo "========================================"
echo "Gesture Recognition Workflow Test"
echo "========================================"
echo ""

echo "System Status:"
echo "--------------"
echo ""

# Check ROS2 topics
echo "✓ Checking ROS2 topics..."
cd /home/severin/dev/r2d2/ros2_ws
source install/setup.bash
ros2 topic list | grep -E "gesture|person" | sed 's/^/  /'
echo ""

# Check gesture intent node
echo "✓ Gesture Intent Node:"
ros2 node list | grep gesture | sed 's/^/  /'
echo ""

# Check parameters
echo "✓ Configuration:"
TIMEOUT=$(ros2 param get /gesture_intent_node auto_shutdown_timeout_seconds 2>/dev/null | grep -oP '\d+\.?\d*')
AUDIO=$(ros2 param get /gesture_intent_node audio_feedback_enabled 2>/dev/null | grep -oP '(True|False)')
echo "  - Watchdog timeout: ${TIMEOUT}s"
echo "  - Audio feedback: ${AUDIO}"
echo ""

# Check audio file
echo "✓ Audio file:"
if [ -f "/home/severin/dev/r2d2/assets/audio/r2d2_beep.mp3" ]; then
    echo "  - r2d2_beep.mp3 exists ✓"
else
    echo "  - r2d2_beep.mp3 MISSING ✗"
fi
echo ""

# Check person detection
echo "✓ Person Detection:"
PERSON_STATUS=$(timeout 2 ros2 topic echo /r2d2/perception/is_target_person --once 2>/dev/null | grep "data:" | awk '{print $2}')
if [ "$PERSON_STATUS" == "true" ]; then
    echo "  - Target person detected: YES ✓"
elif [ "$PERSON_STATUS" == "false" ]; then
    echo "  - Target person detected: NO (stand in front of camera)"
else
    echo "  - Cannot determine status (check camera service)"
fi
echo ""

echo "========================================"
echo "Manual Test Procedure"
echo "========================================"
echo ""
echo "Test 1: Start Conversation with Open Palm"
echo "-------------------------------------------"
echo "1. Stand in front of the camera"
echo "2. Show an OPEN PALM gesture to the camera"
echo "3. Expected results:"
echo "   - You should hear R2D2 beep sound"
echo "   - LED should turn GREEN (speech service active)"
echo "   - Speech recognition should be ready"
echo "   - You can check with: systemctl is-active r2d2-speech-service"
echo ""

echo "Test 2: Stop Conversation with Closed Fist"
echo "--------------------------------------------"
echo "1. While speech service is running, show a CLOSED FIST"
echo "2. Expected results:"
echo "   - You should hear R2D2 beep sound"
echo "   - LED should turn back to BLUE"
echo "   - Speech service should stop"
echo "   - You can check with: systemctl is-active r2d2-speech-service"
echo ""

echo "Test 3: Auto-Shutdown (35-second watchdog)"
echo "--------------------------------------------"
echo "1. Start conversation with open palm"
echo "2. Wait 35 seconds WITHOUT making any gestures"
echo "3. Expected results:"
echo "   - After 35 seconds, you should hear R2D2 beep"
echo "   - Speech service should auto-stop"
echo "   - LED should turn back to BLUE"
echo ""

echo "Test 4: Auto-Shutdown on Person Leave"
echo "---------------------------------------"
echo "1. Start conversation with open palm"
echo "2. Walk away from the camera (person no longer detected)"
echo "3. Wait 35 seconds"
echo "4. Expected results:"
echo "   - After 35 seconds, speech service should auto-stop"
echo "   - LED should turn back to BLUE"
echo ""

echo "========================================"
echo "Monitoring Commands"
echo "========================================"
echo ""
echo "Real-time gesture events:"
echo "  ros2 topic echo /r2d2/perception/gesture_event"
echo ""
echo "Person detection status:"
echo "  ros2 topic echo /r2d2/perception/is_target_person"
echo ""
echo "Camera service logs:"
echo "  sudo journalctl -u r2d2-camera-perception.service -f"
echo ""
echo "Gesture intent logs:"
echo "  sudo journalctl -u r2d2-gesture-intent.service -f"
echo ""
echo "Speech service status:"
echo "  systemctl is-active r2d2-speech-service"
echo ""

echo "========================================"
echo "Ready to test! Follow the procedure above."
echo "========================================"

