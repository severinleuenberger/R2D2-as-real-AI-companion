#!/bin/bash
# Test the speech service manually

cd /home/severin/dev/r2d2/ros2_ws
source install/setup.bash

echo "========================================"
echo "Testing Speech Service Manually"
echo "========================================"
echo ""

echo "1. Calling START session service..."
echo "   You should hear a beep!"
echo ""
ros2 service call /r2d2/speech/start_session std_srvs/srv/Trigger
echo ""

echo "Waiting 3 seconds..."
sleep 3
echo ""

echo "2. Calling STOP session service..."
echo "   You should hear another beep!"
echo ""
ros2 service call /r2d2/speech/stop_session std_srvs/srv/Trigger
echo ""

echo "========================================"
echo "Did you hear TWO beeps?"
echo "========================================"
echo ""
echo "If YES: The audio system works, but gesture_intent"
echo "        isn't calling the service when gestures detected"
echo ""
echo "If NO: There's an audio configuration issue"

