#!/bin/bash
# Simple gesture watcher - shows gestures as they happen

cd /home/severin/dev/r2d2/ros2_ws
source install/setup.bash

echo "========================================"
echo "👀 Watching for Gestures"
echo "========================================"
echo ""
echo "Make gestures now:"
echo "  ☝️  Index finger up (should START speech)"
echo "  ✊ Closed fist (should STOP speech)"
echo ""
echo "Press Ctrl+C to stop"
echo ""
echo "Gesture events will appear below:"
echo "---"
echo ""

# Just watch the gesture topic in real-time
ros2 topic echo /r2d2/perception/gesture_event

