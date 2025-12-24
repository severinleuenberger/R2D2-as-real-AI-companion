#!/bin/bash
# Real-time monitoring of the complete gesture system

cd /home/severin/dev/r2d2/ros2_ws
source install/setup.bash

echo "========================================"
echo "Real-Time Gesture System Monitor"
echo "========================================"
echo ""
echo "This will monitor all relevant topics simultaneously."
echo "Stand in front of the camera and make gestures!"
echo ""
echo "Gestures to try:"
echo "  ☝️  Index finger up (START speech service)"
echo "  ✊ Closed fist (STOP speech service)"
echo ""
echo "Press Ctrl+C to stop monitoring"
echo ""
echo "========================================"
echo ""

# Create a temp file for each topic
TEMP_DIR="/tmp/r2d2_monitor_$$"
mkdir -p "$TEMP_DIR"

# Function to clean up on exit
cleanup() {
    echo ""
    echo "Stopping monitors..."
    kill $(jobs -p) 2>/dev/null
    rm -rf "$TEMP_DIR"
    exit 0
}

trap cleanup INT TERM

# Monitor gesture events
(while true; do
    echo "[$(date +%H:%M:%S)] Gesture Event:" >> "$TEMP_DIR/gesture"
    timeout 2 ros2 topic echo /r2d2/perception/gesture_event --once 2>/dev/null >> "$TEMP_DIR/gesture" || echo "  (no event)" >> "$TEMP_DIR/gesture"
    sleep 0.5
done) &

# Monitor person detection
(while true; do
    echo "[$(date +%H:%M:%S)] Person Detected:" >> "$TEMP_DIR/person"
    timeout 2 ros2 topic echo /r2d2/perception/is_target_person --once 2>/dev/null >> "$TEMP_DIR/person" || echo "  (no data)" >> "$TEMP_DIR/person"
    sleep 1
done) &

# Monitor person ID
(while true; do
    echo "[$(date +%H:%M:%S)] Person ID:" >> "$TEMP_DIR/person_id"
    timeout 2 ros2 topic echo /r2d2/perception/person_id --once 2>/dev/null >> "$TEMP_DIR/person_id" || echo "  (no data)" >> "$TEMP_DIR/person_id"
    sleep 1
done) &

# Display combined output
sleep 2
tail -f "$TEMP_DIR/gesture" "$TEMP_DIR/person" "$TEMP_DIR/person_id" &

# Wait forever (until Ctrl+C)
wait

