#!/bin/bash
################################################################################
# Launch Gesture Intent Node
#
# This node controls the speech service based on gesture events
################################################################################

echo "========================================================================"
echo "           Launching Gesture Intent Node"
echo "========================================================================"
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/dev/r2d2/ros2_ws/install/setup.bash

echo "Starting gesture intent node..."
echo ""
echo "This node will:"
echo "  • Listen for gesture events from image_listener"
echo "  • Control speech service (start/stop)"
echo "  • Play audio feedback (R2D2 beeps)"
echo "  • Auto-shutdown after 5 min no presence"
echo ""
echo "========================================================================"
echo ""

# Launch with all features enabled
ros2 launch r2d2_gesture gesture_intent.launch.py \
    enabled:=true \
    cooldown_start_seconds:=5.0 \
    cooldown_stop_seconds:=3.0 \
    auto_shutdown_enabled:=true \
    auto_shutdown_timeout_seconds:=300.0 \
    auto_restart_on_return:=false \
    audio_feedback_enabled:=true

