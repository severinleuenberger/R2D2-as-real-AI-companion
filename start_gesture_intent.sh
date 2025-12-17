#!/bin/bash
################################################################################
# R2D2 Gesture Intent Service Startup Script
# Launches gesture_intent_node with production parameters
################################################################################

# Source ROS 2
source /opt/ros/humble/setup.bash
source /home/severin/dev/r2d2/ros2_ws/install/setup.bash

# Launch with production parameters
exec ros2 launch r2d2_gesture gesture_intent.launch.py \
    enabled:=true \
    cooldown_start_seconds:=5.0 \
    cooldown_stop_seconds:=3.0 \
    auto_shutdown_enabled:=true \
    auto_shutdown_timeout_seconds:=35.0 \
    auto_restart_on_return:=false \
    audio_feedback_enabled:=true

