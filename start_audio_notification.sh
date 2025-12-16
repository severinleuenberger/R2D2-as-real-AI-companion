#!/bin/bash
# R2D2 Audio Notification - Direct Node Launch
# Updated: December 8, 2025
# Features: 15-second loss confirmation + parameterizable audio files

set -e

cd /home/severin/dev/r2d2/ros2_ws

# Source environment in correct order (critical for ARM)
source ~/depthai_env/bin/activate
source ~/.bashrc
source install/setup.bash

# Set ROS environment
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}

# Critical for ARM64 (Jetson AGX Orin)
export OPENBLAS_CORETYPE=ARMV8

# Launch the audio notification node with optional parameters
# Default parameters:
#   target_person: severin
#   audio_volume: 0.02 (2% - very quiet, as specified)
#   recognition_audio_file: Voicy_R2-D2 - 2.mp3
#   loss_audio_file: Voicy_R2-D2 - 5.mp3
# Note: Timing constants (REACQUIRE_WINDOW=5.0s, RED_HOLD_TIME=15.0s) are fixed in code
#
# Example overrides:
#   ./start_audio_notification.sh "target_person:=alice" "audio_volume:=0.5"

# Set default volume to 0.02 (2% - very quiet) if not provided via command line
if [[ "$*" != *"audio_volume"* ]]; then
    exec python3 -m r2d2_audio.audio_notification_node audio_volume:=0.02 "$@"
else
    exec python3 -m r2d2_audio.audio_notification_node "$@"
fi
