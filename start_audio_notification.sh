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
#   audio_volume: 0.05
#   jitter_tolerance_seconds: 5.0
#   loss_confirmation_seconds: 15.0
#   recognition_audio_file: Voicy_R2-D2 - 2.mp3
#   loss_audio_file: Voicy_R2-D2 - 5.mp3
#
# Example overrides:
#   ./start_audio_notification.sh "target_person:=alice" "audio_volume:=0.3"

exec python3 -m r2d2_audio.audio_notification_node "$@"
