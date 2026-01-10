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

# PulseAudio environment for systemd service (required for audio playback)
export XDG_RUNTIME_DIR=/run/user/$(id -u)
export PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native

# Critical for ARM64 (Jetson AGX Orin)
export OPENBLAS_CORETYPE=ARMV8

# Launch the audio notification node
# Centralized config: config/audio_params.yaml (audio_volume: 0.30)
# Both code default and config file set to 0.30 (30%) for consistency
# Default parameters:
#   target_person: severin
#   audio_volume: 0.30 (30% - set in code and config, can override via audio_volume:=X)
#   jitter_tolerance_seconds: 5.0
#   loss_confirmation_seconds: 15.0
#   recognition_audio_file: Voicy_R2-D2 - 2.mp3
#   loss_audio_file: Voicy_R2-D2 - 5.mp3
#
# Example overrides:
#   ./start_audio_notification.sh audio_volume:=0.5

# Launch all audio services (includes audio_notification_node + GPIO LED node + logger)
exec ros2 launch r2d2_audio all_audio_services.launch.py "$@"
