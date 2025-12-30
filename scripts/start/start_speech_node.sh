#!/bin/bash
# Startup script for R2D2 Speech Node Service

# PulseAudio environment for user session audio
export XDG_RUNTIME_DIR=/run/user/1000
export PULSE_SERVER=unix:/run/user/1000/pulse/native

# Navigate to workspace
cd /home/severin/dev/r2d2/ros2_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Source workspace overlay
source install/setup.bash

# Launch speech node (auto_start=false for gesture control)
exec ros2 launch r2d2_speech speech_node.launch.py auto_start:=false

