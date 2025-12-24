#!/bin/bash
# R2D2 Audio Notification Service Startup Script
# This script properly initializes the ROS 2 environment and launches the audio notification node

set -e

# Navigate to workspace
cd /home/severin/dev/r2d2/ros2_ws

# Source the setup script
source install/setup.bash

# Set ROS 2 variables
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}

# Launch the audio notification node
exec ros2 launch r2d2_audio audio_notification.launch.py
