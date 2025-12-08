#!/bin/bash
# R2D2 Audio Notification - Direct Node Launch
# Usage: ./start_audio_notification.sh [optional ROS args]

set -e

cd /home/severin/dev/r2d2/ros2_ws
source install/setup.bash

# Set ROS environment
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}

# Launch the node directly (avoids launch file libexec issue)
exec audio_notification_node "$@"
