#!/bin/bash
# R2D2 Rosbridge Service - WebSocket Bridge for Web Dashboard
# Provides WebSocket interface for ROS 2 topics (port 9090)

set -e

cd /home/severin/dev/r2d2

# Source ROS 2 environment
source /opt/ros/humble/setup.bash 2>/dev/null || true
if [ -f ~/dev/r2d2/ros2_ws/install/setup.bash ]; then
    source ~/dev/r2d2/ros2_ws/install/setup.bash
fi

# Set ROS environment
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}

# Start rosbridge_websocket (main process for systemd)
# This will be the primary process that systemd monitors
exec ros2 run rosbridge_server rosbridge_websocket \
    --ros-args \
    -p port:=9090 \
    -p address:=0.0.0.0
