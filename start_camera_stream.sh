#!/bin/bash
# R2D2 Camera Stream Service - Direct Node Launch
# Serves MJPEG stream from camera feed via HTTP

set -e

cd /home/severin/dev/r2d2/ros2_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash 2>/dev/null || true
if [ -f ~/dev/r2d2/ros2_ws/install/setup.bash ]; then
    source ~/dev/r2d2/ros2_ws/install/setup.bash
fi

# Source depthai environment if it exists
if [ -f ~/depthai_env/bin/activate ]; then
    source ~/depthai_env/bin/activate
fi

# Set ROS environment
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}

# Critical for ARM64 (Jetson AGX Orin)
export OPENBLAS_CORETYPE=ARMV8

# Launch the camera stream node
exec python3 -m r2d2_camera.camera_stream_node

