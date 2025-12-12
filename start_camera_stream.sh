#!/bin/bash
# R2D2 Camera Stream Service - Direct Node Launch
# Serves MJPEG stream from camera feed via HTTP

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

# Launch the camera stream node
exec python3 -m r2d2_camera.camera_stream_node

