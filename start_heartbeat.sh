#!/bin/bash
# R2D2 Heartbeat Service - Direct Node Launch
# Publishes system health metrics (CPU, GPU, temperature) via ROS 2

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

# Launch the heartbeat node
exec python3 -m r2d2_hello.heartbeat_node

