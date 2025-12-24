#!/bin/bash
# Startup script for R2D2 Speech Node Service

# Navigate to workspace
cd /home/severin/dev/r2d2/ros2_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Source workspace overlay
source install/setup.bash

# Launch speech node (auto_start=false for gesture control)
exec ros2 launch r2d2_speech speech_node.launch.py auto_start:=false

