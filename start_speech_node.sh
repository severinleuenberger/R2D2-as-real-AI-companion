#!/bin/bash
# Startup script for R2D2 Speech Node Service

# Navigate to workspace
cd /home/severin/dev/r2d2/ros2_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Source workspace overlay
source install/setup.bash

# Launch speech node
exec ros2 launch r2d2_speech speech_node.launch.py

