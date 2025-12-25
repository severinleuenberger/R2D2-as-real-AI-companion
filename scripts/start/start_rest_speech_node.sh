#!/bin/bash
# Startup script for R2D2 REST Speech Node Service (Intelligent Mode)

# Navigate to workspace
cd /home/severin/dev/r2d2/ros2_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Source workspace overlay
source install/setup.bash

# Launch REST speech node (Intelligent Mode with o1-preview)
exec ros2 launch r2d2_speech rest_speech_node.launch.py

