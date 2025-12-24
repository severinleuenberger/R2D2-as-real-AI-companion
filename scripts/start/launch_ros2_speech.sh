#!/bin/bash
# Quick launcher for R2D2 Speech ROS2 Node

echo "R2D2 Speech System - ROS2 Node"
echo "==============================="
echo ""

# Check for API key
if [ ! -f ~/.r2d2/.env ]; then
    echo "ERROR: ~/.r2d2/.env not found"
    echo "Please create it with:"
    echo "  OPENAI_API_KEY=sk-..."
    exit 1
fi

# Activate Python venv FIRST (before ROS2)
echo "Activating Python environment..."
source ~/dev/r2d2/r2d2_speech_env/bin/activate

# Source ROS2 AFTER virtualenv so it uses venv's Python
echo "Sourcing ROS2..."
source /opt/ros/humble/setup.bash
source ~/dev/r2d2/ros2_ws/install/setup.bash

# Export PYTHONPATH to ensure existing r2d2_speech package is found
export PYTHONPATH=$HOME/dev/r2d2:$PYTHONPATH

# Ensure ROS2 uses the virtualenv's Python
export ROS_PYTHON_VERSION=3
export PYTHONUNBUFFERED=1

echo ""
echo "Launching speech node..."
echo ""
echo "Using Python: $(which python3)"
echo "The node will auto-configure and auto-activate."
echo "Watch for 'Speech system running' message."
echo ""
echo "Open other terminals to:"
echo "  - Monitor: ros2 topic echo /r2d2/speech/user_transcript"
echo "  - Status:  ros2 lifecycle get /speech_node"
echo ""
echo "Press Ctrl+C to stop."
echo ""

# Launch
ros2 launch r2d2_speech speech_node.launch.py

