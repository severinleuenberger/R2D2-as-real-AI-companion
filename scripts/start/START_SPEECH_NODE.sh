#!/bin/bash
# Start the speech_node manually for testing

cd /home/severin/dev/r2d2/ros2_ws
source install/setup.bash

echo "========================================"
echo "Starting Speech Node"
echo "========================================"
echo ""
echo "This will start the speech_node which provides"
echo "the /r2d2/speech/start_session and stop_session services."
echo ""
echo "Leave this terminal open - the node will run here."
echo "Press Ctrl+C to stop it."
echo ""
echo "========================================"
echo ""

# Launch the speech node
ros2 launch r2d2_speech speech_node.launch.py

