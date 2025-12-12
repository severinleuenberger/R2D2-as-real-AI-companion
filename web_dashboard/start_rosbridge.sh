#!/bin/bash
# Simple script to start rosbridge_server for web dashboard

set -e

# Source ROS 2 environment
source /opt/ros/humble/setup.bash 2>/dev/null || true
if [ -f ~/dev/r2d2/ros2_ws/install/setup.bash ]; then
    source ~/dev/r2d2/ros2_ws/install/setup.bash
fi

# Start rosbridge_websocket
echo "Starting rosbridge_server on port 9090..."
ros2 run rosbridge_server rosbridge_websocket \
    --ros-args \
    -p port:=9090 \
    -p address:=0.0.0.0 &

ROSBRIDGE_PID=$!

# Start rosapi (optional, but useful)
echo "Starting rosapi..."
ros2 run rosapi rosapi_node &

ROSAPI_PID=$!

echo "rosbridge_server started (PID: $ROSBRIDGE_PID)"
echo "rosapi started (PID: $ROSAPI_PID)"
echo "WebSocket available at: ws://$(hostname -I | awk '{print $1}'):9090"
echo ""
echo "Press Ctrl+C to stop"

# Wait for processes
wait


