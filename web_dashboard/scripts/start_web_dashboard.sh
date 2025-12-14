#!/bin/bash
# Startup script for R2D2 Web Dashboard
# Starts both rosbridge and FastAPI web server

# Don't exit on error for background processes
set +e

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEB_DASHBOARD_DIR="$(dirname "$SCRIPT_DIR")"

# Source ROS 2 environment
source /opt/ros/humble/setup.bash 2>/dev/null || true
if [ -f ~/dev/r2d2/ros2_ws/install/setup.bash ]; then
    source ~/dev/r2d2/ros2_ws/install/setup.bash
fi

# Activate Python virtual environment if it exists
if [ -f "$WEB_DASHBOARD_DIR/web_dashboard_env/bin/activate" ]; then
    source "$WEB_DASHBOARD_DIR/web_dashboard_env/bin/activate"
fi

# Set environment variables
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export OPENBLAS_CORETYPE=ARMV8

# Change to web dashboard directory
cd "$WEB_DASHBOARD_DIR"

# Start rosbridge in background
echo "Starting rosbridge_server..."
# Start rosbridge_websocket and rosapi nodes directly
# Note: Output goes to systemd journal automatically
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090 -p address:=0.0.0.0 &
ROSBRIDGE_PID=$!

# Start rosapi for service/topic introspection (if available)
ros2 run rosapi rosapi_node &
ROSAPI_PID=$!

# Wait a moment for rosbridge to start
sleep 3

# Check if rosbridge started successfully
if ! kill -0 $ROSBRIDGE_PID 2>/dev/null; then
    echo "ERROR: rosbridge_websocket failed to start (PID: $ROSBRIDGE_PID)"
    echo "Check ROS 2 environment and rosbridge_server installation"
else
    echo "rosbridge_websocket started successfully (PID: $ROSBRIDGE_PID)"
fi

# Start FastAPI web server
echo "Starting web dashboard server..."
python3 -m app.main

# Cleanup on exit
trap "kill $ROSBRIDGE_PID $ROSAPI_PID 2>/dev/null" EXIT


