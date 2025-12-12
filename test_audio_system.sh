#!/bin/bash
# Quick test script for audio notification system

echo "=== R2D2 Audio Notification System Test ==="
echo ""

# Source ROS 2 environment
source /opt/ros/humble/setup.bash 2>/dev/null
if [ -f ~/dev/r2d2/ros2_ws/install/setup.bash ]; then
    source ~/dev/r2d2/ros2_ws/install/setup.bash
fi

echo "1. Checking service status..."
systemctl --user status r2d2-audio-notification.service 2>&1 | head -5 || sudo systemctl status r2d2-audio-notification.service --no-pager 2>&1 | head -5

echo ""
echo "2. Checking ROS 2 nodes..."
ros2 node list 2>&1 | grep -E "audio|perception|camera" || echo "  No relevant nodes found"

echo ""
echo "3. Checking ROS 2 topics..."
echo "  Topics available:"
ros2 topic list 2>&1 | grep -E "person_id|audio" || echo "  No relevant topics found"

echo ""
echo "4. Monitoring /r2d2/perception/person_id for 10 seconds..."
timeout 10 ros2 topic echo /r2d2/perception/person_id --once 2>&1 || echo "  No messages received"

echo ""
echo "5. Monitoring /r2d2/audio/person_status for 3 seconds..."
timeout 3 ros2 topic echo /r2d2/audio/person_status --once 2>&1 || echo "  No status messages received"

echo ""
echo "=== Test Complete ==="
echo ""
echo "If you see messages above, the system is working."
echo "If you don't see messages, the perception pipeline may not be running."

