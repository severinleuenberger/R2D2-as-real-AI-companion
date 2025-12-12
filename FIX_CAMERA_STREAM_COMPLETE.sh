#!/bin/bash
# Complete fix for camera stream service

set -e

echo "=========================================="
echo "Fixing Camera Stream Service"
echo "=========================================="
echo ""

cd /home/severin/dev/r2d2

# Step 1: Rebuild the camera package
echo "Step 1: Rebuilding camera_stream_node..."
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select r2d2_camera
source install/setup.bash
cd ..

# Step 2: Update service file in systemd
echo ""
echo "Step 2: Updating service file in systemd..."
sudo cp r2d2-camera-stream.service /etc/systemd/system/
sudo chmod 644 /etc/systemd/system/r2d2-camera-stream.service

# Step 3: Reload systemd
echo ""
echo "Step 3: Reloading systemd..."
sudo systemctl daemon-reload

# Step 4: Reset failed state
echo ""
echo "Step 4: Resetting failed state..."
sudo systemctl reset-failed r2d2-camera-stream.service

# Step 5: Start the service
echo ""
echo "Step 5: Starting service..."
sudo systemctl start r2d2-camera-stream.service

# Step 6: Wait and verify
echo ""
echo "Step 6: Waiting for service to start..."
sleep 3

if systemctl is-active --quiet r2d2-camera-stream.service; then
    echo "✅ r2d2-camera-stream.service is RUNNING"
    
    # Check if HTTP server is responding
    sleep 2
    if curl -s http://localhost:8081/health 2>&1 | grep -q "OK"; then
        echo "✅ Camera stream HTTP server is responding"
    else
        echo "⚠️  Camera stream HTTP server not responding yet"
    fi
else
    echo "❌ r2d2-camera-stream.service failed to start"
    echo ""
    echo "Check logs:"
    echo "  sudo journalctl -u r2d2-camera-stream.service -n 30"
fi

echo ""
echo "=========================================="
echo "✅ Fix Complete!"
echo "=========================================="
echo ""
echo "Refresh your browser at: http://100.95.133.26:8080/"
echo "The camera stream service should now appear and be startable."
echo ""

