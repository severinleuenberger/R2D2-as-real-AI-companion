#!/bin/bash
# Quick fix for camera stream service

echo "Updating camera stream service..."
cd /home/severin/dev/r2d2

# Copy updated service file
sudo cp r2d2-camera-stream.service /etc/systemd/system/
sudo chmod 644 /etc/systemd/system/r2d2-camera-stream.service

# Reload systemd
sudo systemctl daemon-reload

# Reset failed state
sudo systemctl reset-failed r2d2-camera-stream.service

# Try to start (will fail if camera service not running, but that's OK)
sudo systemctl start r2d2-camera-stream.service || echo "Service start failed - camera-perception may not be running"

# Check status
echo ""
echo "Service status:"
systemctl is-active r2d2-camera-stream.service || systemctl status r2d2-camera-stream.service --no-pager -l | head -15

echo ""
echo "✅ Service file updated. If camera-perception service is running, camera-stream should start automatically."

