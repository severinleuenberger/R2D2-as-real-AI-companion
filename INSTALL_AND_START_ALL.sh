#!/bin/bash
# Complete installation and startup script for R2D2 services
# Run this ONCE with: bash INSTALL_AND_START_ALL.sh
# You'll be prompted for your sudo password

set -e

echo "=========================================="
echo "R2D2 Services - Complete Installation"
echo "=========================================="
echo ""

cd /home/severin/dev/r2d2

# Step 1: Install services
echo "Step 1: Installing services to systemd..."
sudo cp r2d2-camera-stream.service /etc/systemd/system/
sudo cp r2d2-heartbeat.service /etc/systemd/system/
sudo chmod 644 /etc/systemd/system/r2d2-camera-stream.service
sudo chmod 644 /etc/systemd/system/r2d2-heartbeat.service
sudo systemctl daemon-reload
echo "✅ Services installed"

# Step 2: Start services
echo ""
echo "Step 2: Starting services..."
sudo systemctl start r2d2-heartbeat.service
sudo systemctl start r2d2-camera-stream.service
echo "✅ Services started"

# Step 3: Enable auto-start
echo ""
echo "Step 3: Enabling auto-start on boot..."
sudo systemctl enable r2d2-heartbeat.service
sudo systemctl enable r2d2-camera-stream.service
echo "✅ Auto-start enabled"

# Step 4: Wait for initialization
echo ""
echo "Step 4: Waiting for services to initialize..."
sleep 5

# Step 5: Verify
echo ""
echo "Step 5: Verifying services..."
echo ""

if systemctl is-active --quiet r2d2-heartbeat.service; then
    echo "✅ r2d2-heartbeat.service is RUNNING"
else
    echo "❌ r2d2-heartbeat.service failed to start"
    echo "   Check logs: sudo journalctl -u r2d2-heartbeat.service -n 20"
fi

if systemctl is-active --quiet r2d2-camera-stream.service; then
    echo "✅ r2d2-camera-stream.service is RUNNING"
else
    echo "❌ r2d2-camera-stream.service failed to start"
    echo "   Check logs: sudo journalctl -u r2d2-camera-stream.service -n 20"
fi

# Check heartbeat topic
echo ""
echo "Checking heartbeat topic..."
if timeout 3 ros2 topic echo /r2d2/heartbeat --once 2>&1 | grep -q "timestamp"; then
    echo "✅ /r2d2/heartbeat is publishing"
else
    echo "⚠️  /r2d2/heartbeat not publishing yet (may need a few more seconds)"
fi

# Check camera stream
echo ""
echo "Checking camera stream..."
sleep 2
if curl -s http://localhost:8081/health 2>&1 | grep -q "OK"; then
    echo "✅ Camera stream HTTP server is responding"
else
    echo "⚠️  Camera stream not responding (may need camera service running)"
fi

echo ""
echo "=========================================="
echo "✅ Installation Complete!"
echo "=========================================="
echo ""
echo "Refresh your browser at: http://100.95.133.26:8080/"
echo ""
echo "You should now see:"
echo "  - System Health metrics (CPU, GPU, Temperature)"
echo "  - Camera stream toggle button"
echo "  - Real-time status updates"
echo ""

