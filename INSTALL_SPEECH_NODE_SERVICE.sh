#!/bin/bash
# Install R2D2 Speech Node as a systemd service

set -e

echo "========================================"
echo "Installing R2D2 Speech Node Service"
echo "========================================"
echo ""

# Make startup script executable
echo "1. Making startup script executable..."
chmod +x /home/severin/dev/r2d2/start_speech_node.sh
echo "   ✅ Done"
echo ""

# Copy service file to systemd
echo "2. Installing service file..."
sudo cp /home/severin/dev/r2d2/r2d2-speech-node.service /etc/systemd/system/
echo "   ✅ Done"
echo ""

# Reload systemd daemon
echo "3. Reloading systemd daemon..."
sudo systemctl daemon-reload
echo "   ✅ Done"
echo ""

# Enable service (auto-start on boot)
echo "4. Enabling service for auto-start..."
sudo systemctl enable r2d2-speech-node.service
echo "   ✅ Done"
echo ""

# Start service now
echo "5. Starting service..."
sudo systemctl start r2d2-speech-node.service
sleep 3
echo "   ✅ Done"
echo ""

# Check status
echo "6. Checking service status..."
if systemctl is-active --quiet r2d2-speech-node.service; then
    echo "   ✅ Service is RUNNING"
else
    echo "   ❌ Service FAILED to start"
    echo ""
    echo "Checking logs..."
    sudo journalctl -u r2d2-speech-node.service -n 20 --no-pager
    exit 1
fi
echo ""

# Verify ROS2 services are available
echo "7. Verifying ROS2 services..."
cd /home/severin/dev/r2d2/ros2_ws
source install/setup.bash
sleep 2

if ros2 service list | grep -q "/r2d2/speech/start_session"; then
    echo "   ✅ /r2d2/speech/start_session service available"
else
    echo "   ⚠️  Service not yet available (may need a few more seconds)"
fi

if ros2 service list | grep -q "/r2d2/speech/stop_session"; then
    echo "   ✅ /r2d2/speech/stop_session service available"
else
    echo "   ⚠️  Service not yet available (may need a few more seconds)"
fi
echo ""

echo "========================================"
echo "✅ Installation Complete!"
echo "========================================"
echo ""
echo "Service Status:"
echo "  - Auto-start: ENABLED"
echo "  - Current status: RUNNING"
echo ""
echo "Commands:"
echo "  Check status:  systemctl status r2d2-speech-node"
echo "  View logs:     sudo journalctl -u r2d2-speech-node -f"
echo "  Stop service:  sudo systemctl stop r2d2-speech-node"
echo "  Start service: sudo systemctl start r2d2-speech-node"
echo ""
echo "========================================"
echo "🎉 Ready to test gestures!"
echo "========================================"
echo ""
echo "Now try making gestures:"
echo "  ☝️  Index finger up → Should hear START beep + speech activates"
echo "  ✊ Closed fist → Should hear STOP beep + speech stops"

