#!/bin/bash
# Quick script to start and enable the audio notification service

echo "Starting R2D2 Audio Notification Service..."
echo ""

# Reload systemd to pick up any changes
sudo systemctl daemon-reload

# Enable service to start on boot (always-on background service)
echo "Enabling service to start on boot..."
sudo systemctl enable r2d2-audio-notification.service

# Start the service
echo "Starting service..."
sudo systemctl start r2d2-audio-notification.service

# Wait a moment for it to start
sleep 2

# Check status
echo ""
echo "Service status:"
sudo systemctl status r2d2-audio-notification.service --no-pager -l

echo ""
echo "✓ Service should now be running in the background!"
echo "  - To view logs: sudo journalctl -u r2d2-audio-notification.service -f"
echo "  - To restart: sudo systemctl restart r2d2-audio-notification.service"
echo "  - To stop: sudo systemctl stop r2d2-audio-notification.service"

