#!/bin/bash
# Update camera stream service file in systemd

echo "Updating r2d2-camera-stream.service in systemd..."

cd /home/severin/dev/r2d2

# Copy updated service file
sudo cp r2d2-camera-stream.service /etc/systemd/system/
sudo chmod 644 /etc/systemd/system/r2d2-camera-stream.service

# Reload systemd
sudo systemctl daemon-reload

# Reset failed state
sudo systemctl reset-failed r2d2-camera-stream.service

echo "✅ Service file updated"
echo ""
echo "You can now try starting it:"
echo "  sudo systemctl start r2d2-camera-stream.service"
echo ""
echo "Or check status:"
echo "  sudo systemctl status r2d2-camera-stream.service"

