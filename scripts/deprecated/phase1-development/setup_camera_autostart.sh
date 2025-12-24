#!/bin/bash
# Setup script to enable camera auto-start after reboot

echo "Installing R2D2 Camera Perception Service..."

# Kill any running ROS2/camera processes
echo "Cleaning up background processes..."
pkill -9 ros2 python3 2>/dev/null
sleep 1

# Copy service file
echo "Installing service file..."
sudo cp ~/dev/r2d2/r2d2-camera-perception.service /etc/systemd/system/

# Reload systemd
echo "Reloading systemd..."
sudo systemctl daemon-reload

# Enable the service
echo "Enabling service for auto-start..."
sudo systemctl enable r2d2-camera-perception.service

# Verify
echo ""
echo "✅ Setup Complete!"
echo ""
echo "Service status:"
sudo systemctl status r2d2-camera-perception.service --no-pager || echo "Service installed, will start after reboot"
echo ""
echo "Ready to reboot! The camera will auto-start with the system."
