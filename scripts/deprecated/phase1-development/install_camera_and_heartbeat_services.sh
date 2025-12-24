#!/bin/bash
# Install camera stream and heartbeat services

echo "Installing r2d2-camera-stream.service..."
sudo cp /home/severin/dev/r2d2/r2d2-camera-stream.service /etc/systemd/system/
sudo chmod 644 /etc/systemd/system/r2d2-camera-stream.service

echo "Installing r2d2-heartbeat.service..."
sudo cp /home/severin/dev/r2d2/r2d2-heartbeat.service /etc/systemd/system/
sudo chmod 644 /etc/systemd/system/r2d2-heartbeat.service

echo "Reloading systemd daemon..."
sudo systemctl daemon-reload

echo ""
echo "✅ Services installed successfully!"
echo ""
echo "To start the services:"
echo "  sudo systemctl start r2d2-camera-stream.service"
echo "  sudo systemctl start r2d2-heartbeat.service"
echo ""
echo "To enable auto-start on boot:"
echo "  sudo systemctl enable r2d2-camera-stream.service"
echo "  sudo systemctl enable r2d2-heartbeat.service"
echo ""
echo "To check status:"
echo "  sudo systemctl status r2d2-camera-stream.service"
echo "  sudo systemctl status r2d2-heartbeat.service"

