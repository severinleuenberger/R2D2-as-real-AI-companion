#!/bin/bash
# Install and start R2D2 Rosbridge systemd service

set -e

echo "Installing R2D2 Rosbridge service..."

# Copy service file
sudo cp /home/severin/dev/r2d2/r2d2-rosbridge.service /etc/systemd/system/

# Reload systemd
sudo systemctl daemon-reload

# Enable service (start on boot)
sudo systemctl enable r2d2-rosbridge.service

# Start service
sudo systemctl start r2d2-rosbridge.service

# Check status
echo ""
echo "Service status:"
sudo systemctl status r2d2-rosbridge.service --no-pager -l

echo ""
echo "Rosbridge service installed and started!"
echo "WebSocket available at: ws://$(hostname -I | awk '{print $1}'):9090"
echo ""
echo "Useful commands:"
echo "  Check status:  sudo systemctl status r2d2-rosbridge.service"
echo "  View logs:     sudo journalctl -u r2d2-rosbridge.service -f"
echo "  Restart:       sudo systemctl restart r2d2-rosbridge.service"
echo "  Stop:          sudo systemctl stop r2d2-rosbridge.service"
