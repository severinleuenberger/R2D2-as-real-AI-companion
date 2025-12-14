#!/bin/bash
# Install and start R2D2 Web Dashboard service

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_FILE="$SCRIPT_DIR/r2d2-web-dashboard.service"

echo "Installing R2D2 Web Dashboard service..."

# Make startup script executable
chmod +x "$SCRIPT_DIR/web_dashboard/scripts/start_web_dashboard.sh"

# Copy service file
sudo cp "$SERVICE_FILE" /etc/systemd/system/

# Reload systemd
sudo systemctl daemon-reload

# Enable for autostart
sudo systemctl enable r2d2-web-dashboard.service

# Start service
sudo systemctl start r2d2-web-dashboard.service

# Show status
echo ""
echo "Service status:"
sudo systemctl status r2d2-web-dashboard.service --no-pager -l

echo ""
echo "✅ Web Dashboard service installed and started!"
echo "   Access at: http://100.95.133.26:8080"
echo "   Service will auto-start on reboot"

