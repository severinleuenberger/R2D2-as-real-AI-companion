#!/bin/bash
#
# Installation script for Jetson Freeze Monitor
# Run this with: sudo bash install_freeze_monitor.sh
#

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
MONITOR_SCRIPT="${SCRIPT_DIR}/freeze_monitor.py"
SERVICE_FILE="${SCRIPT_DIR}/freeze-monitor.service"
LOGROTATE_FILE="${SCRIPT_DIR}/freeze-monitor.logrotate"
LOG_DIR="/var/log/freeze_logs"

echo "================================================"
echo "Jetson Freeze Monitor - Installation"
echo "================================================"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Error: This script must be run as root${NC}"
    echo "Please run: sudo bash install_freeze_monitor.sh"
    exit 1
fi

# Check if files exist
echo "Checking required files..."
if [ ! -f "$MONITOR_SCRIPT" ]; then
    echo -e "${RED}Error: freeze_monitor.py not found in $SCRIPT_DIR${NC}"
    exit 1
fi
if [ ! -f "$SERVICE_FILE" ]; then
    echo -e "${RED}Error: freeze-monitor.service not found in $SCRIPT_DIR${NC}"
    exit 1
fi
if [ ! -f "$LOGROTATE_FILE" ]; then
    echo -e "${RED}Error: freeze-monitor.logrotate not found in $SCRIPT_DIR${NC}"
    exit 1
fi
echo -e "${GREEN}✓ All required files found${NC}"
echo ""

# Make monitoring script executable
echo "Setting permissions on monitoring script..."
chmod +x "$MONITOR_SCRIPT"
echo -e "${GREEN}✓ Monitoring script is executable${NC}"
echo ""

# Create log directory
echo "Creating log directory..."
if [ ! -d "$LOG_DIR" ]; then
    mkdir -p "$LOG_DIR"
    chmod 755 "$LOG_DIR"
    echo -e "${GREEN}✓ Created $LOG_DIR${NC}"
else
    echo -e "${YELLOW}→ Log directory already exists${NC}"
fi
echo ""

# Install systemd service
echo "Installing systemd service..."
cp "$SERVICE_FILE" /etc/systemd/system/freeze-monitor.service
chmod 644 /etc/systemd/system/freeze-monitor.service
echo -e "${GREEN}✓ Service file installed to /etc/systemd/system/freeze-monitor.service${NC}"
echo ""

# Install logrotate configuration
echo "Installing logrotate configuration..."
cp "$LOGROTATE_FILE" /etc/logrotate.d/freeze-monitor
chmod 644 /etc/logrotate.d/freeze-monitor
echo -e "${GREEN}✓ Logrotate config installed to /etc/logrotate.d/freeze-monitor${NC}"
echo ""

# Reload systemd
echo "Reloading systemd daemon..."
systemctl daemon-reload
echo -e "${GREEN}✓ Systemd daemon reloaded${NC}"
echo ""

# Enable service
echo "Enabling freeze-monitor service..."
systemctl enable freeze-monitor.service
echo -e "${GREEN}✓ Service enabled (will start on boot)${NC}"
echo ""

# Start service
echo "Starting freeze-monitor service..."
systemctl start freeze-monitor.service
sleep 2  # Give it a moment to start
echo -e "${GREEN}✓ Service started${NC}"
echo ""

# Check service status
echo "Checking service status..."
if systemctl is-active --quiet freeze-monitor.service; then
    echo -e "${GREEN}✓ Service is running!${NC}"
    echo ""
    
    # Show recent logs
    echo "Recent service logs:"
    echo "-------------------"
    journalctl -u freeze-monitor.service -n 10 --no-pager
    echo ""
    
    # Check if log files are being written
    echo "Checking log files..."
    sleep 3  # Wait a bit for logs to be written
    
    if [ -f "$LOG_DIR/summary.log" ]; then
        echo -e "${GREEN}✓ Logs are being written${NC}"
        echo ""
        echo "Sample from summary.log:"
        echo "------------------------"
        tail -n 5 "$LOG_DIR/summary.log"
        echo ""
    else
        echo -e "${YELLOW}→ Waiting for first log entries...${NC}"
    fi
else
    echo -e "${RED}✗ Service failed to start${NC}"
    echo "Check status with: systemctl status freeze-monitor.service"
    exit 1
fi

# Show disk usage
echo "Current disk usage:"
echo "-------------------"
df -h "$LOG_DIR"
echo ""

# Final instructions
echo "================================================"
echo -e "${GREEN}Installation Complete!${NC}"
echo "================================================"
echo ""
echo "The freeze monitor is now running and will:"
echo "  • Start automatically on boot"
echo "  • Log every 10 seconds to $LOG_DIR"
echo "  • Rotate logs daily or at 100MB"
echo "  • Keep 14 days of logs"
echo ""
echo "Useful commands:"
echo "  View service status:    systemctl status freeze-monitor"
echo "  View service logs:      journalctl -u freeze-monitor -f"
echo "  View summary log:       tail -f $LOG_DIR/summary.log"
echo "  View all logs:          ls -lh $LOG_DIR/"
echo "  Stop service:           sudo systemctl stop freeze-monitor"
echo "  Restart service:        sudo systemctl restart freeze-monitor"
echo "  Disable service:        sudo systemctl disable freeze-monitor"
echo ""
echo "After a freeze, check the logs with:"
echo "  cat $LOG_DIR/summary.log | tail -100"
echo "  cat $LOG_DIR/hardware.log | tail -50"
echo "  cat $LOG_DIR/kernel.log | tail -50"
echo ""
echo -e "${YELLOW}Note: The system is using 93% of disk space.${NC}"
echo -e "${YELLOW}Monitor disk usage regularly to avoid running out of space.${NC}"
echo ""

