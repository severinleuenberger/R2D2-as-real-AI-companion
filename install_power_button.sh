#!/bin/bash
################################################################################
# R2D2 Power Button Handler - Installation Script
#
# This script installs the r2d2_power_button.py handler as a systemd service
# that auto-starts on boot and restarts on failure.
#
# Usage: sudo bash install_power_button.sh
################################################################################

set -e  # Exit on any error

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_SCRIPT="${SCRIPT_DIR}/r2d2_power_button.py"
SERVICE_FILE="${SCRIPT_DIR}/r2d2-powerbutton.service"
INSTALL_BIN="/usr/local/bin/r2d2_power_button.py"
INSTALL_SERVICE="/etc/systemd/system/r2d2-powerbutton.service"
LOG_FILE="/var/log/r2d2_power_button.log"

################################################################################
# CHECKS
################################################################################

echo -e "${BLUE}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║         R2D2 Power Button Handler - Installation              ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Check if running as root
if [[ $EUID -ne 0 ]]; then
    echo -e "${RED}✗ This script must be run as root (use: sudo bash install_power_button.sh)${NC}"
    exit 1
fi

# Check if Python script exists
if [[ ! -f "$PYTHON_SCRIPT" ]]; then
    echo -e "${RED}✗ Python script not found: $PYTHON_SCRIPT${NC}"
    exit 1
fi

# Check if service file exists
if [[ ! -f "$SERVICE_FILE" ]]; then
    echo -e "${RED}✗ Service file not found: $SERVICE_FILE${NC}"
    exit 1
fi

# Check if Jetson.GPIO is installed
echo -e "${BLUE}Checking dependencies...${NC}"
if ! python3 -c "import Jetson.GPIO" 2>/dev/null; then
    echo -e "${RED}✗ Jetson.GPIO is not installed${NC}"
    echo -e "${YELLOW}  Installing: sudo apt install python3-jetson-gpio${NC}"
    apt install -y python3-jetson-gpio
    if python3 -c "import Jetson.GPIO" 2>/dev/null; then
        echo -e "${GREEN}✓ Jetson.GPIO installed successfully${NC}"
    else
        echo -e "${RED}✗ Failed to install Jetson.GPIO${NC}"
        exit 1
    fi
else
    echo -e "${GREEN}✓ Jetson.GPIO is available${NC}"
fi

################################################################################
# INSTALLATION
################################################################################

echo ""
echo -e "${BLUE}Installing Power Button Handler...${NC}"

# 1. Install Python script to /usr/local/bin
echo -n "  Installing Python script to $INSTALL_BIN... "
cp "$PYTHON_SCRIPT" "$INSTALL_BIN"
chmod 755 "$INSTALL_BIN"
echo -e "${GREEN}✓${NC}"

# 2. Create log file with proper permissions
echo -n "  Creating log file $LOG_FILE... "
touch "$LOG_FILE"
chmod 666 "$LOG_FILE"
echo -e "${GREEN}✓${NC}"

# 3. Install systemd service file
echo -n "  Installing systemd service to $INSTALL_SERVICE... "
cp "$SERVICE_FILE" "$INSTALL_SERVICE"
chmod 644 "$INSTALL_SERVICE"
echo -e "${GREEN}✓${NC}"

# 4. Reload systemd daemon
echo -n "  Reloading systemd configuration... "
systemctl daemon-reload
echo -e "${GREEN}✓${NC}"

# 5. Enable service for auto-start
echo -n "  Enabling service for auto-start... "
systemctl enable r2d2-powerbutton.service
echo -e "${GREEN}✓${NC}"

# 6. Start the service
echo -n "  Starting service... "
systemctl start r2d2-powerbutton.service
sleep 1
echo -e "${GREEN}✓${NC}"

################################################################################
# VERIFICATION
################################################################################

echo ""
echo -e "${BLUE}Verifying installation...${NC}"

# Check if service is running
if systemctl is-active --quiet r2d2-powerbutton.service; then
    echo -e "${GREEN}✓ Service is running${NC}"
else
    echo -e "${RED}✗ Service is NOT running${NC}"
    echo -e "${YELLOW}  Checking status:${NC}"
    systemctl status r2d2-powerbutton.service || true
fi

# Check if service is enabled
if systemctl is-enabled --quiet r2d2-powerbutton.service; then
    echo -e "${GREEN}✓ Service is enabled for auto-start${NC}"
else
    echo -e "${RED}✗ Service is NOT enabled${NC}"
fi

# Show recent logs
echo ""
echo -e "${BLUE}Recent service logs:${NC}"
journalctl -u r2d2-powerbutton.service -n 10 --no-pager || echo "  (No logs yet)"

################################################################################
# SUMMARY
################################################################################

echo ""
echo -e "${GREEN}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║                  Installation Complete! ✓                      ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${BLUE}Installation Summary:${NC}"
echo "  • Python script: $INSTALL_BIN"
echo "  • Systemd service: $INSTALL_SERVICE"
echo "  • Log file: $LOG_FILE"
echo "  • Status: Running and enabled for auto-start"
echo ""
echo -e "${BLUE}Useful Commands:${NC}"
echo "  • Check status:    sudo systemctl status r2d2-powerbutton.service"
echo "  • View logs:       sudo journalctl -u r2d2-powerbutton.service -f"
echo "  • Restart service: sudo systemctl restart r2d2-powerbutton.service"
echo "  • Stop service:    sudo systemctl stop r2d2-powerbutton.service"
echo "  • Uninstall:       sudo systemctl disable r2d2-powerbutton.service && sudo rm $INSTALL_BIN $INSTALL_SERVICE && sudo systemctl daemon-reload"
echo ""
echo -e "${YELLOW}IMPORTANT: The service is now running and monitoring GPIO17 (Pin 22).${NC}"
echo -e "${YELLOW}Do NOT press the button until you have verified the setup!${NC}"
echo ""
