#!/bin/bash
################################################################################
# Install Gesture System Auto-Start Services
# Run this script to configure the R2D2 gesture system for auto-start on boot
################################################################################

set -e  # Exit on error

echo "========================================================================"
echo "  Installing R2D2 Gesture System Auto-Start Services"
echo "========================================================================"
echo ""

# Check we're in the right directory
if [ ! -f "r2d2-gesture-intent.service" ]; then
    echo "ERROR: Please run this script from /home/severin/dev/r2d2/"
    exit 1
fi

echo "Step 1: Copy service files to systemd directory..."
sudo cp r2d2-gesture-intent.service /etc/systemd/system/
sudo cp r2d2-camera-perception.service /etc/systemd/system/
echo "✓ Service files copied"
echo ""

echo "Step 2: Reload systemd daemon..."
sudo systemctl daemon-reload
echo "✓ Systemd reloaded"
echo ""

echo "Step 3: Enable services for auto-start on boot..."
sudo systemctl enable r2d2-camera-perception.service
sudo systemctl enable r2d2-gesture-intent.service
echo "✓ Services enabled"
echo ""

echo "Step 4: Start services..."
sudo systemctl start r2d2-camera-perception.service
sleep 3
sudo systemctl start r2d2-gesture-intent.service
echo "✓ Services started"
echo ""

echo "Step 5: Check service status..."
echo ""
echo "--- Camera-Perception Service ---"
sudo systemctl status r2d2-camera-perception.service --no-pager | head -15
echo ""
echo "--- Gesture-Intent Service ---"
sudo systemctl status r2d2-gesture-intent.service --no-pager | head -15
echo ""

echo "========================================================================"
echo "  Installation Complete!"
echo "========================================================================"
echo ""
echo "Services are now configured to auto-start on boot."
echo ""
echo "Next steps:"
echo "  1. Verify both services show 'active (running)'"
echo "  2. Test: sudo journalctl -u r2d2-gesture-intent.service -f"
echo "  3. Reboot and verify auto-start: sudo reboot"
echo ""
echo "========================================================================"

