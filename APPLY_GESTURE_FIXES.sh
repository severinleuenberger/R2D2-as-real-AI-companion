#!/bin/bash
# Script to apply gesture system fixes
# Run this with: bash /home/severin/dev/r2d2/APPLY_GESTURE_FIXES.sh

set -e

echo "========================================"
echo "Applying Gesture System Fixes"
echo "========================================"
echo ""

# Reload systemd daemon
echo "1. Reloading systemd daemon..."
sudo systemctl daemon-reload
echo "✅ Daemon reloaded"
echo ""

# Restart camera-perception service
echo "2. Restarting r2d2-camera-perception.service..."
sudo systemctl restart r2d2-camera-perception.service
sleep 2
if systemctl is-active --quiet r2d2-camera-perception.service; then
    echo "✅ Camera perception service restarted"
else
    echo "❌ Warning: Camera perception service failed to start"
    exit 1
fi
echo ""

# Restart gesture-intent service
echo "3. Restarting r2d2-gesture-intent.service..."
sudo systemctl restart r2d2-gesture-intent.service
sleep 2
if systemctl is-active --quiet r2d2-gesture-intent.service; then
    echo "✅ Gesture intent service restarted"
else
    echo "❌ Warning: Gesture intent service failed to start"
    exit 1
fi
echo ""

echo "========================================"
echo "✅ All fixes applied successfully!"
echo "========================================"
echo ""
echo "Services are now running with:"
echo "  - target_person_name:=severin"
echo "  - Correct gesture model path"
echo "  - 35-second watchdog timeout"
echo "  - Audio beep file in place"
echo ""
echo "Next steps:"
echo "  1. Check logs: sudo journalctl -u r2d2-camera-perception.service -n 50"
echo "  2. Check gesture intent: sudo journalctl -u r2d2-gesture-intent.service -n 50"
echo "  3. Test gesture recognition workflow"

