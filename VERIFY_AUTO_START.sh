#!/bin/bash
################################################################################
# Verify R2D2 Auto-Start Configuration
# Run this script after reboot to verify all services auto-started correctly
################################################################################

echo "========================================================================"
echo "  R2D2 Auto-Start Verification (Post-Reboot)"
echo "========================================================================"
echo ""

echo "Waiting 5 seconds for services to stabilize..."
sleep 5
echo ""

echo "=== Service Status Check ==="
echo ""

services=("r2d2-camera-perception" "r2d2-audio-notification" "r2d2-gesture-intent" "r2d2-heartbeat")
all_running=true

for service in "${services[@]}"; do
    echo "--- $service.service ---"
    if sudo systemctl is-active --quiet "$service.service"; then
        echo "✓ Status: ACTIVE (RUNNING)"
    else
        echo "✗ Status: NOT RUNNING"
        all_running=false
    fi
    
    # Show brief status
    sudo systemctl status "$service.service" --no-pager --lines=3 | grep "Active:"
    echo ""
done

echo "========================================================================"
echo ""

if [ "$all_running" = true ]; then
    echo "✓ SUCCESS: All essential services are running!"
    echo ""
    echo "Next steps:"
    echo "  1. Test gesture recognition:"
    echo "     ros2 topic echo /r2d2/perception/gesture_event"
    echo ""
    echo "  2. Stand in front of camera and make gestures"
    echo ""
    echo "  3. Check logs:"
    echo "     sudo journalctl -u r2d2-gesture-intent.service -n 50"
else
    echo "✗ WARNING: Some services are not running!"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Check logs: sudo journalctl -u r2d2-gesture-intent.service -n 50"
    echo "  2. Try manual start: sudo systemctl start r2d2-gesture-intent.service"
    echo "  3. Check dependencies: sudo systemctl status r2d2-camera-perception.service"
fi

echo ""
echo "========================================================================"
echo ""
echo "ROS 2 Topics Check:"
echo "---"
ros2 topic list | grep r2d2 || echo "No ROS 2 topics found (ROS 2 may not be running)"
echo ""
echo "========================================================================"

