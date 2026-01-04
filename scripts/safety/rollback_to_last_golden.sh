#!/bin/bash
# NUCLEAR OPTION: Complete rollback to last verified golden tag
# Run with: ./rollback_to_last_golden.sh

set -e

cd /home/severin/dev/r2d2

LAST_GOLDEN="golden-vad-timeout-fix-2026-01-03"

echo "☢️  NUCLEAR ROLLBACK - Last Verified Golden Tag"
echo "==============================================="
echo ""
echo "⚠️  WARNING: This will DELETE all changes since Jan 3, 2026!"
echo ""
echo "Target: $LAST_GOLDEN"
git show --no-patch --oneline "$LAST_GOLDEN"
echo ""
echo "Current state:"
git log -1 --oneline
echo ""

read -p "Type 'ROLLBACK' to confirm complete system reset: " -r
if [ "$REPLY" != "ROLLBACK" ]; then
    echo "Cancelled."
    exit 0
fi

echo ""
echo "Performing HARD reset to $LAST_GOLDEN..."
git reset --hard "$LAST_GOLDEN"

echo ""
echo "Rebuilding ALL ROS packages..."
cd ~/dev/r2d2/ros2_ws
source /opt/ros/humble/setup.bash
colcon build

echo ""
echo "✅ Rollback complete!"
echo ""
echo "⚠️  CRITICAL: You must now manually:"
echo "1. sudo systemctl daemon-reload"
echo "2. Restart all services:"
echo "   for s in camera-perception audio-notification gesture-intent speech-node rest-speech-node; do"
echo "     sudo systemctl restart r2d2-\$s"
echo "   done"
echo "3. Verify: ~/dev/r2d2/tests/system/test_complete_system.sh"

