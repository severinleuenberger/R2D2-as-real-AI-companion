#!/bin/bash
# Rollback to pre-WebUI safety tag
# Run with: ./rollback_to_safety_tag.sh [--soft|--hard]

set -e

cd /home/severin/dev/r2d2

RESET_TYPE="${1:---soft}"
TAG="golden-pre-webui-2026-01-04"

echo "🔄 Git Rollback to Safety Tag"
echo "=============================="
echo ""
echo "Tag: $TAG"
echo "Reset type: $RESET_TYPE"
echo ""

# Verify tag exists
if ! git rev-parse "$TAG" >/dev/null 2>&1; then
    echo "❌ Tag '$TAG' not found!"
    echo ""
    echo "Available golden tags:"
    git tag | grep golden
    exit 1
fi

# Show what will change
echo "Current commit:"
git log -1 --oneline
echo ""
echo "Target commit:"
git show --no-patch --oneline "$TAG"
echo ""

# Confirm if hard reset
if [ "$RESET_TYPE" = "--hard" ]; then
    echo "⚠️  WARNING: Hard reset will DELETE all changes since tag!"
    read -p "Are you sure? Type 'yes' to confirm: " -r
    if [ "$REPLY" != "yes" ]; then
        echo "Cancelled."
        exit 0
    fi
fi

# Perform reset
echo ""
echo "Performing git reset $RESET_TYPE to $TAG..."
git reset "$RESET_TYPE" "$TAG"

echo ""
echo "✅ Git reset complete!"
echo ""

# Rebuild ROS packages if hard reset
if [ "$RESET_TYPE" = "--hard" ]; then
    echo "Rebuilding ROS packages..."
    cd ~/dev/r2d2/ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build --packages-select r2d2_speech r2d2_audio r2d2_gesture
    echo "✅ ROS packages rebuilt"
    echo ""
fi

echo "Next steps:"
echo "1. Reload systemd: sudo systemctl daemon-reload"
echo "2. Restart services: sudo systemctl restart r2d2-camera-perception r2d2-audio-notification"
echo "3. Verify health: ~/dev/r2d2/tests/system/test_complete_system.sh"

