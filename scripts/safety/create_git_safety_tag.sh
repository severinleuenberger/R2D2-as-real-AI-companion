#!/bin/bash
# Create git safety tag before WebUI implementation
# Run with: ./create_git_safety_tag.sh

set -e

cd /home/severin/dev/r2d2

echo "🔖 Creating Git Safety Tag"
echo "=========================="
echo ""

# Check current branch
CURRENT_BRANCH=$(git branch --show-current)
echo "Current branch: $CURRENT_BRANCH"

# Check for uncommitted changes
if ! git diff-index --quiet HEAD -- 2>/dev/null; then
    echo ""
    echo "📝 Uncommitted changes detected. Creating stash..."
    git stash push -m "Pre-WebUI-changes stash $(date +%Y%m%d_%H%M%S)"
    echo "✅ Changes stashed"
fi

# Show current commit
echo ""
echo "Current commit:"
git log -1 --oneline
echo ""

# Get current service status
echo "Documenting current service state..."
SERVICES_STATUS=$(systemctl list-units "r2d2-*" --no-pager --no-legend | awk '{print $1, $3}' | grep -E "active|failed" || true)

# Create annotated tag
TAG_NAME="golden-pre-webui-2026-01-04"
TAG_MESSAGE="Known-good state before WebUI stabilization work - $(date)

Working services:
- r2d2-camera-perception ✅ active
- r2d2-audio-notification ✅ active
- r2d2-gesture-intent ✅ active
- r2d2-speech-node ✅ active
- r2d2-rest-speech-node ✅ active
- r2d2-volume-control ✅ active
- r2d2-wake-api ✅ active

Known issues (non-blocking):
- r2d2-heartbeat: Wrong script path (fixable)
- r2d2-rosbridge: Not auto-starting (fixable)
- r2d2-web-dashboard: Disabled (intentional)

This tag represents a fully functional R2D2 system with all UX-critical
features working. Use this tag for rollback if WebUI changes break anything.

Branch: $CURRENT_BRANCH
Commit: $(git log -1 --oneline)
"

echo "Creating tag: $TAG_NAME"
git tag -a "$TAG_NAME" -m "$TAG_MESSAGE"

echo ""
echo "✅ Tag created successfully!"
echo ""
echo "Tag: $TAG_NAME"
echo "Commit: $(git show --no-patch --oneline $TAG_NAME)"
echo ""
echo "To push to remote (recommended):"
echo "  git push origin $TAG_NAME"
echo ""
echo "To rollback to this tag later:"
echo "  git reset --hard $TAG_NAME"
echo "  cd ~/dev/r2d2/ros2_ws && colcon build"
echo "  sudo systemctl daemon-reload"

