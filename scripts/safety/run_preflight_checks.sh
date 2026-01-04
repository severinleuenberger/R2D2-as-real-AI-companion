#!/bin/bash
# Pre-flight safety checklist before WebUI implementation
# Verifies system health and creates all safety backups

set +e  # Don't exit on error - we want to see all test results

cd /home/severin/dev/r2d2

echo "╔════════════════════════════════════════════════════════════╗"
echo "║     R2D2 Pre-Flight Safety Checklist                       ║"
echo "║     Before WebUI Implementation                            ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

CHECKS_PASSED=0
CHECKS_FAILED=0

print_check() {
    echo -n "[$1] $2... "
}

print_pass() {
    echo -e "${GREEN}✅ PASS${NC}"
    ((CHECKS_PASSED++))
}

print_fail() {
    echo -e "${RED}❌ FAIL${NC}"
    echo "      $1"
    ((CHECKS_FAILED++))
}

print_skip() {
    echo -e "${YELLOW}⏭  SKIP${NC}"
    echo "      $1"
}

# =============================================================================
# CHECK 1: Verify Core Services Healthy
# =============================================================================
echo "═══════════════════════════════════════════════════════════"
echo " Check 1: Core Services Status"
echo "═══════════════════════════════════════════════════════════"

CORE_SERVICES=(
    "r2d2-camera-perception"
    "r2d2-audio-notification"
    "r2d2-gesture-intent"
    "r2d2-speech-node"
)

for service in "${CORE_SERVICES[@]}"; do
    print_check "1" "$service"
    if systemctl is-active --quiet "$service"; then
        print_pass
    else
        print_fail "Service not active - core functionality may be broken!"
        echo ""
        echo "⚠️  WARNING: Core service not running. Fix before proceeding!"
        echo "   Check: sudo systemctl status $service"
        exit 1
    fi
done

# =============================================================================
# CHECK 2: Verify ROS Topics Publishing
# =============================================================================
echo ""
echo "═══════════════════════════════════════════════════════════"
echo " Check 2: ROS Topics Publishing"
echo "═══════════════════════════════════════════════════════════"

source /opt/ros/humble/setup.bash 2>/dev/null
source ~/dev/r2d2/ros2_ws/install/setup.bash 2>/dev/null

print_check "2" "Camera topic (/oak/rgb/image_raw)"
if timeout 3 ros2 topic hz /oak/rgb/image_raw 2>/dev/null | grep -q "average rate"; then
    print_pass
else
    print_fail "Camera not publishing - check camera service"
fi

print_check "2" "Person status topic"
if timeout 3 ros2 topic echo /r2d2/audio/person_status --once 2>/dev/null | grep -q "status"; then
    print_pass
else
    print_fail "Status not publishing - check audio service"
fi

# =============================================================================
# CHECK 3: Test UX Features (Quick)
# =============================================================================
echo ""
echo "═══════════════════════════════════════════════════════════"
echo " Check 3: UX Feature Test"
echo "═══════════════════════════════════════════════════════════"

print_check "3" "Person recognition active"
PERSON_ID=$(timeout 2 ros2 topic echo /r2d2/perception/person_id --once 2>/dev/null | grep "data:" | awk '{print $2}' | tr -d "'\"" || echo "")
if [ -n "$PERSON_ID" ]; then
    print_pass
    echo "      Recognized: $PERSON_ID"
else
    print_skip "No person in front of camera (this is OK)"
fi

# =============================================================================
# CHECK 4: Create Service Backup
# =============================================================================
echo ""
echo "═══════════════════════════════════════════════════════════"
echo " Check 4: Create Service Backup"
echo "═══════════════════════════════════════════════════════════"

print_check "4" "Creating service file backup"
BACKUP_SCRIPT="~/dev/r2d2/scripts/safety/create_service_backup.sh"

if [ ! -f "$BACKUP_SCRIPT" ]; then
    print_fail "Backup script not found: $BACKUP_SCRIPT"
else
    echo ""
    echo "      You need to run this with sudo:"
    echo "      sudo $BACKUP_SCRIPT"
    print_skip "Manual step required (sudo)"
fi

# =============================================================================
# CHECK 5: Create Git Safety Tag
# =============================================================================
echo ""
echo "═══════════════════════════════════════════════════════════"
echo " Check 5: Create Git Safety Tag"
echo "═══════════════════════════════════════════════════════════"

print_check "5" "Git safety tag"

TAG_SCRIPT="~/dev/r2d2/scripts/safety/create_git_safety_tag.sh"
if [ ! -f "$TAG_SCRIPT" ]; then
    print_fail "Tag script not found: $TAG_SCRIPT"
else
    if git rev-parse golden-pre-webui-2026-01-04 >/dev/null 2>&1; then
        print_pass
        echo "      Tag already exists: golden-pre-webui-2026-01-04"
    else
        echo ""
        echo "      Run this to create tag:"
        echo "      $TAG_SCRIPT"
        print_skip "Manual step required"
    fi
fi

# =============================================================================
# Summary
# =============================================================================
echo ""
echo "═══════════════════════════════════════════════════════════"
echo " Pre-Flight Summary"
echo "═══════════════════════════════════════════════════════════"
echo -e "  ${GREEN}Passed:${NC} $CHECKS_PASSED"
echo -e "  ${RED}Failed:${NC} $CHECKS_FAILED"
echo ""

if [ $CHECKS_FAILED -gt 0 ]; then
    echo -e "${RED}⚠️  STOP: Fix failed checks before proceeding!${NC}"
    exit 1
fi

echo "📋 Manual Steps Required:"
echo ""
echo "1. Create service backup:"
echo "   sudo ~/dev/r2d2/scripts/safety/create_service_backup.sh"
echo ""
echo "2. Create git safety tag:"
echo "   ~/dev/r2d2/scripts/safety/create_git_safety_tag.sh"
echo ""
echo "3. Optionally push tag to remote:"
echo "   cd ~/dev/r2d2 && git push origin golden-pre-webui-2026-01-04"
echo ""
echo -e "${GREEN}Once these steps are complete, you can proceed with WebUI implementation!${NC}"
echo ""
echo "Quick reference card saved to: ~/EMERGENCY_ROLLBACK_CARD.md"

