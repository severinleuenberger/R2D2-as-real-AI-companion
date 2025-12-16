#!/bin/bash
# R2D2 Person Recognition System - Post-Reboot Verification Script
# This script verifies that all components are running correctly after a system reboot

set -e

echo "=========================================="
echo "R2D2 Person Recognition System Verification"
echo "=========================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Track overall status
ALL_OK=true

# Function to check service status
check_service() {
    local service=$1
    local description=$2
    
    echo -n "Checking $description... "
    if systemctl is-active --quiet $service; then
        echo -e "${GREEN}✓ Running${NC}"
        return 0
    else
        echo -e "${RED}✗ Not running${NC}"
        echo "  Status: $(systemctl is-active $service 2>/dev/null || echo 'unknown')"
        ALL_OK=false
        return 1
    fi
}

# Function to check service enabled
check_enabled() {
    local service=$1
    local description=$2
    
    echo -n "Checking $description is enabled for autostart... "
    if systemctl is-enabled --quiet $service; then
        echo -e "${GREEN}✓ Enabled${NC}"
        return 0
    else
        echo -e "${RED}✗ Not enabled${NC}"
        ALL_OK=false
        return 1
    fi
}

# Function to check ROS2 topic exists
check_topic() {
    local topic=$1
    local description=$2
    
    echo -n "Checking $description topic... "
    if timeout 2s ros2 topic list | grep -q "^$topic$"; then
        echo -e "${GREEN}✓ Exists${NC}"
        return 0
    else
        echo -e "${RED}✗ Not found${NC}"
        ALL_OK=false
        return 1
    fi
}

# Function to check topic rate
check_topic_rate() {
    local topic=$1
    local min_rate=$2
    local description=$3
    
    echo -n "Checking $description rate (min ${min_rate} Hz)... "
    local rate=$(timeout 3s ros2 topic hz $topic 2>&1 | grep "average rate" | head -1 | awk '{print $3}' || echo "0")
    
    if (( $(echo "$rate >= $min_rate" | bc -l 2>/dev/null || echo 0) )); then
        echo -e "${GREEN}✓ ${rate} Hz${NC}"
        return 0
    else
        echo -e "${YELLOW}⚠ ${rate} Hz (may be starting up)${NC}"
        return 1
    fi
}

# Source ROS2 environment
echo "Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash
source ~/dev/r2d2/ros2_ws/install/setup.bash 2>/dev/null || {
    echo -e "${YELLOW}⚠ Warning: Could not source workspace setup.bash${NC}"
    echo "  This is normal if services haven't fully started yet"
}

echo ""
echo "=== Service Status Checks ==="
echo ""

# Check critical services
check_service "r2d2-camera-perception.service" "Camera & Perception Service"
check_service "r2d2-audio-notification.service" "Audio Notification Service"

echo ""
echo "=== Autostart Configuration ==="
echo ""

check_enabled "r2d2-camera-perception.service" "Camera & Perception Service"
check_enabled "r2d2-audio-notification.service" "Audio Notification Service"

echo ""
echo "=== ROS2 Topic Checks ==="
echo ""

# Check topics exist
check_topic "/r2d2/perception/person_id" "Person Recognition"
check_topic "/r2d2/audio/person_status" "Person Status"

echo ""
echo "=== Topic Rate Checks ==="
echo ""

# Check topic rates (with tolerance for startup)
check_topic_rate "/r2d2/perception/person_id" "3.0" "Person Recognition"
check_topic_rate "/r2d2/audio/person_status" "0.5" "Person Status"

echo ""
echo "=== Status Message Verification ==="
echo ""

echo -n "Checking status message format... "
if timeout 3s ros2 topic echo /r2d2/audio/person_status --once --no-arr 2>/dev/null | grep -q '"status"'; then
    echo -e "${GREEN}✓ Valid JSON${NC}"
    echo "  Sample message:"
    timeout 2s ros2 topic echo /r2d2/audio/person_status --once --no-arr 2>/dev/null | head -1 | sed 's/^/  /'
else
    echo -e "${RED}✗ Invalid or missing${NC}"
    ALL_OK=false
fi

echo ""
echo "=== Service Logs (Recent) ==="
echo ""

echo "Audio Notification Service (last 5 lines):"
journalctl -u r2d2-audio-notification.service -n 5 --no-pager | tail -3 | sed 's/^/  /'

echo ""
echo "=== Restart Configuration ==="
echo ""

echo -n "Checking restart-on-failure configuration... "
if systemctl show r2d2-audio-notification.service | grep -q "Restart=on-failure"; then
    echo -e "${GREEN}✓ Configured${NC}"
else
    echo -e "${RED}✗ Not configured${NC}"
    ALL_OK=false
fi

echo ""
echo "=========================================="
if [ "$ALL_OK" = true ]; then
    echo -e "${GREEN}✓ All checks passed! System is operational.${NC}"
    echo ""
    echo "Next steps:"
    echo "  1. Monitor status: ros2 topic echo /r2d2/audio/person_status --no-arr"
    echo "  2. View logs: journalctl -u r2d2-audio-notification.service -f"
    echo "  3. Test recognition: Stand in front of camera and wait for 'Hello!' audio"
    exit 0
else
    echo -e "${RED}✗ Some checks failed. Please review the output above.${NC}"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Check service status: sudo systemctl status r2d2-audio-notification.service"
    echo "  2. Check service logs: journalctl -u r2d2-audio-notification.service -n 50"
    echo "  3. Restart service: sudo systemctl restart r2d2-audio-notification.service"
    exit 1
fi

