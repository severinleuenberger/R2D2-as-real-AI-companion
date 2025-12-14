#!/bin/bash
# Verification script for R2D2 Web Dashboard integration
# Checks entire dependency chain: services, rosbridge, topics, API, UI

# set -e  # Disabled: verification script should continue on failures

echo "=========================================="
echo "R2D2 Web Dashboard Integration Verification"
echo "=========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

PASSED=0
FAILED=0

# Function to check and report
check_item() {
    local name="$1"
    local command="$2"
    local expected="$3"
    
    echo -n "Checking $name... "
    if eval "$command" > /dev/null 2>&1; then
        echo -e "${GREEN}✓ PASSED${NC}"
        PASSED=$((PASSED + 1))
        return 0
    else
        echo -e "${RED}✗ FAILED${NC}"
        if [ -n "$expected" ]; then
            echo "  Expected: $expected"
        fi
        FAILED=$((FAILED + 1))
        return 1
    fi
}

# 1. Check systemd services exist
echo "=== 1. Systemd Service Files ==="
check_item "r2d2-camera-perception.service exists" \
    "test -f /etc/systemd/system/r2d2-camera-perception.service" \
    "Service file should exist in /etc/systemd/system/"

check_item "r2d2-audio-notification.service exists" \
    "test -f /etc/systemd/system/r2d2-audio-notification.service" \
    "Service file should exist in /etc/systemd/system/"

check_item "r2d2-camera-stream.service exists" \
    "test -f /etc/systemd/system/r2d2-camera-stream.service" \
    "Service file should exist in /etc/systemd/system/"

echo ""

# 2. Check service status
echo "=== 2. Service Status ==="
check_item "r2d2-camera-perception.service status" \
    "systemctl is-active --quiet r2d2-camera-perception.service || systemctl is-failed --quiet r2d2-camera-perception.service" \
    "Service should be active or failed (not inactive)"

check_item "r2d2-audio-notification.service status" \
    "systemctl is-active --quiet r2d2-audio-notification.service || systemctl is-failed --quiet r2d2-audio-notification.service" \
    "Service should be active or failed (not inactive)"

check_item "r2d2-camera-stream.service status" \
    "systemctl is-active --quiet r2d2-camera-stream.service || systemctl is-failed --quiet r2d2-camera-stream.service" \
    "Service should be active or failed (not inactive)"

echo ""

# 3. Check rosbridge
echo "=== 3. rosbridge Status ==="
ROSBRIDGE_RUNNING=false
if pgrep -f rosbridge_websocket > /dev/null; then
    echo -e "rosbridge process: ${GREEN}✓ RUNNING${NC}"
    ROSBRIDGE_RUNNING=true
    PASSED=$((PASSED + 1))
else
    echo -e "rosbridge process: ${YELLOW}⚠ NOT RUNNING${NC}"
    echo "  Note: rosbridge must be started manually: cd ~/dev/r2d2/web_dashboard && ./start_rosbridge.sh"
    FAILED=$((FAILED + 1))
fi

# Check port 9090
if nc -z localhost 9090 2>/dev/null || timeout 1 bash -c "echo > /dev/tcp/localhost/9090" 2>/dev/null; then
    echo -e "rosbridge port 9090: ${GREEN}✓ ACCESSIBLE${NC}"
    PASSED=$((PASSED + 1))
else
    echo -e "rosbridge port 9090: ${RED}✗ NOT ACCESSIBLE${NC}"
    FAILED=$((FAILED + 1))
fi

echo ""

# 4. Check ROS topics (if ROS 2 is available)
echo "=== 4. ROS 2 Topics ==="
if command -v ros2 > /dev/null 2>&1; then
    # Source ROS 2 environment
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    fi
    if [ -f ~/dev/r2d2/ros2_ws/install/setup.bash ]; then
        source ~/dev/r2d2/ros2_ws/install/setup.bash
    fi
    
    # Check topics
    check_item "/r2d2/audio/person_status topic" \
        "timeout 2 ros2 topic list 2>/dev/null | grep -q '/r2d2/audio/person_status'" \
        "Topic should exist when audio-notification service is running"
    
    check_item "/r2d2/perception/face_count topic" \
        "timeout 2 ros2 topic list 2>/dev/null | grep -q '/r2d2/perception/face_count'" \
        "Topic should exist when camera-perception service is running"
    
    check_item "/oak/rgb/image_raw topic" \
        "timeout 2 ros2 topic list 2>/dev/null | grep -q '/oak/rgb/image_raw'" \
        "Topic should exist when camera-perception service is running"
else
    echo -e "${YELLOW}⚠ ROS 2 not found in PATH, skipping topic checks${NC}"
fi

echo ""

# 5. Check Web Dashboard API
echo "=== 5. Web Dashboard API ==="
check_item "Web dashboard health endpoint" \
    "curl -s -f http://localhost:8080/health > /dev/null" \
    "API should respond at http://localhost:8080/health"

check_item "Services status API" \
    "curl -s -f http://localhost:8080/api/services/status > /dev/null" \
    "API should respond at http://localhost:8080/api/services/status"

check_item "rosbridge status API" \
    "curl -s -f http://localhost:8080/api/status/rosbridge > /dev/null" \
    "API should respond at http://localhost:8080/api/status/rosbridge"

echo ""

# 6. Check UI connectivity (if rosbridge is running)
echo "=== 6. UI Connectivity ==="
if [ "$ROSBRIDGE_RUNNING" = true ]; then
    # Try to connect to WebSocket
    if command -v wscat > /dev/null 2>&1 || command -v websocat > /dev/null 2>&1; then
        echo -e "WebSocket tools available: ${GREEN}✓${NC}"
        PASSED=$((PASSED + 1))
    else
        echo -e "WebSocket tools: ${YELLOW}⚠ Not available (optional)${NC}"
        echo "  Install wscat or websocat for WebSocket testing"
    fi
else
    echo -e "${YELLOW}⚠ rosbridge not running, skipping WebSocket connectivity test${NC}"
    echo "  Start rosbridge to enable UI real-time updates"
fi

echo ""

# Summary
echo "=========================================="
echo "Verification Summary"
echo "=========================================="
echo -e "Passed: ${GREEN}$PASSED${NC}"
echo -e "Failed: ${RED}$FAILED${NC}"
echo ""

if [ $FAILED -eq 0 ]; then
    echo -e "${GREEN}✓ All checks passed!${NC}"
    exit 0
else
    echo -e "${YELLOW}⚠ Some checks failed. Review the output above.${NC}"
    echo ""
    echo "Common fixes:"
    echo "  - Start services: sudo systemctl start r2d2-camera-perception.service r2d2-audio-notification.service"
    echo "  - Start rosbridge: cd ~/dev/r2d2/web_dashboard && ./start_rosbridge.sh"
    echo "  - Check service logs: sudo journalctl -u <service-name> -n 20"
    exit 1
fi
