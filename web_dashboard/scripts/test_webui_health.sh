#!/bin/bash
# WebUI Health Check - Smoke Test
# Tests that all WebUI components are operational

set +e  # Don't exit on error, collect all results

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

PASSED=0
FAILED=0

print_test() {
    echo -n "  Testing: $1... "
}

print_pass() {
    echo -e "${GREEN}✅ PASS${NC}"
    ((PASSED++))
}

print_fail() {
    echo -e "${RED}❌ FAIL${NC}"
    echo "           $1"
    ((FAILED++))
}

echo "╔════════════════════════════════════════════════════════════╗"
echo "║            R2D2 WebUI Health Check                         ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Source ROS environment for topic tests
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/dev/r2d2/ros2_ws/install/setup.bash 2>/dev/null

# Test 1: Port 8080 (FastAPI)
print_test "FastAPI server (port 8080)"
if netstat -tuln 2>/dev/null | grep -q ":8080.*LISTEN"; then
    print_pass
else
    print_fail "Port 8080 not listening. Is r2d2-web-dashboard running?"
fi

# Test 2: Port 9090 (rosbridge)
print_test "rosbridge WebSocket (port 9090)"
if netstat -tuln 2>/dev/null | grep -q ":9090.*LISTEN"; then
    print_pass
else
    print_fail "Port 9090 not listening. Is r2d2-rosbridge running?"
fi

# Test 3: API health endpoint
print_test "API health endpoint"
HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" http://100.95.133.26:8080/health 2>/dev/null || echo "000")
if [ "$HTTP_CODE" = "200" ]; then
    print_pass
else
    print_fail "Health endpoint returned $HTTP_CODE (expected 200)"
fi

# Test 4: rosbridge node
print_test "rosbridge ROS node"
if timeout 3 ros2 node list 2>/dev/null | grep -q rosbridge; then
    print_pass
else
    print_fail "rosbridge node not found. Check: ros2 node list"
fi

# Test 5: Core ROS topics available
print_test "Core ROS topics publishing"
if timeout 2 ros2 topic echo /r2d2/audio/person_status --once 2>/dev/null | grep -q "status"; then
    print_pass
else
    print_fail "Status topic not publishing. Check core services."
fi

# Test 6: WebSocket connection (basic)
print_test "WebSocket endpoint accessible"
if timeout 2 curl -s -i -N -H "Connection: Upgrade" -H "Upgrade: websocket" \
    -H "Sec-WebSocket-Version: 13" -H "Sec-WebSocket-Key: test" \
    http://localhost:9090 2>/dev/null | head -1 | grep -q "101"; then
    print_pass
else
    print_fail "WebSocket upgrade failed on port 9090"
fi

# Summary
echo ""
echo "═══════════════════════════════════════════════════════════"
echo "  Summary"
echo "═══════════════════════════════════════════════════════════"
echo -e "  ${GREEN}Passed:${NC} $PASSED/6"
echo -e "  ${RED}Failed:${NC} $FAILED/6"
echo ""

if [ $FAILED -eq 0 ]; then
    echo -e "${GREEN}✅ All WebUI health checks passed!${NC}"
    echo ""
    echo "WebUI should be accessible at: http://100.95.133.26:8080"
    exit 0
else
    echo -e "${RED}⚠️  Some checks failed. WebUI may not be fully operational.${NC}"
    echo ""
    echo "Troubleshooting:"
    echo "  - Check services: systemctl status r2d2-web-dashboard r2d2-rosbridge"
    echo "  - Check logs: journalctl -u r2d2-web-dashboard -n 50"
    echo "  - Restart: sudo systemctl restart r2d2-web-dashboard r2d2-rosbridge"
    echo ""
    echo "Recovery: See 004_BACKUP_AND_RESTORE.md - Emergency Rollback Reference"
    exit 1
fi

