#!/bin/bash
################################################################################
# Test Face Detection Smoothing Fix
# Tests the 5-second stability requirement before face recognition runs
################################################################################

echo "╔═══════════════════════════════════════════════════════════╗"
echo "║      TESTING FACE DETECTION SMOOTHING FIX            ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Step 1: Build the updated code
echo -e "${BLUE}Step 1: Building updated r2d2_perception package...${NC}"
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_perception --symlink-install
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ Build successful${NC}"
else
    echo -e "${RED}❌ Build failed${NC}"
    exit 1
fi
echo ""

# Step 2: Restart the camera-perception service
echo -e "${BLUE}Step 2: Restarting r2d2-camera-perception service...${NC}"
sudo systemctl restart r2d2-camera-perception.service
sleep 3

if systemctl is-active --quiet r2d2-camera-perception.service; then
    echo -e "${GREEN}✅ Service restarted successfully${NC}"
else
    echo -e "${RED}❌ Service failed to start${NC}"
    echo "Check logs: sudo journalctl -u r2d2-camera-perception.service -n 50"
    exit 1
fi
echo ""

# Step 3: Verify the new parameters
echo -e "${BLUE}Step 3: Verifying hysteresis parameters...${NC}"
cd ~/dev/r2d2/ros2_ws && source install/setup.bash
PRESENCE=$(ros2 param get /image_listener face_presence_threshold 2>/dev/null | grep "value is:" | awk '{print $3}')
ABSENCE=$(ros2 param get /image_listener face_absence_threshold 2>/dev/null | grep "value is:" | awk '{print $3}')
if [ -n "$PRESENCE" ] && [ -n "$ABSENCE" ]; then
    echo -e "${GREEN}✅ face_presence_threshold = ${PRESENCE} seconds${NC}"
    echo -e "${GREEN}✅ face_absence_threshold = ${ABSENCE} seconds${NC}"
else
    echo -e "${YELLOW}⚠️  Could not read parameters (may need to wait for node to initialize)${NC}"
fi
echo ""

# Step 4: Instructions for monitoring
echo -e "${BLUE}Step 4: Monitoring Setup${NC}"
echo ""
echo "Open 3 terminal windows and run these commands:"
echo ""
echo -e "${YELLOW}Terminal 1 - Monitor face_count (should be STABLE, not flickering):${NC}"
echo "  ros2 topic echo /r2d2/perception/face_count"
echo ""
echo -e "${YELLOW}Terminal 2 - Monitor person_id (should STOP publishing when you walk away):${NC}"
echo "  ros2 topic echo /r2d2/perception/person_id"
echo ""
echo -e "${YELLOW}Terminal 3 - Monitor audio notification logs (watch for timer resets):${NC}"
echo "  sudo journalctl -u r2d2-audio-notification -f | grep -E '(Timer reset|Lost you|RED|BLUE)'"
echo ""
echo -e "${BLUE}Press ENTER when you have the monitoring terminals ready...${NC}"
read

# Step 5: Test scenario
echo ""
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║              TEST SCENARIO                            ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""
echo -e "${GREEN}Test Steps:${NC}"
echo "  1. Stand in front of camera (face should be detected)"
echo "  2. Wait ~2 seconds for face to stabilize (hysteresis delay)"
echo "  3. You should hear 'Hello!' beep (recognition after stability)"
echo "  4. Start conversation with index finger gesture"
echo "  5. Speak briefly"
echo "  6. Stop with fist gesture"
echo "  7. ${RED}IMMEDIATELY walk completely out of camera frame${NC}"
echo "  8. Stay away for 25 seconds (empty room, no objects)"
echo "  9. ${GREEN}EXPECT: 'Lost you!' beep at ~20 second mark (5s + 15s)${NC}"
echo "  10. Return to camera"
echo "  11. ${GREEN}EXPECT: 'Hello!' beep after 2 seconds${NC}"
echo ""
echo -e "${BLUE}What to watch in monitoring terminals:${NC}"
echo "  • face_count: Should be STABLE (0 or 1, no flickering)"
echo "  • person_id: Should STOP publishing after 5s absence window"
echo "  • Timer logs: Should NOT reset during 5s absence window"
echo ""
echo -e "${YELLOW}Press ENTER to start the test...${NC}"
read

echo ""
echo -e "${GREEN}Starting test - follow the steps above!${NC}"
echo ""

# Step 6: Quick verification after test
echo -e "${BLUE}After completing the test, press ENTER to see verification...${NC}"
read

echo ""
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║              TEST RESULTS                            ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""

echo -e "${BLUE}Checking recent audio notification activity...${NC}"
sudo journalctl -u r2d2-audio-notification -n 50 --no-pager | grep -E "(Lost you|Timer reset|RED|BLUE)" | tail -10

echo ""
echo -e "${GREEN}Expected Results:${NC}"
echo "  ✅ 'Lost you!' beep played at ~20 seconds after walking away (5s + 15s)"
echo "  ✅ No false recognitions resetting RED timer"
echo "  ✅ person_id stopped publishing after 5s absence window"
echo "  ✅ face_count remained stable (not flickering)"
echo "  ✅ 'Hello!' beep played after 2s when you returned"
echo ""
echo -e "${YELLOW}If 'Lost you!' beep didn't play or was delayed >25s:${NC}"
echo "  • Check if face_count is flickering (shouldn't be with hysteresis)"
echo "  • Check logs: sudo journalctl -u r2d2-camera-perception.service -n 100"
echo "  • Consider increasing face_absence_threshold parameter"
echo ""

