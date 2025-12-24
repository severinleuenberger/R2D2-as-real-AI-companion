#!/bin/bash
################################################################################
# Gesture System - Live Test Script
#
# Purpose:
#   Test the complete gesture recognition system:
#   - Index finger up → Start speech service (beep 16.mp3)
#   - Fist → Stop speech service (beep 20.mp3)
#
# Prerequisites:
#   - Gestures trained for target person
#   - Camera perception service running
#   - Audio notification node running
#   - Speech node available
#
# Usage:
#   ./test_gesture_system_live.sh
#
# Author: R2D2 System
# Date: December 17, 2025
################################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Configuration
TARGET_PERSON="severin"
GESTURE_MODEL="/home/severin/dev/r2d2/data/gesture_recognition/models/severin_gesture_classifier.pkl"
FACE_MODEL="/home/severin/dev/r2d2/data/face_recognition/models/Severin_Leuenberger_lbph.xml"

################################################################################
# Header
################################################################################
clear
echo "========================================================================"
echo "           R2D2 GESTURE SYSTEM - LIVE TEST"
echo "========================================================================"
echo ""
echo -e "${CYAN}Testing the complete gesture-to-speech integration:${NC}"
echo "  • Face recognition detects you"
echo "  • Gesture recognition triggers speech service"
echo "  • Audio feedback confirms actions"
echo ""
echo "========================================================================"
echo ""

################################################################################
# Pre-flight Checks
################################################################################
echo -e "${BLUE}[1/6] Pre-flight checks...${NC}"

# Check if gesture model exists
if [ ! -f "$GESTURE_MODEL" ]; then
    echo -e "${RED}✗ Gesture model not found: $GESTURE_MODEL${NC}"
    echo "Please train gestures first!"
    exit 1
fi
echo -e "${GREEN}✓ Gesture model exists${NC}"

# Check if face model exists
if [ ! -f "$FACE_MODEL" ]; then
    echo -e "${YELLOW}⚠ Face model not found: $FACE_MODEL${NC}"
    echo "Face recognition may not work properly."
else
    echo -e "${GREEN}✓ Face model exists${NC}"
fi

echo ""

################################################################################
# Check Required Services
################################################################################
echo -e "${BLUE}[2/6] Checking services...${NC}"

# Check if camera perception is running
if systemctl is-active --quiet r2d2-camera-perception.service; then
    echo -e "${GREEN}✓ Camera perception service is running${NC}"
else
    echo -e "${YELLOW}⚠ Camera perception service is not running${NC}"
    echo ""
    echo "Starting camera perception service..."
    sudo systemctl start r2d2-camera-perception.service
    sleep 3
    if systemctl is-active --quiet r2d2-camera-perception.service; then
        echo -e "${GREEN}✓ Service started successfully${NC}"
    else
        echo -e "${RED}✗ Failed to start camera perception service${NC}"
        exit 1
    fi
fi

echo ""

################################################################################
# Check ROS2 Environment
################################################################################
echo -e "${BLUE}[3/6] Checking ROS2 environment...${NC}"

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/dev/r2d2/ros2_ws/install/setup.bash

# Check if nodes are running
sleep 2
if ros2 node list | grep -q "image_listener"; then
    echo -e "${GREEN}✓ image_listener node is running${NC}"
else
    echo -e "${RED}✗ image_listener node not found${NC}"
    echo "Please check the camera perception service"
    exit 1
fi

echo ""

################################################################################
# Launch Gesture Intent Node
################################################################################
echo -e "${BLUE}[4/6] Launching gesture intent node...${NC}"
echo ""
echo "This node will:"
echo "  • Listen for gesture events"
echo "  • Control speech service"
echo "  • Play audio feedback (beeps)"
echo ""

# Launch in new terminal
gnome-terminal --title="R2D2 Gesture Intent Node" -- bash -c "
    source /opt/ros/humble/setup.bash;
    source ~/dev/r2d2/ros2_ws/install/setup.bash;
    ros2 launch r2d2_gesture gesture_intent.launch.py \
        enabled:=true \
        cooldown_start_seconds:=5.0 \
        cooldown_stop_seconds:=3.0 \
        auto_shutdown_enabled:=true \
        auto_shutdown_timeout_seconds:=300.0 \
        auto_restart_on_return:=false \
        audio_feedback_enabled:=true;
    exec bash
" &

sleep 3
echo -e "${GREEN}✓ Gesture intent node launched${NC}"
echo ""

################################################################################
# Check Topics
################################################################################
echo -e "${BLUE}[5/6] Verifying topics...${NC}"

# Check gesture event topic
if ros2 topic list | grep -q "/r2d2/perception/gesture_event"; then
    echo -e "${GREEN}✓ /r2d2/perception/gesture_event topic exists${NC}"
else
    echo -e "${YELLOW}⚠ Gesture event topic not found${NC}"
fi

# Check person status topic
if ros2 topic list | grep -q "/r2d2/audio/person_status"; then
    echo -e "${GREEN}✓ /r2d2/audio/person_status topic exists${NC}"
else
    echo -e "${YELLOW}⚠ Person status topic not found${NC}"
fi

echo ""

################################################################################
# Test Instructions
################################################################################
echo -e "${BLUE}[6/6] System ready for testing!${NC}"
echo ""
echo "========================================================================"
echo -e "${CYAN}TEST PROCEDURE:${NC}"
echo "========================================================================"
echo ""
echo "1. FACE RECOGNITION TEST"
echo "   • Stand in front of the camera"
echo "   • Wait for LED to turn RED (you're recognized)"
echo "   • Check terminal output for 'Target person detected'"
echo ""
echo "2. START GESTURE TEST (Index Finger Up)"
echo "   • Raise your INDEX FINGER pointing upward"
echo "   • Hold for 2-3 seconds"
echo "   • Expected: Hear beep (R2D2 sound 16.mp3)"
echo "   • Expected: Speech service starts"
echo "   • Check terminal: 'Gesture: index_finger_up'"
echo ""
echo "3. SPEECH SERVICE TEST"
echo "   • Say something to R2D2"
echo "   • Check if it responds"
echo ""
echo "4. STOP GESTURE TEST (Fist)"
echo "   • Make a FIST (all fingers closed)"
echo "   • Hold for 2-3 seconds"
echo "   • Expected: Hear beep (R2D2 sound 20.mp3)"
echo "   • Expected: Speech service stops"
echo "   • Check terminal: 'Gesture: fist'"
echo ""
echo "5. AUTO-SHUTDOWN TEST (Optional)"
echo "   • Start speech service with gesture"
echo "   • Walk away from camera (LED turns BLUE)"
echo "   • Wait 5 minutes"
echo "   • Expected: Service auto-stops with beep"
echo ""
echo "========================================================================"
echo -e "${CYAN}MONITORING:${NC}"
echo "========================================================================"
echo ""
echo "Terminal Windows:"
echo "  • This terminal: Main instructions"
echo "  • Gesture Intent Node: Gesture events and service control"
echo ""
echo "ROS2 Topics to Monitor:"
echo ""
echo -e "${YELLOW}  Person Status:${NC}"
echo "    ros2 topic echo /r2d2/audio/person_status"
echo ""
echo -e "${YELLOW}  Gesture Events:${NC}"
echo "    ros2 topic echo /r2d2/perception/gesture_event"
echo ""
echo -e "${YELLOW}  Speech Session Status:${NC}"
echo "    ros2 topic echo /r2d2/speech/session_status"
echo ""
echo "========================================================================"
echo -e "${CYAN}TROUBLESHOOTING:${NC}"
echo "========================================================================"
echo ""
echo "If gestures not recognized:"
echo "  1. Check face is recognized (RED LED)"
echo "  2. Hold gesture for 2-3 seconds"
echo "  3. Make gesture clear and distinct"
echo "  4. Check lighting conditions"
echo ""
echo "If no audio feedback:"
echo "  1. Check volume: amixer get Master"
echo "  2. Test audio files exist: ls ~/Voicy_R2-D2*"
echo "  3. Check gesture_intent_node terminal for errors"
echo ""
echo "If speech service doesn't start:"
echo "  1. Launch speech service: bash ~/dev/r2d2/launch_ros2_speech.sh"
echo "  2. Check service is available: ros2 service list | grep speech"
echo ""
echo "========================================================================"
echo ""
echo -e "${GREEN}Press ENTER to start monitoring gesture events...${NC}"
read

echo ""
echo "Monitoring gesture events (Ctrl+C to stop):"
echo ""
ros2 topic echo /r2d2/perception/gesture_event

