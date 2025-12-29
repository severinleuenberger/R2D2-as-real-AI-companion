#!/bin/bash
# R2D2 Audio System Test Script
# Tests all audio components: PulseAudio, Bluetooth, Volume, Speech, Beeps
#
# Usage: ./test_audio_system.sh [--all | --quick | --bluetooth | --speech]
#
# Date: 2025-12-29

# Don't exit on error - we want to see all test results
# set -e

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Test counters
PASSED=0
FAILED=0
SKIPPED=0

print_header() {
    echo -e "\n${BLUE}═══════════════════════════════════════════════════════════════${NC}"
    echo -e "${BLUE}  $1${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════════════════════════${NC}\n"
}

print_test() {
    echo -e "${YELLOW}► Testing:${NC} $1"
}

print_pass() {
    echo -e "${GREEN}  ✓ PASS:${NC} $1"
    ((PASSED++))
}

print_fail() {
    echo -e "${RED}  ✗ FAIL:${NC} $1"
    ((FAILED++))
}

print_skip() {
    echo -e "${YELLOW}  ○ SKIP:${NC} $1"
    ((SKIPPED++))
}

print_info() {
    echo -e "${BLUE}  ℹ INFO:${NC} $1"
}

# ============================================================================
# Test 1: PulseAudio Connection
# ============================================================================
test_pulseaudio() {
    print_header "Test 1: PulseAudio Connection"
    
    print_test "PulseAudio daemon running"
    if pulseaudio --check 2>/dev/null; then
        print_pass "PulseAudio daemon is running"
    else
        print_fail "PulseAudio daemon not running"
        echo "  Try: pulseaudio --start"
        return 1
    fi
    
    print_test "Default sink available"
    DEFAULT_SINK=$(pactl get-default-sink 2>/dev/null)
    if [ -n "$DEFAULT_SINK" ]; then
        print_pass "Default sink: $DEFAULT_SINK"
    else
        print_fail "No default sink configured"
        return 1
    fi
}

# ============================================================================
# Test 2: Bluetooth Device
# ============================================================================
test_bluetooth() {
    print_header "Test 2: Bluetooth Device"
    
    print_test "Bluetooth controller available"
    if bluetoothctl show 2>/dev/null | grep -q "Powered: yes"; then
        print_pass "Bluetooth controller powered on"
    else
        print_fail "Bluetooth controller not powered"
        return 1
    fi
    
    print_test "Bluetooth device connected"
    BT_CONNECTED=$(bluetoothctl info 2>/dev/null | grep "Connected: yes")
    if [ -n "$BT_CONNECTED" ]; then
        BT_NAME=$(bluetoothctl info 2>/dev/null | grep "Name:" | cut -d: -f2 | xargs)
        print_pass "Connected to: $BT_NAME"
    else
        print_skip "No Bluetooth device connected"
    fi
    
    print_test "Bluetooth sink in PulseAudio"
    BT_SINK=$(pactl list sinks short 2>/dev/null | grep bluez)
    if [ -n "$BT_SINK" ]; then
        print_pass "Bluetooth sink available"
        print_info "$BT_SINK"
    else
        print_skip "No Bluetooth sink (device may not be audio-capable)"
    fi
}

# ============================================================================
# Test 3: Audio Playback
# ============================================================================
test_playback() {
    print_header "Test 3: Audio Playback"
    
    print_test "Test tone via PulseAudio (listen for beep)"
    if paplay /usr/share/sounds/alsa/Front_Center.wav 2>/dev/null; then
        print_pass "paplay completed successfully"
    else
        print_fail "paplay failed"
    fi
    
    print_test "Test tone via ffplay (listen for 800Hz beep)"
    if timeout 3 ffplay -autoexit -nodisp -f lavfi "sine=frequency=800:duration=0.5" 2>/dev/null; then
        print_pass "ffplay completed successfully"
    else
        print_fail "ffplay failed"
    fi
    
    echo ""
    read -p "Did you hear both test sounds? (y/n): " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        print_pass "Audio playback confirmed by user"
    else
        print_fail "Audio playback not heard"
    fi
}

# ============================================================================
# Test 4: Volume Control
# ============================================================================
test_volume() {
    print_header "Test 4: Volume Control"
    
    # Source ROS2 environment
    source /opt/ros/humble/setup.bash 2>/dev/null
    source ~/dev/r2d2/ros2_ws/install/setup.bash 2>/dev/null
    
    print_test "Volume control node running"
    if ros2 node list 2>/dev/null | grep -q volume_control; then
        print_pass "volume_control_node is running"
    else
        print_fail "volume_control_node not found"
        print_info "Check: sudo systemctl status r2d2-volume-control"
    fi
    
    print_test "Master volume topic publishing"
    VOLUME=$(timeout 5 ros2 topic echo /r2d2/audio/master_volume --once 2>/dev/null | grep "data:" | awk '{print $2}')
    if [ -n "$VOLUME" ]; then
        print_pass "Master volume: $VOLUME"
    else
        print_fail "Master volume topic not publishing (timeout)"
        print_info "Try: ros2 topic echo /r2d2/audio/master_volume --once"
    fi
}

# ============================================================================
# Test 5: Speech Service
# ============================================================================
test_speech() {
    print_header "Test 5: Speech Service"
    
    # Source ROS2 environment
    source /opt/ros/humble/setup.bash 2>/dev/null
    source ~/dev/r2d2/ros2_ws/install/setup.bash 2>/dev/null
    
    print_test "Speech node service status"
    if systemctl is-active --quiet r2d2-speech-node; then
        print_pass "r2d2-speech-node service is active"
    else
        print_fail "r2d2-speech-node service not active"
        return 1
    fi
    
    print_test "Speech node lifecycle state"
    STATE=$(ros2 lifecycle get /speech_node 2>/dev/null | head -1)
    if echo "$STATE" | grep -q "active"; then
        print_pass "Speech node is active"
    else
        print_fail "Speech node state: $STATE"
    fi
    
    print_test "PulseAudio environment in speech process"
    # Get the speech_node process started by systemd (child of ros2 launch)
    SPEECH_PID=$(systemctl show r2d2-speech-node --property=MainPID --value 2>/dev/null)
    if [ -n "$SPEECH_PID" ] && [ "$SPEECH_PID" != "0" ]; then
        # Get child process (actual speech_node)
        CHILD_PID=$(pgrep -P $SPEECH_PID 2>/dev/null | head -1)
        if [ -n "$CHILD_PID" ]; then
            SPEECH_PID=$CHILD_PID
        fi
        PULSE_ENV=$(cat /proc/$SPEECH_PID/environ 2>/dev/null | tr '\0' '\n' | grep PULSE_SERVER)
        if [ -n "$PULSE_ENV" ]; then
            print_pass "PulseAudio environment set (PID: $SPEECH_PID)"
            print_info "$PULSE_ENV"
        else
            print_fail "PulseAudio environment NOT set in speech process (PID: $SPEECH_PID)"
        fi
    else
        print_fail "Speech process not found"
    fi
    
    print_test "sink_device configuration"
    SINK_DEVICE=$(grep sink_device ~/dev/r2d2/ros2_ws/install/r2d2_speech/share/r2d2_speech/config/speech_params.yaml 2>/dev/null | awk '{print $2}')
    print_info "sink_device: $SINK_DEVICE"
    if echo "$SINK_DEVICE" | grep -q "pulse"; then
        print_pass "Configured for PulseAudio (Bluetooth)"
    elif echo "$SINK_DEVICE" | grep -q "default"; then
        print_info "Configured for ALSA direct (PAM8403)"
    fi
}

# ============================================================================
# Test 6: Audio Notification Service
# ============================================================================
test_notifications() {
    print_header "Test 6: Audio Notification Service"
    
    print_test "Audio notification service status"
    if systemctl is-active --quiet r2d2-audio-notification; then
        print_pass "r2d2-audio-notification service is active"
    else
        print_fail "r2d2-audio-notification service not active"
    fi
    
    print_test "Gesture intent service status"
    if systemctl is-active --quiet r2d2-gesture-intent; then
        print_pass "r2d2-gesture-intent service is active"
    else
        print_fail "r2d2-gesture-intent service not active"
    fi
}

# ============================================================================
# Test 7: Interactive Speech Test
# ============================================================================
test_speech_interactive() {
    print_header "Test 7: Interactive Speech Test"
    
    source /opt/ros/humble/setup.bash 2>/dev/null
    source ~/dev/r2d2/ros2_ws/install/setup.bash 2>/dev/null
    
    echo "This test will start a speech session."
    read -p "Press Enter to start session, then speak to R2D2..."
    
    ros2 service call /r2d2/speech/start_session std_srvs/srv/Trigger "{}" 2>/dev/null
    
    echo ""
    echo "Session started. Speak now and listen for AI response."
    echo "Press Enter when done testing..."
    read
    
    ros2 service call /r2d2/speech/stop_session std_srvs/srv/Trigger "{}" 2>/dev/null
    
    read -p "Did you hear the AI response? (y/n): " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        print_pass "Speech system working correctly"
    else
        print_fail "Speech audio not heard"
        echo "  Check: journalctl -u r2d2-speech-node --since '2 minutes ago' | grep -i pulse"
    fi
}

# ============================================================================
# Summary
# ============================================================================
print_summary() {
    print_header "Test Summary"
    echo -e "  ${GREEN}Passed:${NC}  $PASSED"
    echo -e "  ${RED}Failed:${NC}  $FAILED"
    echo -e "  ${YELLOW}Skipped:${NC} $SKIPPED"
    echo ""
    
    if [ $FAILED -eq 0 ]; then
        echo -e "${GREEN}All tests passed!${NC}"
    else
        echo -e "${RED}Some tests failed. Check the output above for details.${NC}"
    fi
}

# ============================================================================
# Main
# ============================================================================
main() {
    echo ""
    echo "╔═══════════════════════════════════════════════════════════════╗"
    echo "║           R2D2 Audio System Test Suite                        ║"
    echo "║                    Version 1.0                                ║"
    echo "╚═══════════════════════════════════════════════════════════════╝"
    
    case "${1:-all}" in
        --quick)
            test_pulseaudio
            test_volume
            test_speech
            ;;
        --bluetooth)
            test_pulseaudio
            test_bluetooth
            test_playback
            ;;
        --speech)
            test_speech
            test_speech_interactive
            ;;
        --all|*)
            test_pulseaudio
            test_bluetooth
            test_playback
            test_volume
            test_speech
            test_notifications
            
            echo ""
            read -p "Run interactive speech test? (y/n): " -n 1 -r
            echo ""
            if [[ $REPLY =~ ^[Yy]$ ]]; then
                test_speech_interactive
            else
                print_skip "Interactive speech test"
            fi
            ;;
    esac
    
    print_summary
}

main "$@"

