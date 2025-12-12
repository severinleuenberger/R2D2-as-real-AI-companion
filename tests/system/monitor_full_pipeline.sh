#!/bin/bash
# Comprehensive monitoring script for the complete R2D2 pipeline
# Shows all topics, rates, and system health in a dashboard format

echo "📊 R2D2 Complete System Monitor"
echo "================================"
echo ""

# Source ROS 2 environment
source /opt/ros/humble/setup.bash 2>/dev/null
if [ -f ~/dev/r2d2/ros2_ws/install/setup.bash ]; then
    source ~/dev/r2d2/ros2_ws/install/setup.bash
fi

# Function to get topic rate
get_topic_rate() {
    local topic=$1
    local duration=${2:-2}
    timeout $duration ros2 topic hz $topic 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "0.0"
}

# Function to get last message
get_last_message() {
    local topic=$1
    timeout 1 ros2 topic echo $topic --once 2>/dev/null | grep "data:" | head -1 | sed 's/.*data: //' || echo "N/A"
}

# Clear screen and show dashboard
while true; do
    clear
    echo "📊 R2D2 System Monitor - $(date '+%H:%M:%S')"
    echo "=========================================="
    echo ""
    
    # System Status
    echo "🔧 System Status:"
    echo "  Camera Node:        $(ros2 node list 2>/dev/null | grep -q camera_node && echo '✅ RUNNING' || echo '❌ STOPPED')"
    echo "  Perception Node:    $(ros2 node list 2>/dev/null | grep -q image_listener && echo '✅ RUNNING' || echo '❌ STOPPED')"
    echo "  Audio Node:         $(ros2 node list 2>/dev/null | grep -q audio_notification_node && echo '✅ RUNNING' || echo '❌ STOPPED')"
    echo "  LED Node:           $(ros2 node list 2>/dev/null | grep -q status_led_controller && echo '✅ RUNNING' || echo '⚠️  STOPPED (optional)')"
    echo ""
    
    # Topic Rates
    echo "📈 Topic Rates (Hz):"
    CAMERA_RATE=$(get_topic_rate /oak/rgb/image_raw 1)
    PERCEPTION_RATE=$(get_topic_rate /r2d2/perception/face_count 1)
    PERSON_ID_RATE=$(get_topic_rate /r2d2/perception/person_id 1)
    STATUS_RATE=$(get_topic_rate /r2d2/audio/person_status 1)
    
    printf "  Camera (RGB):       %5.1f Hz (expected: 30.0)\n" $CAMERA_RATE
    printf "  Face Detection:     %5.1f Hz (expected: 13.0)\n" $PERCEPTION_RATE
    printf "  Person ID:          %5.1f Hz (expected: 6.5)\n" $PERSON_ID_RATE
    printf "  Status:              %5.1f Hz (expected: 10.0)\n" $STATUS_RATE
    echo ""
    
    # Current Values
    echo "📊 Current Values:"
    FACE_COUNT=$(get_last_message /r2d2/perception/face_count)
    PERSON_ID=$(get_last_message /r2d2/perception/person_id)
    BRIGHTNESS=$(timeout 1 ros2 topic echo /r2d2/perception/brightness --once 2>/dev/null | grep "data:" | awk '{print $2}' || echo "N/A")
    
    echo "  Faces Detected:      $FACE_COUNT"
    echo "  Person ID:           $PERSON_ID"
    echo "  Brightness:          $BRIGHTNESS"
    echo ""
    
    # Status JSON (formatted)
    echo "🎯 Current Status:"
    STATUS_JSON=$(timeout 1 ros2 topic echo /r2d2/audio/person_status --once 2>/dev/null | grep -o '{"[^}]*}' | head -1)
    if [ ! -z "$STATUS_JSON" ]; then
        STATUS=$(echo $STATUS_JSON | grep -o '"status":"[^"]*"' | cut -d'"' -f4)
        PERSON=$(echo $STATUS_JSON | grep -o '"person_identity":"[^"]*"' | cut -d'"' -f4)
        CONFIDENCE=$(echo $STATUS_JSON | grep -o '"confidence":[0-9.]*' | cut -d':' -f2)
        DURATION=$(echo $STATUS_JSON | grep -o '"duration_in_state":[0-9.]*' | cut -d':' -f2)
        
        case $STATUS in
            "red")
                echo "  🔴 RED - Target person recognized"
                ;;
            "blue")
                echo "  🔵 BLUE - No person (idle)"
                ;;
            "green")
                echo "  🟢 GREEN - Unknown person"
                ;;
            *)
                echo "  ⚪ UNKNOWN"
                ;;
        esac
        echo "  Person:              $PERSON"
        echo "  Confidence:          ${CONFIDENCE:-N/A}"
        echo "  Duration:            ${DURATION:-N/A}s"
    else
        echo "  ⚠️  No status data available"
    fi
    echo ""
    
    # Recent Events
    echo "🔔 Recent Events (last 3):"
    timeout 2 ros2 topic echo /r2d2/audio/notification_event --once 2>/dev/null | grep "data:" | tail -3 | sed 's/.*data: /  /' || echo "  No recent events"
    echo ""
    
    echo "Press Ctrl+C to stop monitoring"
    sleep 2
done


