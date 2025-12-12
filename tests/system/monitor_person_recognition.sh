#!/bin/bash
# Real-time monitoring script for person recognition system
# Displays live stream of system behavior including recognition, status, and audio events

echo "🔍 R2D2 Person Recognition System - Live Monitor"
echo "=================================================="
echo ""
echo "This script monitors the complete person recognition pipeline in real-time."
echo "Press Ctrl+C to stop monitoring."
echo ""

# Source ROS 2 environment
source /opt/ros/humble/setup.bash 2>/dev/null
if [ -f ~/dev/r2d2/ros2_ws/install/setup.bash ]; then
    source ~/dev/r2d2/ros2_ws/install/setup.bash
fi

# Check if nodes are running
echo "📡 Checking system status..."
echo ""

# Check camera node
if ros2 node list 2>/dev/null | grep -q camera_node; then
    echo "  ✅ Camera node: RUNNING"
else
    echo "  ❌ Camera node: NOT RUNNING"
fi

# Check perception node
if ros2 node list 2>/dev/null | grep -q image_listener; then
    echo "  ✅ Perception node: RUNNING"
else
    echo "  ❌ Perception node: NOT RUNNING"
fi

# Check audio notification node
if ros2 node list 2>/dev/null | grep -q audio_notification_node; then
    echo "  ✅ Audio notification node: RUNNING"
    TARGET_PERSON=$(ros2 param get /audio_notification_node target_person 2>&1 | grep "String value is" | awk -F"'" '{print $2}' || echo "unknown")
    echo "     Target person: $TARGET_PERSON"
else
    echo "  ❌ Audio notification node: NOT RUNNING"
fi

# Check LED node
if ros2 node list 2>/dev/null | grep -q status_led_controller; then
    echo "  ✅ LED node: RUNNING"
else
    echo "  ⚠️  LED node: NOT RUNNING (optional)"
fi

echo ""
echo "=================================================="
echo "📊 Live Monitoring (Press Ctrl+C to stop)"
echo "=================================================="
echo ""

# Function to format timestamp
timestamp() {
    date +"%H:%M:%S"
}

# Monitor person_id topic
echo "👤 Person Recognition Stream:"
echo "   Format: [TIME] person_id → status"
echo ""

ros2 topic echo /r2d2/perception/person_id --once 2>/dev/null | while IFS= read -r line; do
    if [[ $line =~ data:\ (.+) ]]; then
        PERSON_ID="${BASH_REMATCH[1]}"
        TIMESTAMP=$(timestamp)
        if [ "$PERSON_ID" = "unknown" ]; then
            echo "   [$TIMESTAMP] 👤 Person: $PERSON_ID"
        elif [ "$PERSON_ID" = "$TARGET_PERSON" ] || [ "$PERSON_ID" != "unknown" ]; then
            echo "   [$TIMESTAMP] ✅ Person: $PERSON_ID (RECOGNIZED)"
        else
            echo "   [$TIMESTAMP] 👤 Person: $PERSON_ID"
        fi
    fi
done &

# Monitor person_status topic
echo "📊 Status Stream:"
echo "   Format: [TIME] status | person | confidence | duration"
echo ""

ros2 topic echo /r2d2/audio/person_status --once 2>/dev/null | while IFS= read -r line; do
    if [[ $line =~ \"status\":\ \"([^\"]+)\" ]]; then
        STATUS="${BASH_REMATCH[1]}"
        TIMESTAMP=$(timestamp)
        case $STATUS in
            "red")
                echo "   [$TIMESTAMP] 🔴 RED | Target person recognized"
                ;;
            "blue")
                echo "   [$TIMESTAMP] 🔵 BLUE | No person (idle)"
                ;;
            "green")
                echo "   [$TIMESTAMP] 🟢 GREEN | Unknown person detected"
                ;;
        esac
    fi
done &

# Monitor notification events
echo "🔔 Audio Event Stream:"
echo "   Format: [TIME] event description"
echo ""

ros2 topic echo /r2d2/audio/notification_event --once 2>/dev/null | while IFS= read -r line; do
    if [[ $line =~ data:\ (.+) ]]; then
        EVENT="${BASH_REMATCH[1]}"
        TIMESTAMP=$(timestamp)
        if [[ $EVENT =~ 🎉 ]]; then
            echo "   [$TIMESTAMP] 🔊 $EVENT"
        elif [[ $EVENT =~ ❌ ]]; then
            echo "   [$TIMESTAMP] 🔔 $EVENT"
        else
            echo "   [$TIMESTAMP] ℹ️  $EVENT"
        fi
    fi
done &

# Monitor face count
echo "👥 Face Detection Stream:"
echo "   Format: [TIME] faces detected"
echo ""

ros2 topic echo /r2d2/perception/face_count --once 2>/dev/null | while IFS= read -r line; do
    if [[ $line =~ data:\ ([0-9]+) ]]; then
        COUNT="${BASH_REMATCH[1]}"
        TIMESTAMP=$(timestamp)
        if [ "$COUNT" -gt 0 ]; then
            echo "   [$TIMESTAMP] 👥 $COUNT face(s) detected"
        fi
    fi
done &

# Wait for user interrupt
wait


