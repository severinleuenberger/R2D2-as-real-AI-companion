#!/bin/bash
# Test script for audio notification system
# Verifies audio node is running and responding to recognition events

echo "🔊 R2D2 Audio Notification System Test"
echo "======================================="
echo ""

# Source ROS 2 environment
source /opt/ros/humble/setup.bash 2>/dev/null
if [ -f ~/dev/r2d2/ros2_ws/install/setup.bash ]; then
    source ~/dev/r2d2/ros2_ws/install/setup.bash
fi

# Test 1: Check audio node
echo "Test 1: Audio Notification Node Status"
if ros2 node list 2>/dev/null | grep -q audio_notification_node; then
    echo "  ✅ Audio notification node is running"
    
    # Get target person
    TARGET=$(ros2 param get /audio_notification_node target_person 2>&1 | grep "String value is" | awk -F"'" '{print $2}' || echo "unknown")
    VOLUME=$(ros2 param get /audio_notification_node audio_volume 2>&1 | grep "Double value is" | awk '{print $3}' || echo "unknown")
    ALSA_DEVICE=$(ros2 param get /audio_notification_node alsa_device 2>&1 | grep "String value is" | awk -F"'" '{print $2}' || echo "unknown")
    
    echo "  Target person: $TARGET"
    echo "  Audio volume: $VOLUME"
    echo "  ALSA device: $ALSA_DEVICE"
else
    echo "  ❌ Audio notification node is NOT running"
    echo "     Start with: ros2 launch r2d2_audio audio_notification.launch.py"
    exit 1
fi

# Test 2: Check status topic
echo ""
echo "Test 2: Status Topic (/r2d2/audio/person_status)"
if ros2 topic list 2>/dev/null | grep -q "/r2d2/audio/person_status"; then
    echo "  ✅ Status topic exists"
    
    # Check rate
    RATE=$(timeout 2 ros2 topic hz /r2d2/audio/person_status 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "0")
    if (( $(echo "$RATE > 5" | bc -l) )); then
        echo "  ✅ Rate: $RATE Hz (GOOD)"
    else
        echo "  ⚠️  Rate: $RATE Hz (LOW - expected ~10 Hz)"
    fi
    
    # Get current status
    STATUS_JSON=$(timeout 1 ros2 topic echo /r2d2/audio/person_status --once 2>/dev/null | grep -o '{"[^}]*}' | head -1)
    if [ ! -z "$STATUS_JSON" ]; then
        STATUS=$(echo $STATUS_JSON | grep -o '"status":"[^"]*"' | cut -d'"' -f4)
        echo "  Current status: $STATUS"
    fi
else
    echo "  ❌ Status topic not found"
    exit 1
fi

# Test 3: Check notification events
echo ""
echo "Test 3: Notification Events"
if ros2 topic list 2>/dev/null | grep -q "/r2d2/audio/notification_event"; then
    echo "  ✅ Notification event topic exists"
    echo "  Monitoring for events (5 seconds)..."
    timeout 5 ros2 topic echo /r2d2/audio/notification_event 2>/dev/null | grep "data:" | head -3 || echo "  ⚠️  No events in last 5 seconds"
else
    echo "  ❌ Notification event topic not found"
fi

# Test 4: Simulate recognition
echo ""
echo "Test 4: Simulate Recognition Event"
echo "  Publishing target person recognition..."
ros2 topic pub --once /r2d2/perception/person_id std_msgs/String "{data: $TARGET}" 2>/dev/null
echo "  ✅ Published recognition event"
echo "  Waiting 2 seconds for audio response..."
sleep 2

# Check if status changed
NEW_STATUS_JSON=$(timeout 1 ros2 topic echo /r2d2/audio/person_status --once 2>/dev/null | grep -o '{"[^}]*}' | head -1)
if [ ! -z "$NEW_STATUS_JSON" ]; then
    NEW_STATUS=$(echo $NEW_STATUS_JSON | grep -o '"status":"[^"]*"' | cut -d'"' -f4)
    if [ "$NEW_STATUS" = "red" ]; then
        echo "  ✅ Status changed to RED (recognition successful)"
    else
        echo "  ⚠️  Status: $NEW_STATUS (expected RED)"
    fi
fi

echo ""
echo "======================================="
echo "✅ Audio Notification System Test Complete"
echo ""
echo "Note: If you didn't hear audio, check:"
echo "  1. Audio volume parameter (currently: $VOLUME)"
echo "  2. ALSA device configuration (currently: $ALSA_DEVICE)"
echo "  3. Speaker hardware connection"

