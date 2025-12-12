#!/bin/bash
# Complete system integration test
# Tests the full pipeline from camera to audio output

echo "🚀 R2D2 Complete System Integration Test"
echo "========================================="
echo ""

# Source ROS 2 environment
source /opt/ros/humble/setup.bash 2>/dev/null
if [ -f ~/dev/r2d2/ros2_ws/install/setup.bash ]; then
    source ~/dev/r2d2/ros2_ws/install/setup.bash
fi

PASSED=0
FAILED=0

# Test function
test_check() {
    local name=$1
    local command=$2
    echo -n "Testing: $name... "
    if eval "$command" >/dev/null 2>&1; then
        echo "✅ PASSED"
        ((PASSED++))
        return 0
    else
        echo "❌ FAILED"
        ((FAILED++))
        return 1
    fi
}

echo "Phase 1: System Components"
echo "-------------------------"
test_check "Camera node running" "ros2 node list | grep -q camera_node"
test_check "Perception node running" "ros2 node list | grep -q image_listener"
test_check "Audio node running" "ros2 node list | grep -q audio_notification_node"

echo ""
echo "Phase 2: Topic Availability"
echo "---------------------------"
test_check "Camera topic exists" "ros2 topic list | grep -q /oak/rgb/image_raw"
test_check "Face count topic exists" "ros2 topic list | grep -q /r2d2/perception/face_count"
test_check "Person ID topic exists" "ros2 topic list | grep -q /r2d2/perception/person_id"
test_check "Status topic exists" "ros2 topic list | grep -q /r2d2/audio/person_status"

echo ""
echo "Phase 3: Data Flow"
echo "------------------"
echo -n "Testing: Camera publishing... "
CAMERA_RATE=$(timeout 2 ros2 topic hz /oak/rgb/image_raw 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "0")
if (( $(echo "$CAMERA_RATE > 25" | bc -l) )); then
    echo "✅ PASSED ($CAMERA_RATE Hz)"
    ((PASSED++))
else
    echo "❌ FAILED ($CAMERA_RATE Hz, expected >25)"
    ((FAILED++))
fi

echo -n "Testing: Perception publishing... "
PERCEPTION_RATE=$(timeout 2 ros2 topic hz /r2d2/perception/face_count 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "0")
if (( $(echo "$PERCEPTION_RATE > 10" | bc -l) )); then
    echo "✅ PASSED ($PERCEPTION_RATE Hz)"
    ((PASSED++))
else
    echo "❌ FAILED ($PERCEPTION_RATE Hz, expected >10)"
    ((FAILED++))
fi

echo -n "Testing: Status publishing... "
STATUS_RATE=$(timeout 2 ros2 topic hz /r2d2/audio/person_status 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "0")
if (( $(echo "$STATUS_RATE > 5" | bc -l) )); then
    echo "✅ PASSED ($STATUS_RATE Hz)"
    ((PASSED++))
else
    echo "❌ FAILED ($STATUS_RATE Hz, expected >5)"
    ((FAILED++))
fi

echo ""
echo "Phase 4: Recognition Pipeline"
echo "------------------------------"
TARGET=$(ros2 param get /audio_notification_node target_person 2>&1 | grep "String value is" | awk -F"'" '{print $2}' || echo "target_person")

echo -n "Testing: Face recognition enabled... "
if ros2 topic list | grep -q /r2d2/perception/person_id; then
    PERSON_RATE=$(timeout 2 ros2 topic hz /r2d2/perception/person_id 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "0")
    if (( $(echo "$PERSON_RATE > 0" | bc -l) )); then
        echo "✅ PASSED ($PERSON_RATE Hz)"
        ((PASSED++))
    else
        echo "⚠️  WARNING (no messages, recognition may be disabled)"
    fi
else
    echo "⚠️  WARNING (person_id topic not found)"
fi

echo ""
echo "Phase 5: End-to-End Test"
echo "------------------------"
echo "Stand in front of the camera for 10 seconds..."
echo ""

RECOGNIZED=false
for i in {1..10}; do
    PERSON_ID=$(timeout 1 ros2 topic echo /r2d2/perception/person_id --once 2>/dev/null | grep "data:" | awk '{print $2}' || echo "")
    if [ "$PERSON_ID" = "$TARGET" ]; then
        echo "  ✅ Target person recognized: $PERSON_ID"
        RECOGNIZED=true
        break
    fi
    sleep 1
done

if [ "$RECOGNIZED" = true ]; then
    ((PASSED++))
    echo ""
    echo "Checking status change..."
    sleep 2
    STATUS_JSON=$(timeout 1 ros2 topic echo /r2d2/audio/person_status --once 2>/dev/null | grep -o '{"[^}]*}' | head -1)
    if [ ! -z "$STATUS_JSON" ]; then
        STATUS=$(echo $STATUS_JSON | grep -o '"status":"[^"]*"' | cut -d'"' -f4)
        if [ "$STATUS" = "red" ]; then
            echo "  ✅ Status changed to RED (system working correctly)"
            ((PASSED++))
        else
            echo "  ⚠️  Status: $STATUS (expected RED)"
        fi
    fi
else
    echo "  ⚠️  Target person not recognized (check training/model)"
    ((FAILED++))
fi

echo ""
echo "========================================="
echo "📊 Test Summary"
echo "========================================="
echo "✅ Passed: $PASSED"
echo "❌ Failed: $FAILED"
echo "Total: $((PASSED + FAILED))"
echo ""

if [ $FAILED -eq 0 ]; then
    echo "🎉 All tests passed! System is fully operational."
    exit 0
else
    echo "⚠️  Some tests failed. Check the output above for details."
    exit 1
fi


