#!/bin/bash
# Test script for camera and perception pipeline
# Verifies camera is publishing frames and perception is processing them

echo "📷 R2D2 Camera & Perception Pipeline Test"
echo "=========================================="
echo ""

# Source ROS 2 environment
source /opt/ros/humble/setup.bash 2>/dev/null
if [ -f ~/dev/r2d2/ros2_ws/install/setup.bash ]; then
    source ~/dev/r2d2/ros2_ws/install/setup.bash
fi

# Test 1: Check camera node
echo "Test 1: Camera Node Status"
if ros2 node list 2>/dev/null | grep -q camera_node; then
    echo "  ✅ Camera node is running"
else
    echo "  ❌ Camera node is NOT running"
    echo "     Start with: ros2 launch r2d2_bringup r2d2_camera_perception.launch.py"
    exit 1
fi

# Test 2: Check camera topic
echo ""
echo "Test 2: Camera Topic (/oak/rgb/image_raw)"
if ros2 topic list 2>/dev/null | grep -q "/oak/rgb/image_raw"; then
    echo "  ✅ Camera topic exists"
    
    # Check publication rate
    echo "  Checking publication rate (should be ~30 Hz)..."
    RATE=$(timeout 3 ros2 topic hz /oak/rgb/image_raw 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "0")
    if (( $(echo "$RATE > 25" | bc -l) )); then
        echo "  ✅ Rate: $RATE Hz (GOOD)"
    elif (( $(echo "$RATE > 0" | bc -l) )); then
        echo "  ⚠️  Rate: $RATE Hz (LOW - expected ~30 Hz)"
    else
        echo "  ❌ No messages received"
    fi
else
    echo "  ❌ Camera topic not found"
    exit 1
fi

# Test 3: Check perception node
echo ""
echo "Test 3: Perception Node Status"
if ros2 node list 2>/dev/null | grep -q image_listener; then
    echo "  ✅ Perception node is running"
else
    echo "  ❌ Perception node is NOT running"
    exit 1
fi

# Test 4: Check perception topics
echo ""
echo "Test 4: Perception Topics"
TOPICS=("/r2d2/perception/brightness" "/r2d2/perception/face_count")
for topic in "${TOPICS[@]}"; do
    if ros2 topic list 2>/dev/null | grep -q "$topic"; then
        echo "  ✅ $topic exists"
        
        # Check rate
        RATE=$(timeout 2 ros2 topic hz $topic 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "0")
        if (( $(echo "$RATE > 10" | bc -l) )); then
            echo "     Rate: $RATE Hz (GOOD)"
        else
            echo "     Rate: $RATE Hz (LOW)"
        fi
        
        # Get sample value
        VALUE=$(timeout 1 ros2 topic echo $topic --once 2>/dev/null | grep "data:" | awk '{print $2}' || echo "N/A")
        echo "     Sample: $VALUE"
    else
        echo "  ❌ $topic not found"
    fi
done

# Test 5: Face detection test
echo ""
echo "Test 5: Face Detection"
echo "  Stand in front of the camera for 5 seconds..."
FACE_COUNT=0
for i in {1..5}; do
    COUNT=$(timeout 1 ros2 topic echo /r2d2/perception/face_count --once 2>/dev/null | grep "data:" | awk '{print $2}' || echo "0")
    if [ "$COUNT" != "0" ] && [ "$COUNT" != "N/A" ]; then
        FACE_COUNT=$COUNT
        echo "  ✅ Face detected! Count: $COUNT"
        break
    fi
    sleep 1
done

if [ "$FACE_COUNT" = "0" ]; then
    echo "  ⚠️  No faces detected (make sure you're in front of the camera)"
fi

echo ""
echo "=========================================="
echo "✅ Camera & Perception Pipeline Test Complete"
echo ""


