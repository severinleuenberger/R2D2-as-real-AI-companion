#!/bin/bash
################################################################################
# R2D2 Complete Service Restart
# Simulates reboot by restarting all services in correct dependency order
################################################################################

echo "╔═══════════════════════════════════════════════════════════╗"
echo "║         RESTARTING ALL R2D2 SERVICES                 ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""

# Step 1: Stop all services (reverse order)
echo "Step 1: Stopping all services..."
sudo systemctl stop r2d2-gesture-intent
sudo systemctl stop r2d2-speech-node
sudo systemctl stop r2d2-camera-perception
sudo systemctl stop r2d2-audio-notification
sleep 2
echo "✅ All services stopped"
echo ""

# Step 2: Start in dependency order (like boot)
echo "Step 2: Starting services in boot order..."

echo "  Starting r2d2-audio-notification..."
sudo systemctl start r2d2-audio-notification
sleep 2

echo "  Starting r2d2-camera-perception..."
sudo systemctl start r2d2-camera-perception
sleep 3

echo "  Starting r2d2-gesture-intent..."
sudo systemctl start r2d2-gesture-intent
sleep 1

echo "  Starting r2d2-speech-node..."
sudo systemctl start r2d2-speech-node
sleep 2

echo "✅ All services started"
echo ""

# Step 3: Verify
echo "Step 3: Verifying services..."
for service in r2d2-audio-notification r2d2-camera-perception r2d2-gesture-intent r2d2-speech-node; do
    status=$(systemctl is-active $service)
    if [ "$status" = "active" ]; then
        echo "  ✅ $service: $status"
    else
        echo "  ❌ $service: $status"
    fi
done

echo ""
echo "Step 4: Waiting for nodes to initialize (10 seconds)..."
sleep 10

echo ""
echo "Step 5: Checking ROS2 nodes..."
cd ~/dev/r2d2/ros2_ws && source install/setup.bash
node_count=$(ros2 node list | grep -E "audio|gesture|speech|image|camera" | wc -l)
echo "  Found $node_count critical nodes"

if [ "$node_count" -ge 5 ]; then
    echo "  ✅ All nodes running"
else
    echo "  ⚠️  Some nodes missing"
    ros2 node list | grep -E "audio|gesture|speech|image|camera"
fi

echo ""
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║            RESTART COMPLETE                          ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""
echo "System should now behave like fresh boot."
echo "Test: Walk away for 20s, come back, should hear 'Hello!' beep"

