#!/bin/bash
# Comprehensive diagnostic and fix script for R2D2 Web Dashboard

echo "=========================================="
echo "R2D2 Web Dashboard Diagnostic & Fix"
echo "=========================================="
echo ""

# Check 1: Services Installation
echo "1. Checking service installation..."
if [ -f /etc/systemd/system/r2d2-heartbeat.service ]; then
    echo "   ✅ r2d2-heartbeat.service is installed"
else
    echo "   ❌ r2d2-heartbeat.service is NOT installed"
    NEEDS_INSTALL=true
fi

if [ -f /etc/systemd/system/r2d2-camera-stream.service ]; then
    echo "   ✅ r2d2-camera-stream.service is installed"
else
    echo "   ❌ r2d2-camera-stream.service is NOT installed"
    NEEDS_INSTALL=true
fi

# Check 2: Service Status
echo ""
echo "2. Checking service status..."
if systemctl is-active --quiet r2d2-heartbeat.service 2>/dev/null; then
    echo "   ✅ r2d2-heartbeat.service is RUNNING"
else
    echo "   ❌ r2d2-heartbeat.service is NOT running"
    NEEDS_START=true
fi

if systemctl is-active --quiet r2d2-camera-stream.service 2>/dev/null; then
    echo "   ✅ r2d2-camera-stream.service is RUNNING"
else
    echo "   ❌ r2d2-camera-stream.service is NOT running"
    NEEDS_START=true
fi

# Check 3: ROS 2 Nodes
echo ""
echo "3. Checking ROS 2 nodes..."
if ros2 node list 2>/dev/null | grep -q heartbeat; then
    echo "   ✅ heartbeat node is running"
else
    echo "   ❌ heartbeat node is NOT running"
fi

if ros2 node list 2>/dev/null | grep -q camera_stream; then
    echo "   ✅ camera_stream node is running"
else
    echo "   ❌ camera_stream node is NOT running"
fi

# Check 4: ROS 2 Topics
echo ""
echo "4. Checking ROS 2 topics..."
if timeout 2 ros2 topic echo /r2d2/heartbeat --once 2>&1 | grep -q "timestamp"; then
    echo "   ✅ /r2d2/heartbeat is publishing"
else
    echo "   ❌ /r2d2/heartbeat is NOT publishing"
fi

# Check 5: rosbridge
echo ""
echo "5. Checking rosbridge..."
if netstat -tulnp 2>/dev/null | grep -q ":9090" || ss -tulnp 2>/dev/null | grep -q ":9090"; then
    echo "   ✅ rosbridge is listening on port 9090"
else
    echo "   ❌ rosbridge is NOT running on port 9090"
fi

# Check 6: Web Server
echo ""
echo "6. Checking web server..."
if netstat -tulnp 2>/dev/null | grep -q ":8080" || ss -tulnp 2>/dev/null | grep -q ":8080"; then
    echo "   ✅ Web server is listening on port 8080"
else
    echo "   ❌ Web server is NOT running on port 8080"
fi

# Check 7: Camera Stream HTTP
echo ""
echo "7. Checking camera stream HTTP server..."
if curl -s http://localhost:8081/health 2>&1 | grep -q "OK"; then
    echo "   ✅ Camera stream HTTP server is responding"
else
    echo "   ❌ Camera stream HTTP server is NOT responding"
fi

echo ""
echo "=========================================="
echo "Summary & Actions Needed"
echo "=========================================="

if [ "$NEEDS_INSTALL" = true ]; then
    echo ""
    echo "⚠️  SERVICES NEED TO BE INSTALLED"
    echo "   Run: ./install_camera_and_heartbeat_services.sh"
    echo ""
fi

if [ "$NEEDS_START" = true ]; then
    echo ""
    echo "⚠️  SERVICES NEED TO BE STARTED"
    if [ "$NEEDS_INSTALL" = true ]; then
        echo "   First install, then run:"
    else
        echo "   Run:"
    fi
    echo "   sudo systemctl start r2d2-heartbeat.service"
    echo "   sudo systemctl start r2d2-camera-stream.service"
    echo ""
fi

echo ""
echo "To fix all issues automatically, run:"
echo "  ./install_camera_and_heartbeat_services.sh"
echo "  sudo systemctl start r2d2-heartbeat.service"
echo "  sudo systemctl start r2d2-camera-stream.service"
echo "  sudo systemctl enable r2d2-heartbeat.service"
echo "  sudo systemctl enable r2d2-camera-stream.service"
echo ""

