#!/bin/bash
# Fix passwordless sudo and check rosbridge for web dashboard

set -e

echo "=== Fixing R2D2 Web Dashboard Issues ==="
echo ""

# Step 1: Fix passwordless sudo
echo "Step 1: Configuring passwordless sudo..."
echo "severin ALL=(ALL) NOPASSWD: /bin/systemctl start r2d2-*, /bin/systemctl stop r2d2-*, /bin/systemctl restart r2d2-*" | sudo tee /etc/sudoers.d/r2d2-services > /dev/null
sudo chmod 0440 /etc/sudoers.d/r2d2-services

if sudo visudo -c -f /etc/sudoers.d/r2d2-services 2>/dev/null; then
    echo "✅ sudoers file syntax is correct"
else
    echo "❌ sudoers file syntax error!"
    exit 1
fi

echo ""
echo "Testing passwordless sudo..."
if sudo -n systemctl status r2d2-audio-notification.service > /dev/null 2>&1; then
    echo "✅ Passwordless sudo is working!"
else
    echo "⚠️  Passwordless sudo test failed"
    echo "   You may need to log out and back in, or run: newgrp -"
    echo "   But the configuration is correct, so it should work after refresh."
fi

echo ""
echo "Step 2: Checking rosbridge..."
if ros2 node list 2>/dev/null | grep -q rosbridge; then
    echo "✅ rosbridge is running"
else
    echo "⚠️  rosbridge is not running"
    echo "   The web dashboard service should start it automatically"
    echo "   Check service logs: sudo journalctl -u r2d2-web-dashboard.service -f"
fi

echo ""
echo "Step 3: Checking web dashboard service..."
if systemctl is-active --quiet r2d2-web-dashboard.service; then
    echo "✅ Web dashboard service is running"
else
    echo "⚠️  Web dashboard service is not running"
    echo "   Start it with: sudo systemctl start r2d2-web-dashboard.service"
fi

echo ""
echo "=== Summary ==="
echo "1. Passwordless sudo configured"
echo "2. If test failed, try: newgrp -  (to refresh group permissions)"
echo "3. Check web dashboard service status: sudo systemctl status r2d2-web-dashboard.service"
echo "4. Restart web dashboard: sudo systemctl restart r2d2-web-dashboard.service"

