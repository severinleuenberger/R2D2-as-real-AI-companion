#!/bin/bash
# Start WireGuard service and verify setup
# Run with: sudo bash start_and_verify.sh

set -e

echo "=== Starting WireGuard VPN Service ==="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run as root (use sudo)"
    exit 1
fi

# Step 1: Enable service
echo "[1/3] Enabling WireGuard service..."
systemctl enable wg-quick@wg0
echo "   ✅ Service enabled"

# Step 2: Start service
echo "[2/3] Starting WireGuard service..."
systemctl start wg-quick@wg0
sleep 2
echo "   ✅ Service started"

# Step 3: Verify status
echo "[3/3] Verifying service status..."
if systemctl is-active --quiet wg-quick@wg0; then
    echo "   ✅ WireGuard service is running"
else
    echo "   ❌ Service failed to start"
    echo "   Checking logs..."
    journalctl -u wg-quick@wg0 -n 20 --no-pager
    exit 1
fi

echo ""
echo "=== Service Status ==="
systemctl status wg-quick@wg0 --no-pager -l | head -10

echo ""
echo "=== WireGuard Interface ==="
wg show

echo ""
echo "=== Setup Complete! ==="
echo ""
echo "✅ WireGuard service is running"
echo "✅ Ready for client connections"
echo ""
echo "Next steps:"
echo "1. Configure router port forwarding: UDP 51820 -> 192.168.1.129:51820"
echo "2. View client config: cat /home/severin/dev/r2d2/vpn_config/client_wg0.conf"
echo "3. Transfer client_wg0.conf to Windows laptop"
echo "4. Import into WireGuard client on Windows"
echo ""

