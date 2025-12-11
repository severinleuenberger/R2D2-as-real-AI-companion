#!/bin/bash
# Complete VPN Setup - Run after setup_wireguard.sh
# This script completes the setup: updates client config, starts service, verifies

set -e

echo "=== Completing WireGuard VPN Setup ==="
echo ""

# Step 1: Check if setup was run
if [ ! -f /home/severin/dev/r2d2/vpn_config/client_wg0.conf ]; then
    echo "❌ ERROR: client_wg0.conf not found!"
    echo "   Please run: sudo bash setup_wireguard.sh first"
    exit 1
fi

# Step 2: Get public IP
echo "[1/5] Getting public IP address..."
PUBLIC_IP=$(curl -4 -s ifconfig.me || curl -4 -s icanhazip.com)
if [ -z "$PUBLIC_IP" ]; then
    echo "⚠️  Could not get public IP automatically"
    read -p "Enter your public IP address: " PUBLIC_IP
fi
echo "   Public IP: $PUBLIC_IP"

# Step 3: Update client config with public IP
echo "[2/5] Updating client config with public IP..."
sudo sed -i "s/YOUR_PUBLIC_IP/$PUBLIC_IP/g" /home/severin/dev/r2d2/vpn_config/client_wg0.conf
echo "   ✅ Client config updated"

# Step 4: Verify client config
echo "[3/5] Verifying client config..."
if sudo grep -q "YOUR_PUBLIC_IP" /home/severin/dev/r2d2/vpn_config/client_wg0.conf; then
    echo "   ⚠️  WARNING: YOUR_PUBLIC_IP still found in config!"
    echo "   Please update manually: sudo nano /home/severin/dev/r2d2/vpn_config/client_wg0.conf"
else
    echo "   ✅ Client config looks good"
fi

# Step 5: Start WireGuard service
echo "[4/5] Starting WireGuard service..."
sudo systemctl enable wg-quick@wg0
sudo systemctl start wg-quick@wg0
sleep 2

# Step 6: Verify service is running
echo "[5/5] Verifying service status..."
if sudo systemctl is-active --quiet wg-quick@wg0; then
    echo "   ✅ WireGuard service is running"
else
    echo "   ❌ WireGuard service failed to start"
    echo "   Check logs: sudo journalctl -u wg-quick@wg0 -n 20"
    exit 1
fi

echo ""
echo "=== Setup Complete! ==="
echo ""
echo "✅ WireGuard service is running"
echo "✅ Client config updated with public IP: $PUBLIC_IP"
echo ""
echo "Next steps:"
echo "1. Configure router port forwarding: UDP 51820 -> 192.168.1.129:51820"
echo "2. View client config: sudo cat /home/severin/dev/r2d2/vpn_config/client_wg0.conf"
echo "3. Transfer client_wg0.conf to Windows laptop"
echo "4. Import into WireGuard client on Windows"
echo ""
echo "Check status: bash check_vpn_status.sh"
echo "View config: sudo cat /home/severin/dev/r2d2/vpn_config/client_wg0.conf"
echo ""

