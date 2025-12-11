#!/bin/bash
# Finalize VPN Setup - Update client config and start service
# Run with: sudo bash finalize_setup.sh

set -e

echo "=== Finalizing WireGuard VPN Setup ==="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run as root (use sudo)"
    exit 1
fi

# Step 1: Update client config with public IP
echo "[1/3] Updating client config with public IP..."
PUBLIC_IP="188.61.209.189"
if grep -q "YOUR_PUBLIC_IP" /home/severin/dev/r2d2/vpn_config/client_wg0.conf; then
    sed -i "s/YOUR_PUBLIC_IP/$PUBLIC_IP/g" /home/severin/dev/r2d2/vpn_config/client_wg0.conf
    echo "   ✅ Client config updated with public IP: $PUBLIC_IP"
else
    echo "   ✅ Client config already has public IP"
fi

# Step 2: Enable and start service
echo "[2/3] Enabling and starting WireGuard service..."
systemctl enable wg-quick@wg0
systemctl start wg-quick@wg0
sleep 2
echo "   ✅ Service started"

# Step 3: Verify
echo "[3/3] Verifying service status..."
if systemctl is-active --quiet wg-quick@wg0; then
    echo "   ✅ WireGuard service is running!"
else
    echo "   ❌ Service failed to start"
    echo "   Checking logs..."
    journalctl -u wg-quick@wg0 -n 20 --no-pager
    exit 1
fi

echo ""
echo "=== Service Status ==="
systemctl status wg-quick@wg0 --no-pager -l | head -12

echo ""
echo "=== WireGuard Interface ==="
wg show

echo ""
echo "=== Setup Complete! ==="
echo ""
echo "✅ WireGuard VPN is running and ready!"
echo ""
echo "Your keys:"
echo "  Server Public Key: $(cat /etc/wireguard/server_public.key)"
echo "  Client Public Key: $(cat /etc/wireguard/client_public.key)"
echo ""
echo "Next steps:"
echo "1. View client config for Windows:"
echo "   sudo cat /home/severin/dev/r2d2/vpn_config/client_wg0.conf"
echo ""
echo "2. Store keys in KeePass:"
echo "   cat ~/.r2d2_vpn_secrets/vpn_keys.txt"
echo ""
echo "3. Configure router port forwarding:"
echo "   UDP 51820 -> 192.168.1.129:51820"
echo ""
echo "4. Transfer client_wg0.conf to Windows and import into WireGuard"
echo ""

