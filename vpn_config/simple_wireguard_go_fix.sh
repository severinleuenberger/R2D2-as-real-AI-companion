#!/bin/bash
# Simple fix: Use wireguard binary directly with TUN interface
# Run with: sudo bash simple_wireguard_go_fix.sh

set -e

echo "=== Simple WireGuard-Go Fix ==="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run as root (use sudo)"
    exit 1
fi

echo "[1/4] Checking TUN module..."
modprobe tun
if lsmod | grep -q "^tun"; then
    echo "   ✅ TUN module loaded"
else
    echo "   ⚠️  TUN module not available"
fi

echo "[2/4] Creating custom systemd service..."
cat > /etc/systemd/system/wireguard-wg0.service <<'SERVICEEOF'
[Unit]
Description=WireGuard VPN for wg0 (userspace)
After=network.target
Before=network-online.target
Wants=network-online.target

[Service]
Type=simple
ExecStartPre=/sbin/modprobe tun
ExecStart=/usr/bin/wireguard wg0
ExecStartPost=/bin/sleep 2
ExecStartPost=/usr/bin/wg setconf wg0 /etc/wireguard/wg0.conf
ExecStartPost=/bin/bash -c 'IP=$(grep "^Address" /etc/wireguard/wg0.conf | head -1 | awk "{print \$3}" | cut -d"/" -f1); if [ -n "$IP" ]; then /sbin/ip addr add "$IP/24" dev wg0 2>/dev/null; /sbin/ip link set wg0 up; fi'
ExecStop=/bin/bash -c '/usr/bin/wg-quick down wg0 2>/dev/null || /sbin/ip link delete wg0 2>/dev/null || true'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
SERVICEEOF

echo "   ✅ Service file created"

echo "[3/4] Disabling old service..."
systemctl disable wg-quick@wg0 2>/dev/null || true
systemctl stop wg-quick@wg0 2>/dev/null || true

echo "[4/4] Enabling new service..."
systemctl daemon-reload
systemctl enable wireguard-wg0.service

echo ""
echo "✅ Setup complete!"
echo ""
echo "Now try starting:"
echo "   sudo systemctl start wireguard-wg0"
echo "   sudo systemctl status wireguard-wg0"
echo ""
echo "If it works, check interface:"
echo "   sudo wg show"
echo "   ip addr show wg0"
echo ""

