#!/bin/bash
# Check WireGuard service error
# Run with: sudo bash check_error.sh

echo "=== WireGuard Service Error Diagnosis ==="
echo ""

echo "Service Status:"
systemctl status wg-quick@wg0.service --no-pager -l | head -20

echo ""
echo "Recent Logs:"
journalctl -xeu wg-quick@wg0.service --no-pager -n 30

echo ""
echo "Checking configuration file:"
if [ -f /etc/wireguard/wg0.conf ]; then
    echo "✅ Config file exists"
    echo "First few lines:"
    head -10 /etc/wireguard/wg0.conf
else
    echo "❌ Config file missing!"
fi

echo ""
echo "Checking for syntax errors:"
wg-quick up wg0 2>&1 || true

