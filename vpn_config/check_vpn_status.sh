#!/bin/bash
# Check WireGuard VPN status

echo "=== WireGuard VPN Status ==="
echo ""

# Check if WireGuard is installed
if ! command -v wg &> /dev/null; then
    echo "❌ WireGuard is not installed"
    echo "   Run: sudo bash setup_wireguard.sh"
    exit 1
fi

# Check if service is running
if systemctl is-active --quiet wg-quick@wg0; then
    echo "✅ WireGuard service is running"
else
    echo "❌ WireGuard service is not running"
    echo "   Start with: sudo systemctl start wg-quick@wg0"
fi

echo ""
echo "=== Interface Status ==="
if ip link show wg0 &> /dev/null; then
    wg show
    echo ""
    echo "=== Interface Details ==="
    ip addr show wg0
else
    echo "❌ wg0 interface not found"
    echo "   Start with: sudo systemctl start wg-quick@wg0"
fi

echo ""
echo "=== Connection Test ==="
if systemctl is-active --quiet wg-quick@wg0; then
    PEER_COUNT=$(wg show wg0 peers | wc -l)
    if [ "$PEER_COUNT" -gt 0 ]; then
        echo "✅ $PEER_COUNT peer(s) connected"
        wg show wg0 | grep -A 5 "peer:"
    else
        echo "⚠️  No peers connected yet"
        echo "   Make sure client is connected and router port forwarding is configured"
    fi
else
    echo "❌ Cannot check peers - service not running"
fi

