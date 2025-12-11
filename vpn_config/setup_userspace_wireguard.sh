#!/bin/bash
# Setup WireGuard to use userspace implementation (wireguard-go)
# This works on Jetson when kernel module is not available
# Run with: sudo bash setup_userspace_wireguard.sh

set -e

echo "=== Setting up WireGuard Userspace (wireguard-go) ==="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run as root (use sudo)"
    exit 1
fi

echo "[1/3] Installing wireguard-go..."
apt update
apt install -y wireguard-go

echo "[2/3] Creating systemd service override..."
mkdir -p /etc/systemd/system/wg-quick@.service.d/

cat > /etc/systemd/system/wg-quick@.service.d/override.conf <<'EOF'
[Service]
# Force use of wireguard-go (userspace) instead of kernel module
Environment="WG_QUICK_USERSPACE_IMPLEMENTATION=wireguard-go"
EOF

echo "   ✅ Service override created"

echo "[3/3] Reloading systemd..."
systemctl daemon-reload

echo ""
echo "✅ Setup complete!"
echo ""
echo "Now try starting the service:"
echo "   sudo systemctl start wg-quick@wg0"
echo ""
echo "If it still fails, check logs:"
echo "   sudo journalctl -u wg-quick@wg0 -n 30"
echo ""

