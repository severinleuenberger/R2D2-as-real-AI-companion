#!/bin/bash
# Use wireguard-go (userspace) instead of kernel module
# Run with: sudo bash use_wireguard_go.sh

set -e

echo "=== Setting up WireGuard with userspace implementation ==="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run as root (use sudo)"
    exit 1
fi

echo "[1/3] Installing wireguard-go..."
apt update
if apt install -y wireguard-go; then
    echo "   ✅ wireguard-go installed"
else
    echo "   ❌ wireguard-go not available in repositories"
    echo "   Trying to install from source or alternative..."
    exit 1
fi

echo "[2/3] Creating systemd service override..."
# Create override directory
mkdir -p /etc/systemd/system/wg-quick@.service.d/

# Create override file to use wireguard-go
cat > /etc/systemd/system/wg-quick@.service.d/override.conf <<'EOF'
[Service]
# Use wireguard-go instead of kernel module
ExecStart=
ExecStart=/usr/bin/wg-quick up %i
Environment="WG_QUICK_USERSPACE_IMPLEMENTATION=wireguard-go"
EOF

echo "   ✅ Service override created"

echo "[3/3] Reloading systemd and testing..."
systemctl daemon-reload

echo ""
echo "✅ Setup complete!"
echo ""
echo "Now try starting the service:"
echo "   sudo systemctl start wg-quick@wg0"
echo "   sudo systemctl status wg-quick@wg0"
echo ""

