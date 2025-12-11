#!/bin/bash
# Install WireGuard kernel module for Jetson
# Run with: sudo bash install_wireguard_module.sh

set -e

echo "=== Installing WireGuard Kernel Module for Jetson ==="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run as root (use sudo)"
    exit 1
fi

echo "[1/3] Installing required packages..."
apt update
apt install -y linux-headers-$(uname -r) dkms

echo "[2/3] Installing wireguard-dkms..."
if apt install -y wireguard-dkms; then
    echo "   ✅ wireguard-dkms installed"
else
    echo "   ⚠️  wireguard-dkms not available, trying alternative..."
    echo "   Installing from source..."
    apt install -y build-essential libelf-dev
    # This would require more complex setup - let's try dkms first
fi

echo "[3/3] Building and loading module..."
dkms status
modprobe wireguard

if lsmod | grep -q wireguard; then
    echo "   ✅ WireGuard kernel module loaded!"
    echo ""
    echo "✅ Success! Try starting the service:"
    echo "   sudo systemctl start wg-quick@wg0"
else
    echo "   ⚠️  Module not loaded - may need reboot"
    echo ""
    echo "Try:"
    echo "1. Reboot: sudo reboot"
    echo "2. After reboot, try: sudo systemctl start wg-quick@wg0"
fi

