#!/bin/bash
# Build WireGuard kernel module for Jetson
# Run with: sudo bash build_wireguard_module.sh

set -e

echo "=== Building WireGuard Kernel Module for Jetson ==="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run as root (use sudo)"
    exit 1
fi

KERNEL_VERSION=$(uname -r)
echo "Kernel version: $KERNEL_VERSION"

echo "[1/4] Installing required packages..."
apt update
apt install -y linux-headers-${KERNEL_VERSION} dkms build-essential

echo "[2/4] Installing wireguard-dkms..."
if apt install -y wireguard-dkms; then
    echo "   ✅ wireguard-dkms installed"
else
    echo "   ❌ wireguard-dkms installation failed"
    exit 1
fi

echo "[3/4] Building kernel module..."
# Check DKMS status
dkms status wireguard

# Try to build
if dkms build wireguard -v $(dpkg -l | grep wireguard-dkms | awk '{print $3}' | cut -d'-' -f1) 2>&1; then
    echo "   ✅ Module built successfully"
else
    echo "   ⚠️  Build had issues, checking status..."
    dkms status
fi

echo "[4/4] Installing and loading module..."
if dkms install wireguard -v $(dpkg -l | grep wireguard-dkms | awk '{print $3}' | cut -d'-' -f1) 2>&1; then
    echo "   ✅ Module installed"
else
    echo "   ⚠️  Install had issues"
    dkms status
fi

# Try to load
modprobe wireguard

if lsmod | grep -q wireguard; then
    echo ""
    echo "✅ SUCCESS! WireGuard kernel module is loaded!"
    echo ""
    echo "Now try starting the service:"
    echo "   sudo systemctl start wg-quick@wg0"
    echo "   sudo systemctl status wg-quick@wg0"
else
    echo ""
    echo "⚠️  Module built but not loaded"
    echo "Try:"
    echo "1. Reboot: sudo reboot"
    echo "2. After reboot: sudo modprobe wireguard"
    echo "3. Then: sudo systemctl start wg-quick@wg0"
fi

echo ""

