#!/bin/bash
# Fix WireGuard kernel module issue on Jetson
# Run with: sudo bash fix_kernel_module.sh

set -e

echo "=== Fixing WireGuard Kernel Module Issue ==="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run as root (use sudo)"
    exit 1
fi

echo "[1/4] Checking current WireGuard installation..."
wg --version

echo "[2/4] Attempting to install wireguard-dkms (kernel module)..."
apt update
if apt install -y wireguard-dkms 2>&1 | tee /tmp/wg-dkms-install.log; then
    echo "   ✅ wireguard-dkms installed"
    echo "[3/4] Loading WireGuard kernel module..."
    modprobe wireguard
    if lsmod | grep -q wireguard; then
        echo "   ✅ WireGuard kernel module loaded!"
    else
        echo "   ⚠️  Module installed but not loading - may need reboot"
    fi
else
    echo "   ❌ wireguard-dkms installation failed or not available"
    echo ""
    echo "[3/4] Trying alternative: Check if kernel supports WireGuard..."
    if grep -q "CONFIG_WIREGUARD" /boot/config-$(uname -r) 2>/dev/null; then
        echo "   ✅ Kernel config shows WireGuard support"
        echo "   Trying to load module..."
        modprobe wireguard || echo "   ⚠️  Module still not available"
    else
        echo "   ❌ Kernel doesn't have WireGuard support built-in"
        echo ""
        echo "[4/4] Solution: Use userspace WireGuard (wireguard-go)"
        echo "   Installing wireguard-go..."
        if apt install -y wireguard-go 2>&1; then
            echo "   ✅ wireguard-go installed"
            echo ""
            echo "   ⚠️  NOTE: You'll need to modify the service to use wireguard-go"
            echo "   This requires updating the systemd service file"
        else
            echo "   ❌ wireguard-go not available in repositories"
            echo ""
            echo "   Alternative: Build WireGuard kernel module from source"
            echo "   This is complex on Jetson - may require kernel headers"
        fi
    fi
fi

echo ""
echo "=== Current Status ==="
if lsmod | grep -q wireguard; then
    echo "✅ WireGuard kernel module is loaded"
    echo "   Try starting service again: sudo systemctl start wg-quick@wg0"
else
    echo "❌ WireGuard kernel module is NOT loaded"
    echo ""
    echo "Options:"
    echo "1. Reboot the system (may load module automatically)"
    echo "2. Check if module needs to be built: dkms status"
    echo "3. Use userspace implementation (wireguard-go) - requires service modification"
fi

