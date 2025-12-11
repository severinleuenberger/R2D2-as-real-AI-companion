#!/bin/bash
# WireGuard Complete Removal Script
# Run with: sudo bash remove_wireguard.sh
#
# ⚠️  WARNING: This will permanently delete all WireGuard files and keys!
# Make sure you've backed up any keys you want to keep in KeePass!

set -e

echo "=== WireGuard Complete Removal ==="
echo ""
echo "⚠️  WARNING: This will permanently delete all WireGuard files and keys!"
echo "   - All WireGuard packages will be removed"
echo "   - All configuration files will be deleted"
echo "   - All key files will be deleted (cannot be recovered!)"
echo "   - Systemd services will be removed"
echo ""
read -p "Are you sure? Type 'yes' to continue: " confirm

if [ "$confirm" != "yes" ]; then
    echo "Aborted."
    exit 1
fi

echo ""
echo "[1/7] Stopping and disabling WireGuard services..."
systemctl stop wg-quick@wg0 2>/dev/null || true
systemctl disable wg-quick@wg0 2>/dev/null || true
rm -rf /etc/systemd/system/wg-quick@.service.d/ 2>/dev/null || true
rm -f /etc/systemd/system/multi-user.target.wants/wg-quick@wg0.service 2>/dev/null || true
systemctl daemon-reload
echo "   ✅ Services stopped and disabled"

echo "[2/7] Removing WireGuard packages..."
apt remove --purge -y wireguard wireguard-tools wireguard-go wireguard-dkms 2>/dev/null || true
apt autoremove -y 2>/dev/null || true
echo "   ✅ Packages removed"

echo "[3/7] Removing server configuration and keys..."
rm -rf /etc/wireguard/ 2>/dev/null || true
echo "   ✅ Server configuration removed"

echo "[4/7] Removing client configuration and keys..."
rm -f /home/severin/dev/r2d2/vpn_config/client_wg0.conf 2>/dev/null || true
rm -f /home/severin/dev/r2d2/vpn_config/*.key 2>/dev/null || true
rm -f /home/severin/dev/r2d2/vpn_config/server_private.key 2>/dev/null || true
rm -f /home/severin/dev/r2d2/vpn_config/server_public.key 2>/dev/null || true
rm -f /home/severin/dev/r2d2/vpn_config/client_private.key 2>/dev/null || true
rm -f /home/severin/dev/r2d2/vpn_config/client_public.key 2>/dev/null || true
echo "   ✅ Client configuration removed"

echo "[5/7] Removing secure storage directory..."
rm -rf ~/.r2d2_vpn_secrets/ 2>/dev/null || true
echo "   ✅ Secure storage removed"

echo "[6/7] Cleaning up IP forwarding (skipped - may be needed for other services)..."
# Note: We're NOT removing IP forwarding as it might be needed for other services
# If you're sure it was only for WireGuard, you can manually edit /etc/sysctl.conf
echo "   ⚠️  IP forwarding left in place (check manually if needed)"

echo "[7/7] Verifying removal..."
echo ""
echo "Checking for remaining WireGuard components..."

# Check packages
if dpkg -l | grep -qi wireguard; then
    echo "   ⚠️  Warning: Some WireGuard packages may still be installed:"
    dpkg -l | grep -i wireguard
else
    echo "   ✅ All WireGuard packages removed"
fi

# Check services
if systemctl list-units | grep -qi wireguard; then
    echo "   ⚠️  Warning: Some WireGuard services may still exist:"
    systemctl list-units | grep -i wireguard
else
    echo "   ✅ All WireGuard services removed"
fi

# Check config directory
if [ -d /etc/wireguard ]; then
    echo "   ⚠️  Warning: /etc/wireguard directory still exists"
    ls -la /etc/wireguard/
else
    echo "   ✅ /etc/wireguard directory removed"
fi

# Check key files
if find /home/severin/dev/r2d2/vpn_config -name "*.key" -o -name "client_wg0.conf" 2>/dev/null | grep -q .; then
    echo "   ⚠️  Warning: Some key/config files may still exist:"
    find /home/severin/dev/r2d2/vpn_config -name "*.key" -o -name "client_wg0.conf" 2>/dev/null
else
    echo "   ✅ All key files removed"
fi

# Check secure storage
if [ -d ~/.r2d2_vpn_secrets ]; then
    echo "   ⚠️  Warning: Secure storage directory still exists"
    ls -la ~/.r2d2_vpn_secrets/
else
    echo "   ✅ Secure storage directory removed"
fi

echo ""
echo "=== Removal Complete ==="
echo ""
echo "✅ WireGuard has been removed"
echo "✅ All configuration files deleted"
echo "✅ All keys deleted"
echo ""
echo "⚠️  REMINDER: Make sure you backed up any keys you wanted to keep in KeePass!"
echo ""
echo "Verify Tailscale still works:"
echo "   tailscale status"
echo "   ssh jetson-tailscale  # (from Windows)"
echo ""

