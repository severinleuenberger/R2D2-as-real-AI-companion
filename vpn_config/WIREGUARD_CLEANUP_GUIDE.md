# WireGuard Complete Removal Guide

**Date:** December 11, 2025  
**Purpose:** Complete removal of WireGuard installation and all related files  
**Status:** Pre-removal checklist

---

## ⚠️ IMPORTANT: Before Removal

**Read this entire guide before starting!**

This guide will help you completely remove WireGuard and all related files. Since we're using Tailscale instead, WireGuard is no longer needed.

---

## What Will Be Removed

### Packages
- `wireguard` (metapackage)
- `wireguard-tools` (userland utilities)
- `wireguard-go` (userspace implementation)
- `wireguard-dkms` (if installed)

### Configuration Files
- `/etc/wireguard/wg0.conf` (server configuration)
- `/etc/wireguard/*.key` (all key files)
- `/home/severin/dev/r2d2/vpn_config/client_wg0.conf` (client configuration)
- `/home/severin/dev/r2d2/vpn_config/*.key` (any key files in vpn_config)

### System Services
- `wg-quick@wg0.service` (systemd service)
- Any WireGuard systemd overrides

### System Configuration
- IP forwarding settings (if only added for WireGuard)
- Firewall rules (if added for WireGuard)

### Secure Storage
- `~/.r2d2_vpn_secrets/` (if exists, contains sensitive keys)

---

## Pre-Removal Checklist

Before removing, verify:

- [ ] Tailscale is working and connected
- [ ] SSH connection works through Tailscale
- [ ] VS Code Remote SSH works through Tailscale
- [ ] You have backed up any keys you want to keep (in KeePass)
- [ ] You understand that WireGuard keys will be permanently deleted

---

## Step-by-Step Removal

### Step 1: Stop and Disable WireGuard Services

```bash
# Stop WireGuard service (if running)
sudo systemctl stop wg-quick@wg0

# Disable service
sudo systemctl disable wg-quick@wg0

# Remove any systemd overrides
sudo rm -rf /etc/systemd/system/wg-quick@.service.d/

# Reload systemd
sudo systemctl daemon-reload
```

### Step 2: Remove WireGuard Packages

```bash
# Remove all WireGuard packages
sudo apt remove --purge wireguard wireguard-tools wireguard-go wireguard-dkms

# Remove any leftover dependencies (if not needed)
sudo apt autoremove
```

### Step 3: Remove Configuration Files

**⚠️ WARNING: These files contain cryptographic keys. Once deleted, they cannot be recovered!**

```bash
# Remove server configuration and keys
sudo rm -rf /etc/wireguard/

# Remove client configuration (contains private key!)
rm /home/severin/dev/r2d2/vpn_config/client_wg0.conf

# Remove any key files in vpn_config
rm /home/severin/dev/r2d2/vpn_config/*.key
```

### Step 4: Remove Secure Storage Directory

**⚠️ WARNING: This contains all your WireGuard keys. Make sure you've stored them in KeePass if needed!**

```bash
# Remove secure storage directory
rm -rf ~/.r2d2_vpn_secrets/
```

### Step 5: Clean Up System Configuration

**Check if IP forwarding was only added for WireGuard:**

```bash
# Check sysctl.conf
grep "net.ipv4.ip_forward" /etc/sysctl.conf
```

**If you see:**
```
net.ipv4.ip_forward=1
```

**And you don't need IP forwarding for other services, you can remove it:**

```bash
# Edit sysctl.conf
sudo nano /etc/sysctl.conf

# Remove or comment out the line:
# net.ipv4.ip_forward=1

# Apply changes
sudo sysctl -p
```

**Note:** If you're using other services that need IP forwarding, keep this setting.

### Step 6: Remove Firewall Rules (if added)

**Check for WireGuard firewall rules:**

```bash
# If using UFW
sudo ufw status | grep -i wireguard

# If using iptables directly
sudo iptables -L -n | grep -i wireguard
```

**Remove WireGuard-specific rules:**

```bash
# If you added UFW rules for WireGuard port 51820
sudo ufw delete allow 51820/udp

# If you added iptables rules, you'll need to remove them manually
# Check what was added and remove accordingly
```

### Step 7: Verify Removal

**Check that everything is removed:**

```bash
# Check for WireGuard packages
dpkg -l | grep -i wireguard
# Should show nothing

# Check for WireGuard services
systemctl list-units | grep -i wireguard
# Should show nothing (or only failed/stopped services)

# Check for configuration files
ls -la /etc/wireguard/ 2>/dev/null
# Should show "No such file or directory"

# Check for key files
find /home/severin/dev/r2d2/vpn_config -name "*.key" -o -name "client_wg0.conf"
# Should show nothing

# Check for secure storage
ls -la ~/.r2d2_vpn_secrets/ 2>/dev/null
# Should show "No such file or directory"
```

---

## Automated Removal Script

**⚠️ USE WITH CAUTION! This will permanently delete all WireGuard files and keys!**

Create and run this script to automate the removal:

```bash
#!/bin/bash
# WireGuard Complete Removal Script
# Run with: sudo bash remove_wireguard.sh

set -e

echo "=== WireGuard Complete Removal ==="
echo ""
echo "⚠️  WARNING: This will permanently delete all WireGuard files and keys!"
read -p "Are you sure? Type 'yes' to continue: " confirm

if [ "$confirm" != "yes" ]; then
    echo "Aborted."
    exit 1
fi

echo ""
echo "[1/7] Stopping services..."
systemctl stop wg-quick@wg0 2>/dev/null || true
systemctl disable wg-quick@wg0 2>/dev/null || true
rm -rf /etc/systemd/system/wg-quick@.service.d/ 2>/dev/null || true
systemctl daemon-reload

echo "[2/7] Removing packages..."
apt remove --purge -y wireguard wireguard-tools wireguard-go wireguard-dkms 2>/dev/null || true
apt autoremove -y

echo "[3/7] Removing server configuration..."
rm -rf /etc/wireguard/

echo "[4/7] Removing client configuration..."
rm -f /home/severin/dev/r2d2/vpn_config/client_wg0.conf
rm -f /home/severin/dev/r2d2/vpn_config/*.key

echo "[5/7] Removing secure storage..."
rm -rf ~/.r2d2_vpn_secrets/

echo "[6/7] Cleaning up IP forwarding (if only for WireGuard)..."
# Note: This is commented out - uncomment if you want to remove IP forwarding
# sed -i '/net.ipv4.ip_forward=1/d' /etc/sysctl.conf
# sysctl -p

echo "[7/7] Verifying removal..."
echo ""
echo "Checking for remaining WireGuard files..."
if dpkg -l | grep -qi wireguard; then
    echo "⚠️  Warning: Some WireGuard packages may still be installed"
else
    echo "✅ All WireGuard packages removed"
fi

if [ -d /etc/wireguard ]; then
    echo "⚠️  Warning: /etc/wireguard directory still exists"
else
    echo "✅ /etc/wireguard directory removed"
fi

echo ""
echo "=== Removal Complete ==="
echo ""
echo "✅ WireGuard has been removed"
echo "✅ All configuration files deleted"
echo "✅ All keys deleted (make sure you backed them up in KeePass!)"
echo ""
```

**Save this as:** `remove_wireguard.sh`

**Run with:**
```bash
chmod +x remove_wireguard.sh
sudo bash remove_wireguard.sh
```

---

## Post-Removal Verification

After removal, verify Tailscale still works:

```bash
# Check Tailscale status
tailscale status

# Test connection from Windows
# ssh jetson-tailscale
```

**Expected:** Tailscale should work normally (it doesn't depend on WireGuard packages).

---

## What to Keep

**Keep these files (they're documentation, not WireGuard):**
- ✅ `vpn_config/setup_wireguard.sh` (for reference)
- ✅ `vpn_config/*.md` (documentation files)
- ✅ `012_VPN_SETUP_AND_REMOTE_ACCESS.md` (for reference)
- ✅ `012_VPN_SETUP_TAILSCALE.md` (current solution)

**Remove these (contain keys or are WireGuard-specific):**
- ❌ `vpn_config/client_wg0.conf` (contains private key)
- ❌ `vpn_config/*.key` (all key files)
- ❌ `~/.r2d2_vpn_secrets/` (contains all keys)

---

## Security Reminder

**Before deletion, ensure:**
- ✅ All keys are stored in KeePass (if you want to keep them)
- ✅ You understand keys cannot be recovered after deletion
- ✅ Tailscale is working and you don't need WireGuard

**After deletion:**
- ✅ Keys are permanently removed from the system
- ✅ No WireGuard remnants remain
- ✅ System is clean

---

## Troubleshooting

### Problem: Can't remove packages

**Solution:**
```bash
# Force removal
sudo apt remove --purge -y wireguard wireguard-tools wireguard-go wireguard-dkms
sudo apt autoremove -y
```

### Problem: Service won't disable

**Solution:**
```bash
# Force disable
sudo systemctl disable --now wg-quick@wg0
sudo systemctl daemon-reload
```

### Problem: Files won't delete

**Solution:**
```bash
# Check permissions
ls -la /etc/wireguard/

# Use sudo if needed
sudo rm -rf /etc/wireguard/
```

---

## Summary

**What gets removed:**
- ✅ All WireGuard packages
- ✅ All configuration files
- ✅ All key files (server and client)
- ✅ Systemd services
- ✅ Secure storage directory

**What stays:**
- ✅ Documentation files (for reference)
- ✅ Tailscale (completely separate)
- ✅ SSH configuration (works with Tailscale)

**Result:**
- ✅ Clean system with no WireGuard remnants
- ✅ Tailscale continues to work
- ✅ Remote access still functional

---

**Created:** December 11, 2025  
**Related:** `012_VPN_SETUP_TAILSCALE.md` - Current VPN solution

