# VPN Setup and Remote Access Guide - Tailscale Solution

**Date:** December 11, 2025  
**Platform:** NVIDIA Jetson AGX Orin 64GB (Ubuntu 22.04)  
**Purpose:** Secure remote access to Jetson Orin over public Internet  
**VPN Technology:** Tailscale (WireGuard-based, zero-configuration)

---

## Executive Summary

This guide documents the successful setup of Tailscale VPN for remote access to the Jetson Orin. Tailscale was chosen over native WireGuard due to kernel module compatibility issues on Jetson devices.

**Why Tailscale?**
- ✅ Works on Jetson without kernel modules
- ✅ Zero-configuration setup (5 minutes)
- ✅ Free for personal use
- ✅ WireGuard-based (secure)
- ✅ No router configuration needed
- ✅ Works from anywhere in the world

**Current Status:** ✅ **FULLY OPERATIONAL**
- Tailscale installed and connected on Jetson
- Tailscale installed and connected on Windows laptop
- SSH connection working through Tailscale
- VS Code Remote SSH working through Tailscale

---

## Quick Start

### Current Configuration

| Device | Hostname | Tailscale IP | Status |
|--------|----------|--------------|--------|
| **Jetson Orin** | r2d2 | 100.95.133.26 | ✅ Connected |
| **Windows Laptop** | itxcl883 | 100.100.52.23 | ✅ Connected |

### Connect from Windows

**SSH:**
```powershell
ssh jetson-tailscale
```

**Or using IP directly:**
```powershell
ssh severin@100.95.133.26
```

**VS Code Remote SSH:**
- Press `F1` → "Remote-SSH: Connect to Host"
- Select: `jetson-tailscale`

---

## Installation History

### Jetson Setup (Completed)

1. **Installed Tailscale:**
   ```bash
   curl -fsSL https://tailscale.com/install.sh | sh
   ```

2. **Authenticated:**
   ```bash
   sudo tailscale up
   # Followed URL to authenticate with Google account
   ```

3. **Enabled on boot:**
   ```bash
   sudo systemctl enable tailscaled
   sudo systemctl start tailscaled
   ```

### Windows Setup (Completed)

1. **Downloaded Tailscale:**
   - From: https://tailscale.com/download/windows
   - Installed Tailscale for Windows

2. **Signed in:**
   - Opened Tailscale app
   - Signed in with same account (Google)
   - Connected automatically

3. **Configured SSH:**
   - Added to `C:\Users\SeverinLeuenberger\.ssh\config`:
     ```
     Host jetson-tailscale
         HostName 100.95.133.26
         User severin
         IdentityFile C:\Users\SeverinLeuenberger\.ssh\id_ed25519
     ```

---

## Daily Usage

### On Jetson

**Check Tailscale status:**
```bash
tailscale status
```

**Get Tailscale IP:**
```bash
tailscale ip -4
```

**Check service:**
```bash
sudo systemctl status tailscaled
```

**Restart if needed:**
```bash
sudo systemctl restart tailscaled
```

### On Windows

**Check Tailscale:**
- Right-click Tailscale icon in system tray
- Status shows "Connected"
- Click "Network devices" to see all devices

**Admin Console:**
- https://login.tailscale.com/admin/machines
- View all devices, IPs, and status

---

## Network Architecture

```
Internet
    ↓
Tailscale Network (Encrypted WireGuard)
    ├─ Windows Laptop (100.100.52.23)
    └─ Jetson Orin (100.95.133.26)
         ↓
    SSH (Port 22) accessible via Tailscale IP
```

**Key Points:**
- All traffic encrypted via WireGuard protocol
- No router port forwarding needed
- Works behind any firewall/NAT
- Automatic key management
- IP addresses managed by Tailscale

---

## SSH Configuration

### Windows SSH Config

**Location:** `C:\Users\SeverinLeuenberger\.ssh\config`

**Current Configuration:**
```
Host r2d2
    HostName 192.168.55.1
    User severin
    IdentityFile C:\Users\SeverinLeuenberger\.ssh\id_ed25519

Host jetson-tailscale
    HostName 100.95.133.26
    User severin
    IdentityFile C:\Users\SeverinLeuenberger\.ssh\id_ed25519
```

**Usage:**
- `ssh r2d2` - Connects via local network (when on same network)
- `ssh jetson-tailscale` - Connects via Tailscale (works from anywhere)

---

## VS Code Remote SSH Setup

### Configuration

1. **SSH Config:** Already configured (see above)

2. **Connect in VS Code:**
   - Press `F1` (or `Ctrl+Shift+P`)
   - Type: "Remote-SSH: Connect to Host"
   - Select: `jetson-tailscale`
   - VS Code connects to Jetson through Tailscale

3. **Works seamlessly:**
   - Edit files on Jetson
   - Run terminal commands
   - Use all VS Code features remotely
   - Works from anywhere in the world

---

## Troubleshooting

### Problem: Can't connect from Windows

**Symptoms:**
- SSH connection times out
- Ping fails

**Solutions:**

1. **Check Tailscale status on Windows:**
   - Right-click Tailscale icon
   - Verify it shows "Connected"
   - If disconnected, reconnect

2. **Check Tailscale status on Jetson:**
   ```bash
   tailscale status
   ```
   Both devices should show as "Connected"

3. **Check admin console:**
   - https://login.tailscale.com/admin/machines
   - Verify both devices are online

4. **Test ping from Windows:**
   ```powershell
   ping 100.95.133.26
   ```
   Should get replies

5. **Check Windows Firewall:**
   - May need to allow Tailscale through firewall
   - Or temporarily disable to test

### Problem: Tailscale IP changed

**Symptoms:**
- SSH connection fails
- IP address different

**Solutions:**

1. **Get current IP on Jetson:**
   ```bash
   tailscale ip -4
   ```

2. **Update SSH config on Windows:**
   - Edit `C:\Users\SeverinLeuenberger\.ssh\config`
   - Update `HostName` with new IP

3. **Or use device name (if MagicDNS enabled):**
   ```
   Host jetson-tailscale
       HostName r2d2
       User severin
   ```

### Problem: Tailscale service not running

**On Jetson:**
```bash
# Check status
sudo systemctl status tailscaled

# Start if stopped
sudo systemctl start tailscaled

# Enable on boot
sudo systemctl enable tailscaled
```

**On Windows:**
- Open Tailscale from Start menu
- Or restart the service

### Problem: SSH service not running on Jetson

```bash
# Check status
sudo systemctl status ssh

# Start if stopped
sudo systemctl start ssh

# Enable on boot
sudo systemctl enable ssh
```

---

## Security Notes

### What's Secure

✅ **All traffic encrypted** - WireGuard protocol  
✅ **Only your devices** - Only devices in your Tailscale network can connect  
✅ **No direct exposure** - SSH not exposed to public Internet  
✅ **Automatic key management** - Tailscale handles all keys  
✅ **No router config** - Works behind any firewall

### What to Store Securely

**⚠️ No secrets to store!** Tailscale manages everything automatically:
- ✅ Keys are managed by Tailscale
- ✅ No configuration files with secrets
- ✅ Authentication via OAuth (Google account)

**Optional: Store Tailscale account info:**
- Account email: `severin.leuenberger@gmail.com`
- Admin console: https://login.tailscale.com/admin/machines
- (Store in KeePass if desired, but not required)

---

## Advantages Over WireGuard

| Feature | WireGuard | Tailscale |
|---------|-----------|-----------|
| **Setup Time** | 30-60 min | 5 min |
| **Kernel Module** | Required (failed on Jetson) | Not needed ✅ |
| **Router Config** | Required | Not needed ✅ |
| **Dynamic IP** | Manual update | Automatic ✅ |
| **Device Management** | Manual | Web console ✅ |
| **Works on Jetson** | Complex/Unreliable | ✅ Works perfectly |
| **Key Management** | Manual | Automatic ✅ |

---

## Maintenance

### Regular Checks

**Monthly:**
- Check Tailscale status: `tailscale status`
- Verify both devices connected in admin console
- Update Tailscale if needed: `sudo apt update && sudo apt upgrade tailscale`

**If Issues:**
- Restart Tailscale: `sudo systemctl restart tailscaled`
- Check logs: `sudo journalctl -u tailscaled -n 50`

### Updates

**On Jetson:**
```bash
sudo apt update
sudo apt upgrade tailscale
sudo systemctl restart tailscaled
```

**On Windows:**
- Tailscale auto-updates
- Or check for updates in Tailscale app

---

## Quick Reference

### Essential Commands

**On Jetson:**
```bash
# Status
tailscale status

# Get IP
tailscale ip -4

# Service management
sudo systemctl status tailscaled
sudo systemctl restart tailscaled
```

**On Windows:**
```powershell
# SSH connection
ssh jetson-tailscale

# Or direct IP
ssh severin@100.95.133.26
```

### File Locations

| Item | Location | Notes |
|------|----------|-------|
| **Tailscale Config** | Managed by Tailscale | No manual config needed |
| **SSH Config (Windows)** | `C:\Users\SeverinLeuenberger\.ssh\config` | Contains Tailscale host entry |
| **Tailscale Service** | `/lib/systemd/system/tailscaled.service` | Systemd service file |

---

## Success Criteria

✅ **Tailscale installed on Jetson**  
✅ **Tailscale installed on Windows**  
✅ **Both devices connected to Tailscale network**  
✅ **SSH connection works through Tailscale**  
✅ **VS Code Remote SSH works through Tailscale**  
✅ **No router configuration needed**  
✅ **Works from anywhere in the world**

**All criteria met!** 🎉

---

## Related Documentation

- `012_VPN_SETUP_AND_REMOTE_ACCESS.md` - Original WireGuard guide (for reference)
- `vpn_config/WIREGUARD_CLEANUP_GUIDE.md` - WireGuard removal instructions
- `vpn_config/TAILSCALE_SETUP_GUIDE.md` - Quick setup reference

---

## Support & Resources

### Tailscale Resources
- **Official Website:** https://tailscale.com/
- **Documentation:** https://tailscale.com/kb/
- **Admin Console:** https://login.tailscale.com/admin/machines
- **Download:** https://tailscale.com/download/

### Getting Help
1. Check Tailscale status on both devices
2. Verify devices in admin console
3. Check troubleshooting section above
4. Tailscale documentation: https://tailscale.com/kb/

---

**Created:** December 11, 2025  
**Last Updated:** December 11, 2025  
**Status:** ✅ Production Ready  
**Related Documents:**
- `000_INTERNAL_AGENT_NOTES.md` - Development environment setup
- `001_ARCHITECTURE_OVERVIEW.md` - System architecture

