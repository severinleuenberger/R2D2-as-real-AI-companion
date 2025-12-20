# VPN Setup and Remote Access Guide - Tailscale Solution

**Date:** December 11, 2025  
**Platform:** NVIDIA Jetson AGX Orin 64GB (Ubuntu 22.04)  
**Purpose:** Secure remote access to Jetson Orin over public Internet  
**VPN Technology:** Tailscale (WireGuard-based, zero-configuration)

---

## Executive Summary

This guide documents the successful setup of Tailscale VPN for remote access to the Jetson Orin. Tailscale provides an encrypted VPN tunnel from your Windows laptop to the Jetson Orin, allowing you to:
- ✅ Access Jetson via SSH (VS Code Remote SSH, terminal) as if on local network
- ✅ Keep your existing SSH workflow unchanged
- ✅ Secure connection (no direct SSH exposure to Internet)
- ✅ Low latency, stable connection
- ✅ Works from anywhere in the world

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

## Why Not WireGuard?

**Note:** We initially attempted to set up native WireGuard VPN, but encountered kernel module compatibility issues on the Jetson Orin:

- **Problem:** The Jetson Orin uses a custom NVIDIA kernel (5.15.148-tegra) that doesn't include the WireGuard kernel module
- **Attempted Solutions:**
  - Tried installing `wireguard-dkms` (kernel headers package not available for Jetson kernel)
  - Tried using `wireguard-go` (userspace implementation) - encountered TUN device creation issues
  - Kernel module could not be built or loaded
- **Result:** WireGuard setup failed, all WireGuard components were removed
- **Solution:** Switched to Tailscale, which works perfectly on Jetson without requiring kernel modules

**For reference:** The original WireGuard setup attempt is documented in `vpn_config/WIREGUARD_CLEANUP_GUIDE.md` (removal guide) and `vpn_config/setup_wireguard.sh` (setup script that was attempted).

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

## Installation Guide

### Step 1: Install Tailscale on Jetson (2 minutes)

```bash
# Install Tailscale
curl -fsSL https://tailscale.com/install.sh | sh

# Authenticate (will show URL to visit)
sudo tailscale up

# Enable on boot
sudo systemctl enable tailscaled
sudo systemctl start tailscaled
```

**What happens:**
- Tailscale installs automatically
- Running `sudo tailscale up` shows a URL
- Open that URL in a web browser (on any device)
- Sign in with Google, Microsoft, or GitHub
- Authorize the device
- Jetson will be added to your Tailscale network

### Step 2: Get Your Jetson's Tailscale IP (1 minute)

After authentication, run:

```bash
tailscale status
```

**You'll see something like:**
```
100.95.133.26    r2d2    severin@    linux   -
```

**Note the IP address** (e.g., `100.95.133.26`) - you'll use this to connect.

**Or get just the IP:**
```bash
tailscale ip -4
```

### Step 3: Install Tailscale on Windows (3 minutes)

1. **Download Tailscale:**
   - Go to: https://tailscale.com/download/windows
   - Download and install Tailscale for Windows
   - Or install from Microsoft Store: Search "Tailscale"

2. **Sign in:**
   - Open Tailscale app
   - Sign in with the same account you used on Jetson (Google/Microsoft/GitHub)
   - The app will connect automatically

3. **Verify connection:**
   - Both devices should appear in the Tailscale admin console
   - https://login.tailscale.com/admin/machines
   - Status should show "Connected" for both devices

### Step 4: Configure SSH on Windows (2 minutes)

**Add to your SSH config:** `C:\Users\SeverinLeuenberger\.ssh\config`

```
Host jetson-tailscale
    HostName 100.95.133.26
    User severin
    IdentityFile C:\Users\SeverinLeuenberger\.ssh\id_ed25519
```

**Replace `100.95.133.26` with your actual Tailscale IP from Step 2.**

**How to edit:**
1. Open PowerShell or Command Prompt
2. Navigate: `cd .ssh`
3. Edit: `notepad config`
4. Add the configuration above
5. Save and close

### Step 5: Test Connection (1 minute)

**From Windows (PowerShell):**

```powershell
# Test ping
ping 100.95.133.26

# Test SSH
ssh jetson-tailscale
```

**VS Code Remote SSH:**
- Open VS Code
- Press `F1` (or `Ctrl+Shift+P`)
- Type: "Remote-SSH: Connect to Host"
- Select: `jetson-tailscale`
- Should connect successfully

---

## Verification & Testing

### Check Tailscale Status on Jetson

```bash
tailscale status
```

**Expected Output:**
```
100.95.133.26  r2d2      severin.leuenberger@  linux    -                          
100.100.52.23  itxcl883  severin.leuenberger@  windows  active; direct ...
```

Both devices should show as connected.

### Test Connection from Windows

1. **Ping test:**
   ```powershell
   ping 100.95.133.26
   ```
   Should get replies.

2. **SSH test:**
   ```powershell
   ssh jetson-tailscale
   ```
   Should connect successfully.

3. **VS Code Remote SSH:**
   - Open VS Code
   - Connect to `jetson-tailscale`
   - Should connect through Tailscale automatically

### Monitor Connection

**On Jetson:**
```bash
# Check status
tailscale status

# Get IP
tailscale ip -4

# Check service
sudo systemctl status tailscaled

# View logs
sudo journalctl -u tailscaled -f
```

**On Windows:**
- Tailscale app shows connection status in system tray
- Right-click icon → "Network devices" to see all devices
- Admin console: https://login.tailscale.com/admin/machines

---

## Network Architecture

```
Internet
    ↓
Tailscale Network (Encrypted WireGuard Protocol)
    ├─ Windows Laptop (100.100.52.23)
    └─ Jetson Orin (100.95.133.26)
         ↓
    SSH (Port 22) accessible via Tailscale IP
```

**Key Points:**
- All traffic encrypted via WireGuard protocol (managed by Tailscale)
- No router port forwarding needed
- Works behind any firewall/NAT
- Automatic key management
- IP addresses managed by Tailscale
- Works from anywhere in the world

### IP Addresses

| Device | IP Address | Purpose |
|--------|------------|---------|
| **Jetson (Tailscale)** | `100.95.133.26` | Tailscale-assigned IP (may change) |
| **Windows (Tailscale)** | `100.100.52.23` | Tailscale-assigned IP (may change) |
| **Jetson (Local Network)** | `192.168.1.129` | Local network IP (for reference) |

**Note:** Tailscale IPs are in the `100.x.x.x` range and are managed automatically. If they change, update your SSH config.

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

**Optional: Use device name instead of IP:**
If Tailscale MagicDNS is enabled, you can use:
```
Host jetson-tailscale
    HostName r2d2
    User severin
    IdentityFile C:\Users\SeverinLeuenberger\.ssh\id_ed25519
```

### Web UI Access

The R2D2 Web UI is secured to only listen on the Tailscale interface. You cannot access it via the local IP (192.168.x.x).

**Service Mode (Always On):**
- URL: `http://100.95.133.26:8079`
- Use this to check status and start the main dashboard

**Main Dashboard (On Demand):**
- URL: `http://100.95.133.26:8080`
- Only active after starting it from the Service Mode page

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
- Tailscale shows disconnected

**Solutions:**

1. **Check Tailscale status on Windows:**
   - Right-click Tailscale icon in system tray
   - Verify it shows "Connected"
   - If disconnected, click to reconnect
   - Or restart Tailscale app

2. **Check Tailscale status on Jetson:**
   ```bash
   tailscale status
   ```
   Both devices should show as "Connected"

3. **Check admin console:**
   - https://login.tailscale.com/admin/machines
   - Verify both devices are online (green status)

4. **Test ping from Windows:**
   ```powershell
   ping 100.95.133.26
   ```
   Should get replies. If not, check Windows Firewall.

5. **Check Windows Firewall:**
   - May need to allow Tailscale through firewall
   - Or temporarily disable firewall to test
   - Settings → Privacy & Security → Windows Security → Firewall

6. **Restart Tailscale on Windows:**
   - Right-click Tailscale icon → Exit
   - Open Tailscale from Start menu
   - Wait for connection

### Problem: Tailscale IP changed

**Symptoms:**
- SSH connection fails
- IP address different from what's in SSH config

**Solutions:**

1. **Get current IP on Jetson:**
   ```bash
   tailscale ip -4
   ```

2. **Update SSH config on Windows:**
   - Edit `C:\Users\SeverinLeuenberger\.ssh\config`
   - Update `HostName` with new IP
   - Save and retry connection

3. **Or use device name (if MagicDNS enabled):**
   - In Tailscale admin console → Settings → MagicDNS
   - Enable MagicDNS
   - Then use device name in SSH config:
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

# Restart if needed
sudo systemctl restart tailscaled
```

**On Windows:**
- Open Tailscale from Start menu
- Or check if service is running in Task Manager

### Problem: SSH service not running on Jetson

```bash
# Check status
sudo systemctl status ssh

# Start if stopped
sudo systemctl start ssh

# Enable on boot
sudo systemctl enable ssh
```

### Problem: Connection is slow

**Solutions:**

1. **Check Tailscale status:**
   ```bash
   tailscale status
   ```
   Look for connection quality indicators

2. **Check if direct connection:**
   - In admin console, check if connection shows "direct" or "relay"
   - Direct connections are faster
   - Relay connections may be slower but still work

3. **Restart Tailscale:**
   ```bash
   sudo systemctl restart tailscaled
   ```

---

## Security Notes

### What's Secure

✅ **All traffic encrypted** - WireGuard protocol (managed by Tailscale)  
✅ **Only your devices** - Only devices in your Tailscale network can connect  
✅ **No direct exposure** - SSH not exposed to public Internet  
✅ **Automatic key management** - Tailscale handles all keys  
✅ **No router config** - Works behind any firewall

### What to Store Securely

**⚠️ No secrets to store!** Tailscale manages everything automatically:
- ✅ Keys are managed by Tailscale
- ✅ No configuration files with secrets
- ✅ Authentication via OAuth (Google/Microsoft/GitHub account)

**Optional: Store Tailscale account info in KeePass:**
- Account email: `severin.leuenberger@gmail.com`
- Admin console: https://login.tailscale.com/admin/machines
- (Not required, but can be stored for reference)

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

**Connect via SSH:**
```powershell
ssh jetson-tailscale
```

**Connect via VS Code:**
- `F1` → "Remote-SSH: Connect to Host" → `jetson-tailscale`

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

### Adding More Devices

To add additional devices (phone, tablet, etc.):

1. **Install Tailscale** on the new device
2. **Sign in** with the same account
3. **Device automatically joins** your Tailscale network
4. **Access from anywhere** using the device's Tailscale IP

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

### Network Addresses

- **Jetson Tailscale IP:** `100.95.133.26` (may change - check with `tailscale ip -4`)
- **Windows Tailscale IP:** `100.100.52.23` (may change)
- **SSH Port:** `22/TCP` (accessed via Tailscale)

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

- `vpn_config/WIREGUARD_CLEANUP_GUIDE.md` - WireGuard removal instructions (for reference)
- `vpn_config/TAILSCALE_SETUP_GUIDE.md` - Quick setup reference
- `vpn_config/README_VPN_SOLUTION.md` - Quick reference summary
- `000_INTERNAL_AGENT_NOTES.md` - Development environment setup
- `001_ARCHITECTURE_OVERVIEW.md` - System architecture

---

**Created:** December 10, 2025  
**Last Updated:** December 11, 2025  
**Status:** ✅ Production Ready  
**Solution:** Tailscale VPN
