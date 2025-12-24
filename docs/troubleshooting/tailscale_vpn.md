# Tailscale VPN Troubleshooting Guide

## Overview

R2D2 uses Tailscale VPN for secure remote access to the Jetson AGX Orin from Windows. This guide covers common connection issues and their solutions.

---

## Quick Fix: Windows Disconnected

### Problem

SSH over Tailscale fails with connection timeout. Jetson shows Windows device as **"offline"** in Tailscale status.

### Root Cause

Windows Tailscale client is disconnected. Common causes:
- Windows sleep/hibernate
- Network change (WiFi switch, VPN disconnect)
- Windows restart without Tailscale auto-start
- Manual disconnect
- Windows firewall blocking Tailscale

### Solution

#### Step 1: Reconnect Tailscale on Windows

1. **Open Tailscale app:**
   - Click Tailscale icon in system tray, OR
   - Search "Tailscale" in Start menu and open it

2. **Click "Connect" or "Sign in":**
   - If you see a "Connect" button, click it
   - If you see "Sign in", sign in with your account (Google/Microsoft/GitHub)
   - Use the same account you used when setting up Tailscale originally

3. **Wait for connection:**
   - Status should change to "Connected"
   - System tray icon should show connected status (10-30 seconds)

#### Step 2: Verify Connection

From **Windows PowerShell**:

```powershell
# Test ping to Jetson
ping 100.95.133.26

# Test SSH connection
ssh jetson-tailscale
```

From **Jetson terminal**:

```bash
# Check Tailscale status
tailscale status
```

**Expected output:**
```
100.95.133.26  r2d2      severin.leuenberger@  linux    -
100.100.52.23  itxcl883  severin.leuenberger@  windows  active; direct ...
```

Windows device should show as **"active"** (not offline).

#### Step 3: Check Admin Console

Visit: https://login.tailscale.com/admin/machines

Both devices should show:
- ✅ **Online** (green status)
- ✅ **Connected** status
- ✅ Recent "Last seen" timestamp

---

## Advanced Troubleshooting

### If "Connect" Button Doesn't Work

#### Option A: Restart Tailscale Service

1. **Open Task Manager** (Ctrl+Shift+Esc)
2. Go to **Services** tab
3. Find **"Tailscale"** service
4. Right-click → **Restart**
5. Open Tailscale app and try connecting again

#### Option B: Sign Out and Sign Back In

1. Open Tailscale app
2. Click on your profile/account icon (top right)
3. Select **"Sign out"** or **"Disconnect"**
4. Sign back in with your account
5. Wait for reconnection (30-60 seconds)

#### Option C: Reinstall Tailscale (Last Resort)

1. **Uninstall Tailscale:**
   - Settings → Apps → Installed apps
   - Find "Tailscale" → Uninstall

2. **Reinstall Tailscale:**
   - Download from: https://tailscale.com/download/windows
   - Install and sign in with same account

3. **Device should automatically join your Tailscale network**

### DNS Connectivity Warning

**Symptom:** Tailscale reports "can't reach the configured DNS servers"

**Solution:** Restart Tailscale on Jetson

```bash
sudo systemctl restart tailscaled
sleep 2
tailscale status
```

This will:
- Restart the Tailscale service
- Re-establish connections
- Resolve DNS warnings

---

## Diagnostic Commands

### Check Tailscale Status (Jetson)

```bash
# View all connected devices
tailscale status

# Check service status
sudo systemctl status tailscaled

# Verify Tailscale is enabled on boot
systemctl is-enabled tailscaled
```

### Check SSH Configuration (Windows)

Verify your SSH config at `C:\Users\SeverinLeuenberger\.ssh\config`:

```
Host jetson-tailscale
    HostName 100.95.133.26
    User severin
    IdentityFile C:\Users\SeverinLeuenberger\.ssh\id_ed25519
```

### Test Connection (Windows PowerShell)

```powershell
# Test ping
ping 100.95.133.26

# Test SSH with verbose output
ssh -v jetson-tailscale

# Check Tailscale peer list
tailscale status
```

---

## Diagnostic Evidence Interpretation

When running diagnostics, look for these key indicators:

### CONFIRMED Issues

- **Windows device offline** - Status shows: "offline, last seen X minutes ago"
- **Ping test failed** - Cannot reach Windows device from Jetson
- **DNS connectivity warning** - May affect connections

### REJECTED Hypotheses (These are OK)

- ✓ Tailscale service running (PID active)
- ✓ SSH service running and listening on port 22
- ✓ Tailscale IP is correct (100.95.133.26 for Jetson)
- ✓ Systemd service enabled for auto-start

---

## Prevention

### Enable Auto-Start on Windows

To prevent future disconnections:

1. **Open Tailscale app**
2. Go to **Settings** (gear icon)
3. Enable **"Start Tailscale on boot"** or **"Run on startup"**
4. This ensures Tailscale starts automatically when Windows boots

### Verify Auto-Start on Jetson

Tailscale should already be configured to auto-start:

```bash
# Verify auto-start is enabled
systemctl is-enabled tailscaled
# Should return: enabled

# Verify service is running
sudo systemctl status tailscaled
```

### Windows Firewall Configuration

Ensure Tailscale is allowed through Windows Firewall:

1. **Open Windows Security:**
   - Settings → Privacy & Security → Windows Security → Firewall & network protection

2. **Allow an app through firewall:**
   - Click "Allow an app through firewall"
   - Find "Tailscale" in the list
   - Ensure both **Private** and **Public** checkboxes are checked

---

## Still Not Working?

If Tailscale still won't connect after trying all solutions:

### 1. Check Windows Firewall

- Settings → Privacy & Security → Windows Security → Firewall
- Ensure Tailscale is allowed through firewall (both Private and Public networks)

### 2. Check Internet Connection

- Make sure Windows laptop has active internet access
- Tailscale requires internet to establish connections

### 3. Check for VPN Conflicts

- If you have other VPN software running, it might conflict
- Try disconnecting other VPNs temporarily
- Some corporate VPNs block Tailscale traffic

### 4. Check Tailscale Logs on Windows

1. Open **Event Viewer** (search in Start menu)
2. Go to **Windows Logs → Application**
3. Look for Tailscale errors (filter by source: "Tailscale")

### 5. Verify Network Settings

From Windows PowerShell:

```powershell
# Check if Tailscale adapter is present
ipconfig /all | Select-String -Pattern "Tailscale"

# Check routing
route print | Select-String -Pattern "100.64"
```

### 6. Contact Tailscale Support

If nothing works:
- Check Tailscale status page: https://status.tailscale.com/
- Contact support: https://tailscale.com/support
- Join Tailscale Slack community

---

## Common Scenarios

### Scenario 1: After Windows Sleep/Hibernate

**Symptom:** SSH fails after Windows laptop wakes from sleep

**Solution:**
1. Wait 30 seconds for Tailscale to auto-reconnect
2. If no auto-reconnect, manually reconnect via Tailscale app
3. Enable "Start on boot" to improve reliability

### Scenario 2: After Network Switch

**Symptom:** SSH fails after switching WiFi networks

**Solution:**
1. Wait 30-60 seconds for Tailscale to detect network change
2. Check Tailscale app shows "Connected"
3. If stuck, restart Tailscale service

### Scenario 3: After Windows Update

**Symptom:** SSH fails after Windows updates

**Solution:**
1. Windows may have disabled Tailscale auto-start
2. Manually start Tailscale from Start menu
3. Re-enable "Start on boot" in settings

### Scenario 4: Corporate Network/Firewall

**Symptom:** Tailscale connects but SSH is slow or fails

**Solution:**
1. Corporate firewall may be blocking direct connections
2. Tailscale should fall back to relay servers (slower but works)
3. Check with IT department about Tailscale connectivity

---

## Reference Information

### IP Addresses

- **Jetson (r2d2):** `100.95.133.26`
- **Windows (itxcl883):** `100.100.52.23`

### SSH Configuration

- **Host alias:** `jetson-tailscale`
- **User:** `severin`
- **Key:** `~/.ssh/id_ed25519`

### Tailscale Network

- **Account:** `severin.leuenberger@`
- **Network:** Personal Tailscale account
- **Admin console:** https://login.tailscale.com/admin/machines

---

## Summary

| Problem | Solution | Time |
|---------|----------|------|
| Windows disconnected | Reconnect Tailscale on Windows | 30 seconds |
| Connect button missing | Sign out and sign back in | 1 minute |
| Service not running | Restart Tailscale service | 30 seconds |
| DNS warning | Restart tailscaled on Jetson | 10 seconds |
| Persistent issues | Reinstall Tailscale | 5 minutes |

**Most common fix:** Reconnect Tailscale on Windows (click Connect button)

**Prevention:** Enable "Start Tailscale on boot" in Tailscale settings

---

## Related Documentation

- **Main VPN Setup:** `012_VPN_SETUP_AND_REMOTE_ACCESS.md`
- **Tailscale Documentation:** https://tailscale.com/kb/
- **SSH Configuration:** `~/.ssh/config`

---

**Last Updated:** December 2025  
**Status:** ✅ Comprehensive troubleshooting guide

