# Windows Tailscale Reconnection Guide

## Issue Confirmed
Windows Tailscale shows **"not connected"** - this is why SSH over Tailscale fails.

## Quick Fix Steps

### Step 1: Reconnect Tailscale on Windows

1. **Open Tailscale app:**
   - Click Tailscale icon in system tray (if visible)
   - Or search "Tailscale" in Start menu and open it

2. **Click "Connect" or "Sign in":**
   - If you see a "Connect" button, click it
   - If you see "Sign in", sign in with your account (Google/Microsoft/GitHub)
   - Use the same account you used when setting up Tailscale originally

3. **Wait for connection:**
   - Status should change to "Connected"
   - System tray icon should show connected status

### Step 2: Verify Connection

After Tailscale connects on Windows:

1. **Check Tailscale status:**
   - Right-click Tailscale icon → Status
   - Should show "Connected"
   - Should list devices including "r2d2" (Jetson)

2. **Check admin console:**
   - Visit: https://login.tailscale.com/admin/machines
   - Both devices should show as **online** (green status)

3. **Test ping from Windows (PowerShell):**
   ```powershell
   ping 100.95.133.26
   ```
   Should get replies.

4. **Test SSH from Windows (PowerShell):**
   ```powershell
   ssh jetson-tailscale
   ```
   Should connect successfully.

## If "Connect" Button Doesn't Work

### Option A: Restart Tailscale Service

1. **Open Task Manager** (Ctrl+Shift+Esc)
2. Go to **Services** tab
3. Find **Tailscale** service
4. Right-click → **Restart**
5. Try connecting again

### Option B: Sign Out and Sign Back In

1. Open Tailscale app
2. Click on your profile/account icon
3. Select **"Sign out"** or **"Disconnect"**
4. Sign back in with your account
5. Wait for reconnection

### Option C: Reinstall Tailscale (Last Resort)

1. **Uninstall Tailscale:**
   - Settings → Apps → Installed apps
   - Find "Tailscale" → Uninstall

2. **Reinstall Tailscale:**
   - Download from: https://tailscale.com/download/windows
   - Install and sign in with same account

3. **Device should automatically join your Tailscale network**

## Common Causes of Disconnection

1. **Windows sleep/hibernate** - Tailscale may disconnect and not auto-reconnect
2. **Network change** - Switching WiFi networks or VPN can disconnect Tailscale
3. **Windows firewall** - May block Tailscale (add exception)
4. **Windows updates** - Can interrupt Tailscale service
5. **Manual disconnect** - If you manually disconnected earlier

## Prevention: Enable Auto-Start

To prevent future disconnections:

1. **Open Tailscale app**
2. Go to **Settings** (gear icon)
3. Enable **"Start Tailscale on boot"** or **"Run on startup"**
4. This ensures Tailscale starts automatically when Windows boots

## Verify It's Working

After reconnecting, run this on Jetson to verify:
```bash
tailscale status
```

You should see:
```
100.95.133.26  r2d2      severin.leuenberger@  linux    -
100.100.52.23  itxcl883  severin.leuenberger@  windows  active; direct ...
```

Windows device should show as **"active"** (not offline).

## Still Not Working?

If Tailscale still won't connect on Windows:

1. **Check Windows Firewall:**
   - Settings → Privacy & Security → Windows Security → Firewall
   - Ensure Tailscale is allowed through firewall

2. **Check Internet connection:**
   - Make sure Windows laptop has internet access
   - Tailscale needs internet to connect

3. **Check for VPN conflicts:**
   - If you have other VPN software running, it might conflict
   - Try disconnecting other VPNs temporarily

4. **Check Tailscale logs on Windows:**
   - Open Event Viewer (search "Event Viewer" in Start menu)
   - Windows Logs → Application
   - Look for Tailscale errors

5. **Contact Tailscale support:**
   - If nothing works, check Tailscale status page
   - Or visit: https://tailscale.com/support

## Summary

**Problem:** Windows Tailscale disconnected  
**Solution:** Reconnect Tailscale on Windows (click Connect/Sign in)  
**Prevention:** Enable auto-start in Tailscale settings  
**Verification:** Check `tailscale status` on Jetson shows Windows as "active"  

Once reconnected, SSH should work: `ssh jetson-tailscale`

