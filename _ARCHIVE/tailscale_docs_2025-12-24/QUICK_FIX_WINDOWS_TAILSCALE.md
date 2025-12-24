# Quick Fix: Windows Tailscale Reconnection

## Problem Confirmed
✅ **Root cause identified:** Windows Tailscale shows "not connected"
✅ **Evidence from Jetson:** Tailscale status shows Windows device as `Online: false`

## Solution: Reconnect Tailscale on Windows

### Method 1: Simple Reconnection (Try This First)

1. **Open Tailscale app on Windows:**
   - Click Tailscale icon in system tray, OR
   - Search "Tailscale" in Start menu

2. **Click "Connect" button:**
   - Should be visible in the Tailscale window
   - Or click "Sign in" if prompted

3. **Wait 10-30 seconds** for connection to establish

4. **Verify:**
   - Status should change to "Connected"
   - System tray icon should show green/connected

### Method 2: If "Connect" Button Doesn't Appear

1. **Sign out and back in:**
   - Open Tailscale app
   - Click on your profile/account (top right)
   - Select "Sign out" or "Disconnect"
   - Sign back in with your account (Google/Microsoft/GitHub)

2. **Wait for reconnection**

### Method 3: Restart Tailscale Service

1. **Open Task Manager** (Ctrl+Shift+Esc)
2. Go to **Services** tab
3. Find **"Tailscale"** service
4. Right-click → **Restart**
5. Open Tailscale app and connect

## Verify It Worked

After reconnecting on Windows, check from Jetson:

```bash
tailscale status
```

Should show:
```
100.100.52.23  itxcl883  severin.leuenberger@  windows  active; direct ...
```

Notice: Should show as **"active"** (not "offline").

Then test SSH from Windows:
```powershell
ssh jetson-tailscale
```

Should connect successfully!

## Why This Happened

Tailscale disconnected on Windows. Common causes:
- Windows sleep/hibernate
- Network change
- Windows restart without Tailscale auto-starting
- Manual disconnect

**Prevention:** Enable "Start Tailscale on boot" in Tailscale Settings.

## Need More Help?

See full guide: `/home/severin/windows_tailscale_reconnect_guide.md`

