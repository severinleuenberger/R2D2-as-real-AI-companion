# Tailscale Setup Guide for R2D2 Project

**Date:** December 11, 2025  
**Platform:** NVIDIA Jetson AGX Orin 64GB (Ubuntu 22.04)  
**Purpose:** Secure remote access to Jetson Orin (alternative to WireGuard)

---

## Why Tailscale?

- ✅ Works on Jetson without kernel modules
- ✅ Easy setup (5 minutes)
- ✅ Free for personal use
- ✅ WireGuard-based (secure)
- ✅ No router configuration needed
- ✅ Works from anywhere

---

## Step 1: Install Tailscale on Jetson ✅

You've already run:
```bash
curl -fsSL https://tailscale.com/install.sh | sh
```

---

## Step 2: Authenticate Tailscale

Run this command:
```bash
sudo tailscale up
```

**What happens:**
- Tailscale will show you a URL
- Open that URL in a web browser (on any device)
- Sign in with Google, Microsoft, or GitHub
- Authorize the device

**After authentication:**
- The device will be added to your Tailscale network
- You'll get a Tailscale IP address (e.g., `100.x.x.x`)

---

## Step 3: Get Your Jetson's Tailscale IP

After authentication, run:
```bash
tailscale status
```

**You'll see something like:**
```
100.64.1.2    jetson-orin    severin@    linux   -
```

**Note the IP address** (e.g., `100.64.1.2`) - you'll use this to connect.

---

## Step 4: Install Tailscale on Windows Laptop

1. **Download Tailscale:**
   - Go to: https://tailscale.com/download/windows
   - Download and install Tailscale for Windows

2. **Sign in:**
   - Open Tailscale app
   - Sign in with the same account you used on Jetson
   - The app will connect automatically

3. **Verify connection:**
   - Both devices should appear in the Tailscale admin console
   - https://login.tailscale.com/admin/machines

---

## Step 5: Test SSH Connection

**From Windows (PowerShell or Command Prompt):**

```powershell
# Test ping
ping 100.64.1.2
# (Use your actual Tailscale IP from Step 3)

# Test SSH
ssh severin@100.64.1.2
# (Use your actual Tailscale IP)
```

**VS Code Remote SSH:**
- Add to your SSH config:
  ```
  Host jetson-tailscale
      HostName 100.64.1.2
      User severin
  ```
- Connect as usual - it will work through Tailscale!

---

## Step 6: Make Tailscale Start on Boot

On Jetson:
```bash
sudo systemctl enable tailscaled
sudo systemctl start tailscaled
```

---

## Daily Usage

**On Jetson:**
- Tailscale starts automatically on boot
- Check status: `tailscale status`
- Check IP: `tailscale ip -4`

**On Windows:**
- Tailscale runs in the background
- Connect to Jetson using Tailscale IP
- VS Code Remote SSH works automatically

---

## Troubleshooting

### Problem: Can't authenticate

**Solution:**
- Make sure you can access the URL from a browser
- Try: `sudo tailscale up --accept-routes`

### Problem: Can't connect from Windows

**Solution:**
1. Check both devices are online in Tailscale admin console
2. Verify Tailscale IP: `tailscale status` on Jetson
3. Check firewall: `sudo ufw allow 41641/udp` (Tailscale port)

### Problem: Tailscale IP changed

**Solution:**
- Tailscale IPs can change
- Check current IP: `tailscale ip -4` on Jetson
- Or use device name: `ssh severin@jetson-orin` (if DNS enabled)

---

## Advantages Over WireGuard

| Feature | WireGuard | Tailscale |
|---------|-----------|-----------|
| Setup Time | 30-60 min | 5 min |
| Kernel Module | Required | Not needed |
| Router Config | Required | Not needed |
| Dynamic IP | Manual update | Automatic |
| Device Management | Manual | Web console |
| Works on Jetson | Complex | ✅ Easy |

---

## Security Notes

- ✅ All traffic is encrypted (WireGuard protocol)
- ✅ Only devices in your Tailscale network can connect
- ✅ No direct exposure to Internet
- ✅ Keys managed automatically

---

## Quick Reference

**Check status:**
```bash
tailscale status
```

**Get IP address:**
```bash
tailscale ip -4
```

**Restart service:**
```bash
sudo systemctl restart tailscaled
```

**View logs:**
```bash
sudo journalctl -u tailscaled -f
```

---

**Created:** December 11, 2025  
**Related:** `012_VPN_SETUP_AND_REMOTE_ACCESS.md` (WireGuard guide - for reference)

