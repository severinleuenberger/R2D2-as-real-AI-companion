# Windows 11 Laptop Setup Instructions

This guide walks you through setting up WireGuard VPN client on your Windows 11 laptop to connect to the Jetson Orin.

---

## Prerequisites

- ✅ Windows 11 laptop
- ✅ Admin rights (for installation)
- ✅ Client configuration file (`client_wg0.conf`) from Jetson
- ✅ KeePass database with VPN credentials (for reference)

---

## Step 1: Install WireGuard Client (5 minutes)

### Option A: Microsoft Store (Recommended)

1. **Open Microsoft Store:**
   - Press `Windows Key`
   - Type "Microsoft Store" and open it

2. **Search for WireGuard:**
   - In the search bar, type: `WireGuard`
   - Look for "WireGuard" by WireGuard Development Team

3. **Install:**
   - Click **Get** or **Install**
   - Wait for installation to complete

4. **Launch WireGuard:**
   - Click **Open** or find "WireGuard" in Start Menu
   - The app will open (may show empty tunnel list)

### Option B: Direct Download (Alternative)

1. **Visit WireGuard website:**
   - Go to: https://www.wireguard.com/install/
   - Click **Download for Windows**

2. **Download installer:**
   - Download `wireguard-installer.exe`
   - Run the installer
   - Follow installation wizard
   - **Note:** May require admin rights

3. **Launch WireGuard:**
   - Find "WireGuard" in Start Menu
   - Open the application

---

## Step 2: Get Client Configuration File (2 minutes)

You need the `client_wg0.conf` file from the Jetson. Choose one method:

### Method A: Copy File Contents (Recommended - Most Secure)

1. **On Jetson, view the file:**
   ```bash
   cat /home/severin/dev/r2d2/vpn_config/client_wg0.conf
   ```

2. **Copy the entire output** (all lines)

3. **On Windows, create new file:**
   - Open Notepad
   - Paste the contents
   - **IMPORTANT:** Make sure `YOUR_PUBLIC_IP` is replaced with actual IP (e.g., `188.61.209.189`)
   - Save as: `client_wg0.conf` (anywhere, e.g., Desktop or Downloads)

### Method B: Transfer via USB Drive

1. **On Jetson:**
   ```bash
   # Copy to USB drive (mount USB first)
   cp /home/severin/dev/r2d2/vpn_config/client_wg0.conf /media/usb/
   ```

2. **On Windows:**
   - Insert USB drive
   - Copy `client_wg0.conf` to your laptop

### Method C: Copy from KeePass (If Stored)

1. **Open KeePass** on Windows
2. **Find entry:** "R2D2 VPN - Jetson Orin"
3. **Copy client config** from Notes field
4. **Paste into Notepad**
5. **Save as:** `client_wg0.conf`

### Method D: SCP (If you have temporary SSH access)

1. **Open PowerShell or Command Prompt** on Windows
2. **Use SCP:**
   ```powershell
   scp severin@192.168.1.129:/home/severin/dev/r2d2/vpn_config/client_wg0.conf .
   ```
   (Replace `192.168.1.129` with Jetson's local IP if different)

---

## Step 3: Update Client Configuration (1 minute)

**⚠️ CRITICAL:** Make sure the client config has your actual public IP!

1. **Open `client_wg0.conf` in Notepad**

2. **Find this line:**
   ```ini
   Endpoint = YOUR_PUBLIC_IP:51820
   ```

3. **Replace `YOUR_PUBLIC_IP` with your actual public IP:**
   ```ini
   Endpoint = 188.61.209.189:51820
   ```
   (Use the IP from `get_public_ip.sh` on Jetson)

4. **Save the file**

---

## Step 4: Import Configuration into WireGuard (2 minutes)

1. **Open WireGuard** application on Windows

2. **Import tunnel:**
   - Click the **"+"** button (bottom left) or **"Add Tunnel"**
   - Select **"Import tunnel(s) from file..."**
   - Navigate to where you saved `client_wg0.conf`
   - Select the file
   - Click **Open**

3. **Verify import:**
   - You should see a new tunnel entry (e.g., "wg0" or "client_wg0")
   - The tunnel shows as **"Inactive"** (gray/red icon)

4. **Optional - Rename tunnel:**
   - Right-click the tunnel → **Edit**
   - Change name to: "R2D2 Jetson VPN"
   - Click **Save**

---

## Step 5: Configure Router Port Forwarding (5-10 minutes)

**⚠️ IMPORTANT:** This must be done on your router before the VPN will work!

### Find Your Router Admin Panel

1. **Find router IP:**
   - Usually: `192.168.1.1` or `192.168.0.1`
   - Check router label or documentation

2. **Access router:**
   - Open web browser
   - Go to: `http://192.168.1.1` (or your router IP)
   - Login with admin credentials

### Add Port Forwarding Rule

**Settings to configure:**
- **Service Name:** `WireGuard` or `R2D2 VPN`
- **Protocol:** `UDP` (NOT TCP!)
- **External Port:** `51820`
- **Internal IP:** `192.168.1.129` (Jetson's local IP - check with `ip addr` on Jetson)
- **Internal Port:** `51820`
- **Enable:** `Yes` or `Enabled`

**Router-specific instructions:**
- See `012_VPN_SETUP_AND_REMOTE_ACCESS.md` for detailed router guides
- Common brands: Netgear, TP-Link, ASUS, Linksys

### Save and Apply

1. **Click "Save"** or **"Apply"**
2. **Wait for router to restart** (may take 30-60 seconds)
3. **Verify rule is active**

---

## Step 6: Connect to VPN (1 minute)

1. **Open WireGuard** on Windows

2. **Activate tunnel:**
   - Find your tunnel (e.g., "R2D2 Jetson VPN")
   - Click the **toggle switch** or **"Activate"** button
   - Status should change to **"Active"** (green icon)

3. **Verify connection:**
   - Look for data transfer indicators (bytes sent/received)
   - Icon should be **green**
   - Status shows **"Active"**

4. **Check connection details:**
   - Right-click tunnel → **Details**
   - Should show:
     - **Latest handshake:** Recent timestamp (< 30 seconds)
     - **Transfer:** Data sent/received
     - **Endpoint:** Your public IP:51820

---

## Step 7: Test Connection (2 minutes)

### Test 1: Ping VPN Server

1. **Open PowerShell** (Windows Key + X → Windows PowerShell)

2. **Ping Jetson via VPN:**
   ```powershell
   ping 10.8.0.1
   ```
   Should see replies like:
   ```
   Reply from 10.8.0.1: bytes=32 time=XXms TTL=64
   ```

### Test 2: SSH Connection

1. **Test SSH:**
   ```powershell
   ssh severin@10.8.0.1
   ```
   Should connect successfully (may prompt for password or use SSH key)

2. **Or use your existing SSH config:**
   - If you have VS Code Remote SSH configured, it should work automatically
   - VS Code will connect through the VPN tunnel

### Test 3: VS Code Remote SSH

1. **Open VS Code** on Windows

2. **Connect to remote:**
   - Press `F1` or `Ctrl+Shift+P`
   - Type: "Remote-SSH: Connect to Host"
   - Select your Jetson host (should work automatically through VPN)

3. **Verify connection:**
   - Should connect successfully
   - Can browse files, run commands, etc.

---

## Step 8: Secure Cleanup (1 minute)

**⚠️ SECURITY:** After successful import, delete the client config file from your laptop!

1. **Verify VPN is working** (see Step 7)

2. **Delete client config file:**
   - WireGuard has already imported and stored the config internally
   - Delete `client_wg0.conf` from your laptop
   - Or move it to KeePass Notes for backup (see Secure Storage Guide)

3. **Verify in KeePass:**
   - Make sure all VPN credentials are stored in KeePass
   - Client config contents are in KeePass Notes

---

## Troubleshooting

### Problem: WireGuard won't connect

**Symptoms:**
- Tunnel shows "Inactive" or red icon
- No handshake in connection details

**Solutions:**

1. **Check router port forwarding:**
   - Verify UDP 51820 is forwarded correctly
   - Test with online port checker: https://www.yougetsignal.com/tools/open-ports/
   - Port should show as "Open" for UDP 51820

2. **Verify public IP in config:**
   - Make sure `Endpoint` in config has correct public IP
   - Get current IP: Run `get_public_ip.sh` on Jetson

3. **Check WireGuard service on Jetson:**
   ```bash
   # On Jetson
   sudo systemctl status wg-quick@wg0
   sudo wg show
   ```

4. **Check Windows Firewall:**
   - Windows Firewall may block WireGuard
   - Allow WireGuard through firewall when prompted

### Problem: Can ping but can't SSH

**Symptoms:**
- `ping 10.8.0.1` works
- `ssh severin@10.8.0.1` fails

**Solutions:**

1. **Check SSH is running on Jetson:**
   ```bash
   # On Jetson
   sudo systemctl status ssh
   ```

2. **Verify SSH port:**
   - Default is port 22
   - Should work automatically through VPN

3. **Check SSH key:**
   - Make sure your SSH key is authorized on Jetson
   - Or use password authentication

### Problem: Connection drops frequently

**Symptoms:**
- VPN connects but disconnects after a few minutes
- Handshake shows old timestamp

**Solutions:**

1. **Increase keepalive:**
   - Edit tunnel in WireGuard → Advanced
   - Increase `PersistentKeepalive` to 60 seconds

2. **Check router NAT timeout:**
   - Some routers have aggressive NAT timeouts
   - Enable "NAT Keep-Alive" if available

3. **Check internet connection:**
   - Verify stable internet on both ends

---

## Daily Usage

### Connecting to VPN

1. **Open WireGuard** on Windows
2. **Click toggle** to activate tunnel
3. **Wait for green icon** (connected)
4. **Use SSH/VS Code** as normal

### Disconnecting from VPN

1. **Open WireGuard** on Windows
2. **Click toggle** to deactivate tunnel
3. **Icon turns gray** (disconnected)

### Auto-Connect on Startup (Optional)

1. **Right-click tunnel** in WireGuard
2. **Select "Start on Boot"** or "Auto-Connect"
3. **VPN will connect automatically** when Windows starts

---

## Quick Reference

### WireGuard Commands (Windows)

- **Activate:** Click toggle switch in WireGuard app
- **Deactivate:** Click toggle switch again
- **View Details:** Right-click tunnel → Details
- **Edit Config:** Right-click tunnel → Edit

### Network Addresses

- **VPN Server (Jetson):** `10.8.0.1`
- **VPN Client (Windows):** `10.8.0.2`
- **SSH via VPN:** `ssh severin@10.8.0.1`

### File Locations

- **WireGuard Config:** Stored internally by WireGuard app
- **Original Config:** Should be deleted after import (or stored in KeePass)

---

## Success Checklist

- [ ] WireGuard installed on Windows
- [ ] Client config file obtained from Jetson
- [ ] Public IP updated in client config
- [ ] Config imported into WireGuard
- [ ] Router port forwarding configured
- [ ] VPN connection active (green icon)
- [ ] Can ping `10.8.0.1`
- [ ] Can SSH to `10.8.0.1`
- [ ] VS Code Remote SSH works
- [ ] Client config deleted from laptop (or in KeePass)
- [ ] All credentials stored in KeePass

---

**Created:** December 10, 2025  
**Related Documents:**
- `012_VPN_SETUP_AND_REMOTE_ACCESS.md` - Full VPN documentation
- `SECURE_STORAGE_GUIDE.md` - How to store credentials in KeePass

