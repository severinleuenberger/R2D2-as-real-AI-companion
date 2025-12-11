# VPN Setup and Remote Access Guide

**Date:** December 10, 2025  
**Platform:** NVIDIA Jetson AGX Orin 64GB (Ubuntu 22.04)  
**Purpose:** Secure remote access to Jetson Orin over public Internet  
**VPN Technology:** WireGuard (UDP port 51820)

---

## Executive Summary

This guide establishes an encrypted VPN tunnel from your Windows laptop to the Jetson Orin, allowing you to:
- ✅ Access Jetson via SSH (VS Code Remote SSH, terminal) as if on local network
- ✅ Keep your existing SSH workflow unchanged
- ✅ Secure connection (no direct SSH exposure to Internet)
- ✅ Low latency, stable connection

**Estimated Setup Time:** 15-30 minutes (quick setup)  
**Prerequisites:** Router access, public IP or dynamic DNS

---

## ⚠️ Security Notice

**IMPORTANT:** This setup generates sensitive cryptographic keys that must be kept secure:
- ✅ **Never commit** `.key` files or `client_wg0.conf` to git
- ✅ **Store all keys** in KeePass (encrypted password manager)
- ✅ **Delete client config** from laptop after importing to WireGuard

**See:** 
- `vpn_config/SECURE_STORAGE_GUIDE.md` - How to store credentials in KeePass
- `vpn_config/WINDOWS_SETUP_INSTRUCTIONS.md` - Windows laptop setup guide

---

## Quick Start (Get Connected in 15 Minutes)

### Step 1: Install WireGuard on Jetson (2 min)

```bash
cd /home/severin/dev/r2d2/vpn_config
sudo bash setup_wireguard.sh
```

This script will:
- Install WireGuard and tools
- Generate server and client keys
- Create server configuration (`/etc/wireguard/wg0.conf`)
- Create client configuration (`client_wg0.conf`)
- Enable IP forwarding

**Expected Output:**
```
=== Setup Complete! ===
Server Public Key: [key]
Client Public Key: [key]
IMPORTANT: Update client_wg0.conf with your public IP address!
```

### Step 2: Get Your Public IP Address (1 min)

```bash
cd /home/severin/dev/r2d2/vpn_config
bash get_public_ip.sh
```

**Note:** If you have a dynamic IP, consider setting up dynamic DNS (see Troubleshooting section).

### Step 3: Update Client Configuration (2 min)

Edit the client configuration file:

```bash
nano /home/severin/dev/r2d2/vpn_config/client_wg0.conf
```

Replace `YOUR_PUBLIC_IP` with your actual public IP address (from Step 2).

**Example:**
```ini
[Peer]
Endpoint = 123.45.67.89:51820  # Your public IP here
```

### Step 4: Configure Router Port Forwarding (5-10 min)

**Critical:** Your router must forward UDP port 51820 to the Jetson's local IP.

1. **Find Jetson's local IP:**
   ```bash
   ip addr show | grep "inet " | grep -v 127.0.0.1
   ```
   Example: `192.168.1.129`

2. **Access router admin panel:**
   - Usually: `http://192.168.1.1` or `http://192.168.0.1`
   - Check router label or documentation

3. **Add port forwarding rule:**
   - **Service Name:** WireGuard (or R2D2 VPN)
   - **Protocol:** UDP
   - **External Port:** 51820
   - **Internal IP:** `192.168.1.129` (Jetson's IP)
   - **Internal Port:** 51820
   - **Enable:** Yes

4. **Save and apply changes**

**Router Examples:**
- **Netgear:** Advanced → Port Forwarding → Add Custom Service
- **TP-Link:** Advanced → NAT Forwarding → Port Forwarding
- **ASUS:** WAN → Virtual Server / Port Forwarding
- **Linksys:** Connectivity → Port Forwarding

### Step 5: Start WireGuard Service on Jetson (1 min)

```bash
sudo systemctl enable wg-quick@wg0
sudo systemctl start wg-quick@wg0
sudo systemctl status wg-quick@wg0
```

**Verify it's running:**
```bash
sudo wg show
```

You should see the `wg0` interface with server configuration.

### Step 6: Transfer Client Config to Windows Laptop (2 min)

**⚠️ SECURITY:** The client config contains your private key. Transfer securely!

**Option A: Copy file contents manually (Recommended)**
```bash
cat /home/severin/dev/r2d2/vpn_config/client_wg0.conf
```
Copy the entire output and save as `client_wg0.conf` on your Windows laptop.

**Option B: Use USB drive**
- Copy file to USB drive on Jetson
- Transfer to Windows laptop

**Option C: Store in KeePass first**
- Copy contents to KeePass Notes (see `SECURE_STORAGE_GUIDE.md`)
- Copy from KeePass to Windows when needed

**Option D: Use SCP (if you have temporary SSH access)**
```bash
# From Windows (PowerShell or WSL)
scp severin@192.168.1.129:/home/severin/dev/r2d2/vpn_config/client_wg0.conf .
```

**After transfer:** Delete the file from laptop after importing to WireGuard (WireGuard stores it internally).

### Step 7: Install WireGuard Client on Windows (3 min)

**📋 See detailed instructions:** `vpn_config/WINDOWS_SETUP_INSTRUCTIONS.md`

**Quick steps:**
1. **Install WireGuard:**
   - Microsoft Store: Search "WireGuard" → Install
   - Or: https://www.wireguard.com/install/

2. **Import configuration:**
   - Open WireGuard app
   - Click "Add Tunnel" → "Import tunnel(s) from file"
   - Select `client_wg0.conf`
   - Click "Activate"

3. **Verify connection:**
   - Status should show "Active" with data transfer
   - Green icon indicates connected

4. **⚠️ SECURITY:** Delete `client_wg0.conf` from laptop after successful import (WireGuard stores it internally)

### Step 8: Test SSH Connection (1 min)

**From Windows (PowerShell or Command Prompt):**
```bash
# Test ping to Jetson via VPN
ping 10.8.0.1

# Test SSH connection
ssh severin@10.8.0.1
# Or use your existing SSH config (it should work automatically)
```

**VS Code Remote SSH:**
- Your existing SSH config should work automatically
- VS Code will connect through the VPN tunnel
- No changes needed to your SSH configuration

---

## Verification & Testing

### Check VPN Status on Jetson

```bash
cd /home/severin/dev/r2d2/vpn_config
bash check_vpn_status.sh
```

**Expected Output:**
```
✅ WireGuard service is running
✅ 1 peer(s) connected
```

### Test Connection from Windows

1. **Ping test:**
   ```powershell
   ping 10.8.0.1
   ```

2. **SSH test:**
   ```powershell
   ssh severin@10.8.0.1
   ```

3. **VS Code Remote SSH:**
   - Open VS Code
   - Connect to existing SSH target
   - Should connect through VPN automatically

### Monitor Connection

**On Jetson:**
```bash
# Watch real-time connection stats
watch -n 1 'sudo wg show'

# Check service logs
sudo journalctl -u wg-quick@wg0 -f
```

**On Windows:**
- WireGuard app shows connection status and data transfer
- Green icon = connected, red = disconnected

---

## Network Configuration Details

### VPN Network Topology

```
Internet
    ↓
Router (Public IP: YOUR_PUBLIC_IP)
    ↓ (UDP 51820 forwarded)
Jetson Orin (Local IP: 192.168.1.129)
    ↓ (WireGuard Server: 10.8.0.1)
VPN Tunnel (10.8.0.0/24)
    ↓
Windows Laptop (VPN IP: 10.8.0.2)
```

### IP Addresses

| Device | IP Address | Purpose |
|--------|------------|---------|
| **Jetson (VPN Server)** | `10.8.0.1` | WireGuard server IP |
| **Windows (VPN Client)** | `10.8.0.2` | WireGuard client IP |
| **Jetson (Local Network)** | `192.168.1.129` | Local network IP (may vary) |
| **Router (Public)** | `YOUR_PUBLIC_IP` | Internet-facing IP |

### Port Configuration

- **WireGuard Port:** UDP 51820 (standard)
- **SSH Port:** TCP 22 (unchanged, now accessed via VPN)

---

## Troubleshooting

### Problem: WireGuard service won't start

**Symptoms:**
```bash
sudo systemctl status wg-quick@wg0
# Shows: failed or inactive
```

**Solutions:**

1. **Check configuration syntax:**
   ```bash
   sudo wg-quick up wg0
   ```
   Look for error messages.

2. **Verify interface name:**
   ```bash
   # Check if interface exists
   ip link show wg0
   ```

3. **Check for port conflicts:**
   ```bash
   sudo netstat -ulnp | grep 51820
   ```
   If another service is using port 51820, change it in `/etc/wireguard/wg0.conf`.

4. **Check logs:**
   ```bash
   sudo journalctl -u wg-quick@wg0 -n 50
   ```

### Problem: Client can't connect to server

**Symptoms:**
- WireGuard client shows "No handshake" or red status
- Can't ping `10.8.0.1`

**Solutions:**

1. **Verify router port forwarding:**
   - Check router admin panel
   - Ensure UDP 51820 is forwarded to Jetson's local IP
   - Test with: https://www.yougetsignal.com/tools/open-ports/ (port 51820)

2. **Check firewall on Jetson:**
   ```bash
   # Allow WireGuard port
   sudo ufw allow 51820/udp
   sudo ufw status
   ```

3. **Verify public IP in client config:**
   - Make sure `Endpoint` in `client_wg0.conf` has correct public IP
   - If using dynamic IP, update it when it changes

4. **Check server is running:**
   ```bash
   sudo systemctl status wg-quick@wg0
   sudo wg show
   ```

5. **Test from external network:**
   ```bash
   # From Windows (outside your network, e.g., mobile hotspot)
   # Try connecting - if it works, router forwarding is correct
   ```

### Problem: Can connect to VPN but can't SSH

**Symptoms:**
- VPN shows connected (green)
- Can ping `10.8.0.1`
- SSH connection times out

**Solutions:**

1. **Verify SSH is running on Jetson:**
   ```bash
   sudo systemctl status ssh
   ```

2. **Check SSH is listening on VPN interface:**
   ```bash
   sudo netstat -tlnp | grep :22
   ```
   Should show SSH listening on `0.0.0.0:22` (all interfaces).

3. **Test SSH locally first:**
   ```bash
   # On Jetson
   ssh localhost
   ```

4. **Check firewall rules:**
   ```bash
   sudo ufw status
   sudo ufw allow 22/tcp
   ```

5. **Verify routing:**
   ```bash
   # On Windows, check route to Jetson
   route print | findstr 10.8.0
   ```

### Problem: Dynamic IP address changes

**Symptoms:**
- VPN works initially, then stops working
- Public IP changes periodically

**Solutions:**

1. **Set up Dynamic DNS (Recommended):**
   - Use services like: No-IP, DuckDNS, or Dynu
   - Update router or use DDNS client on Jetson
   - Update `client_wg0.conf` to use hostname instead of IP

2. **Manual update:**
   ```bash
   # Get new IP
   bash /home/severin/dev/r2d2/vpn_config/get_public_ip.sh
   
   # Update client config
   nano /home/severin/dev/r2d2/vpn_config/client_wg0.conf
   # Change Endpoint to new IP
   
   # Re-import on Windows WireGuard client
   ```

3. **Automated script (optional):**
   Create a cron job to check and update IP (see Advanced section).

### Problem: Connection is slow or unstable

**Symptoms:**
- High latency
- Frequent disconnections
- Slow file transfers

**Solutions:**

1. **Check connection quality:**
   ```bash
   # On Jetson
   sudo wg show wg0
   # Look at "latest handshake" - should be recent (< 30 seconds)
   ```

2. **Adjust keepalive:**
   - In `client_wg0.conf`, `PersistentKeepalive` is set to 25 seconds
   - Increase if behind strict NAT: `PersistentKeepalive = 60`

3. **Check router NAT settings:**
   - Some routers have aggressive NAT timeouts
   - Enable "NAT Keep-Alive" or similar setting

4. **Test from different network:**
   - Try from mobile hotspot to isolate router issues

### Problem: Can't access local network resources through VPN

**Symptoms:**
- VPN connected, can SSH to Jetson
- Can't access other devices on Jetson's local network

**Solutions:**

1. **This is expected behavior** - WireGuard only routes traffic to VPN network by default
2. **To access local network, modify server config:**
   ```bash
   sudo nano /etc/wireguard/wg0.conf
   ```
   Add to `[Peer]` section:
   ```ini
   AllowedIPs = 10.8.0.2/32, 192.168.1.0/24
   ```
   Then restart: `sudo systemctl restart wg-quick@wg0`

### Problem: Multiple clients / Add more devices

**To add additional clients (e.g., phone, second laptop):**

1. **Generate new client keys:**
   ```bash
   cd /etc/wireguard
   wg genkey | tee client2_private.key | wg pubkey > client2_public.key
   ```

2. **Add peer to server config:**
   ```bash
   sudo nano /etc/wireguard/wg0.conf
   ```
   Add new `[Peer]` section:
   ```ini
   [Peer]
   PublicKey = [client2_public_key]
   AllowedIPs = 10.8.0.3/32
   ```

3. **Create client config:**
   ```bash
   # Use client2_private.key and assign IP 10.8.0.3
   ```

4. **Restart service:**
   ```bash
   sudo systemctl restart wg-quick@wg0
   ```

---

## Router Configuration Guide

### Common Router Brands

#### Netgear
1. Login: `http://192.168.1.1` or `http://routerlogin.net`
2. Advanced → Port Forwarding → Add Custom Service
3. Service Name: `WireGuard`
4. External Starting Port: `51820`
5. External Ending Port: `51820`
6. Internal Starting Port: `51820`
7. Internal Ending Port: `51820`
8. Internal IP Address: `192.168.1.129` (Jetson IP)
9. Protocol: `UDP`
10. Apply

#### TP-Link
1. Login: `http://192.168.0.1` or `http://tplinkwifi.net`
2. Advanced → NAT Forwarding → Port Forwarding
3. Add new rule:
   - Service Name: `WireGuard`
   - External Port: `51820`
   - Internal Port: `51820`
   - Protocol: `UDP`
   - Internal IP: `192.168.1.129`
4. Status: `Enabled`
5. Save

#### ASUS
1. Login: `http://192.168.1.1` or `http://router.asus.com`
2. WAN → Virtual Server / Port Forwarding
3. Enable Port Forwarding: `Yes`
4. Add:
   - Service Name: `WireGuard`
   - Port Range: `51820`
   - Local IP: `192.168.1.129`
   - Local Port: `51820`
   - Protocol: `UDP`
5. Apply

#### Linksys
1. Login: `http://192.168.1.1`
2. Connectivity → Port Forwarding
3. Add:
   - Application Name: `WireGuard`
   - External Port: `51820`
   - Internal Port: `51820`
   - Protocol: `UDP`
   - Device IP: `192.168.1.129`
4. Apply

#### Generic Router
Look for settings named:
- Port Forwarding
- Virtual Server
- NAT Forwarding
- Port Mapping
- Applications & Gaming

**Key Settings:**
- Protocol: **UDP** (not TCP!)
- External Port: `51820`
- Internal Port: `51820`
- Internal IP: Jetson's local IP (e.g., `192.168.1.129`)

---

## Advanced Configuration

### Dynamic DNS Setup

If your public IP changes frequently, use Dynamic DNS:

1. **Sign up for free DDNS service:**
   - No-IP: https://www.noip.com/
   - DuckDNS: https://www.duckdns.org/
   - Dynu: https://www.dynu.com/

2. **Configure on router (if supported):**
   - Most routers have DDNS settings
   - Enter your DDNS credentials
   - Router will update IP automatically

3. **Or use client on Jetson:**
   ```bash
   # Install ddclient
   sudo apt install ddclient
   
   # Configure with your DDNS provider
   sudo nano /etc/ddclient.conf
   ```

4. **Update client config:**
   ```ini
   [Peer]
   Endpoint = your-hostname.ddns.net:51820
   ```

### Firewall Configuration

**UFW (Uncomplicated Firewall):**
```bash
# Allow WireGuard
sudo ufw allow 51820/udp

# Allow SSH (if not already)
sudo ufw allow 22/tcp

# Check status
sudo ufw status
```

**iptables (if using custom rules):**
The setup script already configures NAT forwarding. If you have custom iptables rules, ensure they don't block WireGuard.

### Performance Tuning

**Increase MTU (if experiencing slow speeds):**
```bash
sudo nano /etc/wireguard/wg0.conf
```
Add to `[Interface]` section:
```ini
MTU = 1420
```

**Adjust keepalive (for unstable connections):**
In `client_wg0.conf`:
```ini
PersistentKeepalive = 60  # Increase from 25 to 60
```

### Security Best Practices

1. **Keep keys secure:**
   - Never share private keys
   - Use `chmod 600` on all `.key` files
   - Don't commit keys to git

2. **Regular updates:**
   ```bash
   sudo apt update && sudo apt upgrade wireguard
   ```

3. **Monitor connections:**
   ```bash
   sudo wg show
   # Check for unexpected peers
   ```

4. **Use strong keys:**
   - WireGuard generates strong keys by default
   - Don't use weak or predictable keys

---

## Maintenance

### Daily Operations

**Check VPN status:**
```bash
cd /home/severin/dev/r2d2/vpn_config
bash check_vpn_status.sh
```

**Restart service (if needed):**
```bash
sudo systemctl restart wg-quick@wg0
```

**View connection stats:**
```bash
sudo wg show wg0
```

### Updates

**Update WireGuard:**
```bash
sudo apt update && sudo apt upgrade wireguard
sudo systemctl restart wg-quick@wg0
```

**Update client config (if server IP changes):**
1. Get new public IP: `bash get_public_ip.sh`
2. Update `client_wg0.conf` on Windows
3. Re-import in WireGuard client

### Backup

**Backup server configuration:**
```bash
# Backup keys and config
sudo cp /etc/wireguard/wg0.conf /home/severin/dev/r2d2/vpn_config/wg0.conf.backup
sudo cp /etc/wireguard/*.key /home/severin/dev/r2d2/vpn_config/keys_backup/
```

**⚠️ Important:** Keep backups secure! These contain private keys.

---

## Quick Reference

### Essential Commands

```bash
# Start VPN
sudo systemctl start wg-quick@wg0

# Stop VPN
sudo systemctl stop wg-quick@wg0

# Restart VPN
sudo systemctl restart wg-quick@wg0

# Check status
sudo systemctl status wg-quick@wg0
sudo wg show

# View logs
sudo journalctl -u wg-quick@wg0 -f

# Check connection
cd /home/severin/dev/r2d2/vpn_config
bash check_vpn_status.sh
```

### File Locations

| File | Location | Purpose |
|------|----------|---------|
| **Server Config** | `/etc/wireguard/wg0.conf` | WireGuard server configuration |
| **Server Keys** | `/etc/wireguard/server_*.key` | Server private/public keys |
| **Client Config** | `/home/severin/dev/r2d2/vpn_config/client_wg0.conf` | Client configuration (transfer to Windows) |
| **Setup Script** | `/home/severin/dev/r2d2/vpn_config/setup_wireguard.sh` | Automated setup script |
| **Status Script** | `/home/severin/dev/r2d2/vpn_config/check_vpn_status.sh` | Status checking script |

### Network Addresses

- **VPN Server (Jetson):** `10.8.0.1`
- **VPN Client (Windows):** `10.8.0.2`
- **WireGuard Port:** `51820/UDP`
- **SSH Port:** `22/TCP` (accessed via VPN)

---

## Support & Resources

### Documentation
- **WireGuard Official:** https://www.wireguard.com/
- **WireGuard Quick Start:** https://www.wireguard.com/quickstart/
- **R2D2 Project:** See `001_ARCHITECTURE_OVERVIEW.md` for system context

### Common Issues
- See **Troubleshooting** section above
- Check WireGuard logs: `sudo journalctl -u wg-quick@wg0 -n 50`
- Verify router port forwarding with online port checker

### Getting Help
1. Check troubleshooting section
2. Review WireGuard logs
3. Test connection from different network
4. Verify router configuration

---

## Success Criteria

✅ **VPN tunnel establishes successfully from laptop**  
✅ **SSH connection works through VPN (VS Code Remote SSH functional)**  
✅ **No direct SSH exposure to Internet**  
✅ **Connection stable and low-latency**

---

**Created:** December 10, 2025  
**Last Updated:** December 10, 2025  
**Related Documents:**
- `000_INTERNAL_AGENT_NOTES.md` - Development environment setup
- `001_ARCHITECTURE_OVERVIEW.md` - System architecture
- `010_PROJECT_GOALS_AND_SETUP.md` - Project setup guide

