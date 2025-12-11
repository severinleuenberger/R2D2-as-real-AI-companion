# Next Steps After Running setup_wireguard.sh

After the setup script completes, follow these steps in order:

---

## Step 1: Verify Setup Completed ✅

Check that these files were created:

```bash
# Check secure directory exists
ls -la ~/.r2d2_vpn_secrets/

# Check client config exists
ls -la /home/severin/dev/r2d2/vpn_config/client_wg0.conf

# Check WireGuard service files
sudo ls -la /etc/wireguard/
```

**Expected:**
- `~/.r2d2_vpn_secrets/vpn_keys.txt` - All keys and config
- `~/.r2d2_vpn_secrets/client_wg0.conf.backup` - Client config backup
- `/home/severin/dev/r2d2/vpn_config/client_wg0.conf` - Client config (needs public IP update)
- `/etc/wireguard/wg0.conf` - Server config
- `/etc/wireguard/*.key` - Server and client keys

---

## Step 2: Get Your Public IP Address

```bash
cd /home/severin/dev/r2d2/vpn_config
bash get_public_ip.sh
```

**Note the IPv4 address** (e.g., `188.61.209.189`) - you'll need it in the next step.

---

## Step 3: Update Client Config with Public IP

The client config file has `YOUR_PUBLIC_IP` as a placeholder. Replace it with your actual public IP:

```bash
nano /home/severin/dev/r2d2/vpn_config/client_wg0.conf
```

**Find this line:**
```ini
Endpoint = YOUR_PUBLIC_IP:51820
```

**Replace with your actual public IP:**
```ini
Endpoint = 188.61.209.189:51820
```

**Save and exit:** `Ctrl+X`, then `Y`, then `Enter`

**Verify the change:**
```bash
grep "Endpoint" /home/severin/dev/r2d2/vpn_config/client_wg0.conf
```
Should show your actual IP, not `YOUR_PUBLIC_IP`.

---

## Step 4: Start WireGuard Service

```bash
# Enable service to start on boot
sudo systemctl enable wg-quick@wg0

# Start the service now
sudo systemctl start wg-quick@wg0

# Check status
sudo systemctl status wg-quick@wg0
```

**Expected output:** Should show `active (running)` in green.

**Verify WireGuard is running:**
```bash
sudo wg show
```

**Expected output:** Should show the `wg0` interface with server configuration.

---

## Step 5: Store Keys in KeePass (On Windows Laptop)

**⚠️ IMPORTANT:** Do this before proceeding with router setup!

1. **View the sensitive information:**
   ```bash
   cat ~/.r2d2_vpn_secrets/vpn_keys.txt
   ```

2. **Copy everything** from the output

3. **On Windows laptop:**
   - Open KeePass
   - Create new entry: "R2D2 VPN - Jetson Orin"
   - Paste all information into Notes or Custom Fields
   - Save KeePass database

**See:** `SECURE_STORAGE_GUIDE.md` for detailed instructions

---

## Step 6: Configure Router Port Forwarding

**⚠️ CRITICAL:** This must be done for VPN to work from outside your network!

1. **Find Jetson's local IP:**
   ```bash
   ip addr show | grep "inet " | grep -v 127.0.0.1
   ```
   Example: `192.168.1.129`

2. **Access router admin:**
   - Usually: `http://192.168.1.1` or `http://192.168.0.1`
   - Login with admin credentials

3. **Add port forwarding rule:**
   - **Service Name:** `WireGuard` or `R2D2 VPN`
   - **Protocol:** `UDP` (NOT TCP!)
   - **External Port:** `51820`
   - **Internal IP:** `192.168.1.129` (Jetson's local IP)
   - **Internal Port:** `51820`
   - **Enable:** `Yes`

4. **Save and apply**

**See:** `012_VPN_SETUP_AND_REMOTE_ACCESS.md` → Router Configuration Guide for brand-specific instructions

---

## Step 7: Test VPN Connection (From Jetson)

Before testing from Windows, verify the service is ready:

```bash
# Check service status
sudo systemctl status wg-quick@wg0

# Check WireGuard interface
sudo wg show

# Check VPN status script
cd /home/severin/dev/r2d2/vpn_config
bash check_vpn_status.sh
```

**Expected:**
- Service: `active (running)`
- Interface: `wg0` shows server config
- Status script: Shows service running (may show no peers yet - that's OK)

---

## Step 8: Transfer Client Config to Windows

**Choose one method:**

### Method A: Copy Contents (Recommended)
```bash
cat /home/severin/dev/r2d2/vpn_config/client_wg0.conf
```
Copy the entire output and save as `client_wg0.conf` on Windows.

### Method B: From KeePass
If you stored it in KeePass, copy from there.

### Method C: USB Drive
```bash
# Copy to USB (mount USB first)
cp /home/severin/dev/r2d2/vpn_config/client_wg0.conf /media/usb/
```

---

## Step 9: Windows Laptop Setup

**Follow:** `WINDOWS_SETUP_INSTRUCTIONS.md`

**Quick summary:**
1. Install WireGuard from Microsoft Store
2. Import `client_wg0.conf` into WireGuard
3. Activate tunnel
4. Test connection: `ping 10.8.0.1`
5. Test SSH: `ssh severin@10.8.0.1`
6. **Delete `client_wg0.conf` from laptop** after successful import

---

## Step 10: Final Verification

**On Windows:**
- [ ] WireGuard shows "Active" (green icon)
- [ ] Can ping `10.8.0.1`
- [ ] Can SSH to `10.8.0.1`
- [ ] VS Code Remote SSH works
- [ ] Client config deleted from laptop

**On Jetson:**
```bash
# Check for connected peers
sudo wg show

# Should show client connected with recent handshake
```

---

## Troubleshooting

If something doesn't work:

1. **Check service logs:**
   ```bash
   sudo journalctl -u wg-quick@wg0 -n 50
   ```

2. **Verify router port forwarding:**
   - Test with online port checker: https://www.yougetsignal.com/tools/open-ports/
   - Port 51820 UDP should show as "Open"

3. **Check firewall:**
   ```bash
   sudo ufw status
   sudo ufw allow 51820/udp
   ```

4. **See:** `012_VPN_SETUP_AND_REMOTE_ACCESS.md` → Troubleshooting section

---

**Created:** December 10, 2025

