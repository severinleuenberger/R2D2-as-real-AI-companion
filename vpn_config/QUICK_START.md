# VPN Quick Start - 5 Minute Setup

## Step-by-Step (Run these commands)

```bash
# 1. Navigate to VPN config directory
cd /home/severin/dev/r2d2/vpn_config

# 2. Run setup script (requires sudo password)
sudo bash setup_wireguard.sh

# 3. Get your public IP address
bash get_public_ip.sh

# 4. Edit client config with your public IP
nano client_wg0.conf
# Replace "YOUR_PUBLIC_IP" with the IP from step 3

# 5. Start WireGuard service
sudo systemctl enable wg-quick@wg0
sudo systemctl start wg-quick@wg0

# 6. Verify it's running
sudo wg show
```

## Next Steps (On Your Router)

1. Access router admin (usually `http://192.168.1.1`)
2. Find "Port Forwarding" or "Virtual Server"
3. Add rule:
   - **Protocol:** UDP
   - **External Port:** 51820
   - **Internal IP:** 192.168.1.129 (Jetson's IP)
   - **Internal Port:** 51820
4. Save and apply

## On Windows Laptop

1. Install WireGuard from Microsoft Store
2. Copy `client_wg0.conf` to your laptop
3. Open WireGuard → Add Tunnel → Import from file
4. Click "Activate"
5. Test: `ping 10.8.0.1` and `ssh severin@10.8.0.1`

## Troubleshooting

```bash
# Check status
bash check_vpn_status.sh

# View logs
sudo journalctl -u wg-quick@wg0 -n 20
```

**Full documentation:** `../012_VPN_SETUP_AND_REMOTE_ACCESS.md`

