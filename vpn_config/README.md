# WireGuard VPN Configuration Files

This directory contains all files needed to set up WireGuard VPN on the Jetson Orin.

## Quick Start

1. **Run setup script (requires sudo):**
   ```bash
   sudo bash setup_wireguard.sh
   ```

2. **Get your public IP:**
   ```bash
   bash get_public_ip.sh
   ```

3. **Update client config with public IP:**
   ```bash
   nano client_wg0.conf
   # Replace YOUR_PUBLIC_IP with actual IP
   ```

4. **Start WireGuard service:**
   ```bash
   sudo systemctl enable wg-quick@wg0
   sudo systemctl start wg-quick@wg0
   ```

5. **Configure router port forwarding:**
   - UDP port 51820 → Jetson's local IP (192.168.1.129)

6. **Transfer `client_wg0.conf` to Windows laptop and import into WireGuard client**

## Files

- `setup_wireguard.sh` - Automated setup script (run with sudo)
- `get_public_ip.sh` - Get your public IP address
- `check_vpn_status.sh` - Check VPN connection status
- `client_wg0.conf` - Client configuration (transfer to Windows)
- `README.md` - This file

## Documentation

Full documentation: `../012_VPN_SETUP_AND_REMOTE_ACCESS.md`

## Troubleshooting

```bash
# Check status
bash check_vpn_status.sh

# View WireGuard status
sudo wg show

# Check service logs
sudo journalctl -u wg-quick@wg0 -f
```

