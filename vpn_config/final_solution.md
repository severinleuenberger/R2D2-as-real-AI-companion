# WireGuard on Jetson - Final Solution Options

## Current Issue
The Jetson Orin kernel (5.15.148-tegra) doesn't have the WireGuard kernel module built-in, and wireguard-go is having issues creating the TUN device.

## Solution Options

### Option 1: Build WireGuard Kernel Module (Recommended if possible)
This requires kernel headers and DKMS:

```bash
sudo apt install -y linux-headers-$(uname -r) dkms wireguard-dkms
sudo modprobe wireguard
```

**If this works:** The original `wg-quick@wg0` service should work.

### Option 2: Use Tailscale (Easiest Alternative)
Tailscale is a WireGuard-based VPN that works without kernel modules:

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
```

**Pros:** 
- No kernel module needed
- Works on Jetson
- Easy setup
- Free for personal use

**Cons:**
- Requires Tailscale account
- Traffic goes through Tailscale servers (can be configured)

### Option 3: Use ZeroTier (Alternative)
Another VPN solution that works without kernel modules:

```bash
curl -s https://install.zerotier.com | sudo bash
sudo zerotier-cli join <network-id>
```

### Option 4: Manual TUN Setup for wireguard-go
Try creating TUN device manually:

```bash
sudo ip tuntap add mode tun wg0
sudo chmod 666 /dev/net/tun
sudo /usr/bin/wireguard wg0 &
sleep 2
sudo wg setconf wg0 /etc/wireguard/wg0.conf
```

### Option 5: Reboot (Sometimes helps)
Sometimes modules load after reboot:

```bash
sudo reboot
# After reboot:
sudo systemctl start wg-quick@wg0
```

## Recommendation
Try **Option 1** first (build kernel module). If that fails, use **Option 2** (Tailscale) as it's the easiest and most reliable on Jetson.

