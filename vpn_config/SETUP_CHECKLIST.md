# VPN Setup Checklist

Use this checklist to track your progress through the VPN setup.

---

## 🔒 Security Preparation

- [ ] Understand that `.key` files and `client_wg0.conf` must NEVER be committed to git
- [ ] Have KeePass open and ready on Windows laptop
- [ ] Know where to store sensitive information in KeePass

**Reference:** `SECURE_STORAGE_GUIDE.md`

---

## 🖥️ Jetson Setup (Run on Jetson)

- [ ] Navigate to VPN config directory: `cd /home/severin/dev/r2d2/vpn_config`
- [ ] Run setup script: `sudo bash setup_wireguard.sh`
- [ ] Setup script completed successfully
- [ ] Sensitive info saved to `~/.r2d2_vpn_secrets/vpn_keys.txt`
- [ ] Get public IP: `bash get_public_ip.sh`
- [ ] Public IP noted: `_____________` (e.g., 188.61.209.189)
- [ ] Edit client config: `nano client_wg0.conf`
- [ ] Replaced `YOUR_PUBLIC_IP` with actual public IP
- [ ] Start WireGuard service: `sudo systemctl enable wg-quick@wg0 && sudo systemctl start wg-quick@wg0`
- [ ] Verify service running: `sudo systemctl status wg-quick@wg0`
- [ ] Check WireGuard status: `sudo wg show`

---

## 🔐 Store in KeePass (On Windows Laptop)

- [ ] Open KeePass database
- [ ] Create new entry: "R2D2 VPN - Jetson Orin"
- [ ] Copy Server Private Key from `~/.r2d2_vpn_secrets/vpn_keys.txt` → KeePass
- [ ] Copy Server Public Key → KeePass
- [ ] Copy Client Private Key → KeePass
- [ ] Copy Client Public Key → KeePass
- [ ] Store network configuration (IPs, ports) → KeePass
- [ ] Store public IP address → KeePass
- [ ] Copy client config file contents → KeePass Notes
- [ ] Save KeePass database

**Reference:** `SECURE_STORAGE_GUIDE.md`

---

## 🛠️ Router Configuration

- [ ] Access router admin panel (usually `http://192.168.1.1`)
- [ ] Find "Port Forwarding" or "Virtual Server" section
- [ ] Add new rule:
  - [ ] Service Name: `WireGuard` or `R2D2 VPN`
  - [ ] Protocol: `UDP` (NOT TCP!)
  - [ ] External Port: `51820`
  - [ ] Internal IP: `_____________` (Jetson's local IP, e.g., 192.168.1.129)
  - [ ] Internal Port: `51820`
  - [ ] Enable: `Yes`
- [ ] Save and apply router changes
- [ ] Wait for router to restart (30-60 seconds)
- [ ] Verify port forwarding is active

**Reference:** `012_VPN_SETUP_AND_REMOTE_ACCESS.md` → Router Configuration Guide

---

## 💻 Windows Laptop Setup

- [ ] Install WireGuard from Microsoft Store
- [ ] Get client config file from Jetson (copy contents)
- [ ] Create `client_wg0.conf` on Windows laptop
- [ ] Verify public IP is correct in config (not `YOUR_PUBLIC_IP`)
- [ ] Import config into WireGuard app
- [ ] Activate tunnel in WireGuard
- [ ] Verify connection shows "Active" (green icon)
- [ ] Test ping: `ping 10.8.0.1` (should work)
- [ ] Test SSH: `ssh severin@10.8.0.1` (should work)
- [ ] Test VS Code Remote SSH (should work)
- [ ] **SECURITY:** Delete `client_wg0.conf` from laptop after successful import

**Reference:** `WINDOWS_SETUP_INSTRUCTIONS.md`

---

## ✅ Verification

- [ ] VPN connection active (green icon in WireGuard)
- [ ] Can ping `10.8.0.1` from Windows
- [ ] Can SSH to `10.8.0.1` from Windows
- [ ] VS Code Remote SSH connects successfully
- [ ] All sensitive information stored in KeePass
- [ ] Client config deleted from laptop (or in KeePass)
- [ ] No `.key` files or `client_wg0.conf` in git repository

---

## 📝 Notes

**Public IP:** `_____________`  
**Jetson Local IP:** `_____________`  
**VPN Server IP:** `10.8.0.1`  
**VPN Client IP:** `10.8.0.2`  
**VPN Port:** `51820/UDP`

**KeePass Entry:** `R2D2 VPN - Jetson Orin`

---

**Created:** December 10, 2025

