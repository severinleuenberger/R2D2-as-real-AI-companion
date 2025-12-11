# WireGuard Removal Checklist

**Date:** December 11, 2025  
**Purpose:** Step-by-step checklist for WireGuard removal

---

## ⚠️ Pre-Removal: Store Keys in KeePass (If Needed)

**Before removing WireGuard, store any keys you want to keep:**

1. **Check if secure storage exists:**
   ```bash
   cat ~/.r2d2_vpn_secrets/vpn_keys.txt
   ```

2. **If it exists, copy to KeePass:**
   - Open KeePass on Windows
   - Create entry: "R2D2 VPN - WireGuard Keys (Backup)"
   - Copy all content from `vpn_keys.txt` to KeePass Notes
   - Save KeePass database

3. **Note:** These keys are no longer needed since we're using Tailscale, but you can keep them as backup.

---

## Removal Steps

### Option A: Automated Removal (Recommended)

1. **Review the removal script:**
   ```bash
   cat /home/severin/dev/r2d2/vpn_config/remove_wireguard.sh
   ```

2. **Run the automated script:**
   ```bash
   cd /home/severin/dev/r2d2/vpn_config
   sudo bash remove_wireguard.sh
   ```

3. **Type 'yes' when prompted**

4. **Verify removal completed successfully**

### Option B: Manual Removal

Follow the steps in `WIREGUARD_CLEANUP_GUIDE.md` section by section.

---

## Post-Removal Verification

After removal, verify:

- [ ] No WireGuard packages installed: `dpkg -l | grep wireguard` (should show nothing)
- [ ] No WireGuard services: `systemctl list-units | grep wireguard` (should show nothing)
- [ ] No config directory: `ls /etc/wireguard/` (should show "No such file")
- [ ] No key files: `find vpn_config -name "*.key"` (should show nothing)
- [ ] Tailscale still works: `tailscale status` (should show connected devices)
- [ ] SSH through Tailscale works: `ssh jetson-tailscale` (from Windows)

---

## What Gets Removed

✅ **Packages:**
- wireguard
- wireguard-tools
- wireguard-go
- wireguard-dkms (if installed)

✅ **Files:**
- `/etc/wireguard/wg0.conf`
- `/etc/wireguard/*.key`
- `/home/severin/dev/r2d2/vpn_config/client_wg0.conf`
- `/home/severin/dev/r2d2/vpn_config/*.key`
- `~/.r2d2_vpn_secrets/` (entire directory)

✅ **Services:**
- `wg-quick@wg0.service`
- Systemd overrides

---

## What Stays (Documentation Only)

✅ **Documentation files (safe to keep):**
- `vpn_config/setup_wireguard.sh` (setup script - no keys)
- `vpn_config/*.md` (all documentation files)
- `012_VPN_SETUP_AND_REMOTE_ACCESS.md` (for reference)
- `012_VPN_SETUP_TAILSCALE.md` (current solution)

---

## After Removal

**Expected result:**
- ✅ Clean system with no WireGuard remnants
- ✅ Tailscale continues to work normally
- ✅ Remote access still functional via Tailscale
- ✅ No security issues

**If something breaks:**
- Check Tailscale status: `tailscale status`
- Restart Tailscale: `sudo systemctl restart tailscaled`
- See troubleshooting in `012_VPN_SETUP_TAILSCALE.md`

---

**Ready to remove?** Run: `sudo bash /home/severin/dev/r2d2/vpn_config/remove_wireguard.sh`

