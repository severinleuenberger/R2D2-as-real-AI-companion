# Secure Storage Guide for VPN Credentials

**⚠️ CRITICAL: Never commit sensitive VPN information to git!**

This guide explains how to securely store your VPN keys and configuration.

---

## What Information is Sensitive?

The following information should **NEVER** be committed to git:

- ✅ **Private Keys** (`.key` files, `PrivateKey` in config files)
- ✅ **Client Configuration** (`client_wg0.conf` - contains private key)
- ✅ **Public Keys** (can be shared, but better to keep private)
- ✅ **Server Configuration** (contains private key)

**Safe to commit:**
- ✅ Setup scripts (no keys)
- ✅ Documentation (no keys)
- ✅ Helper scripts (no keys)

---

## Where Sensitive Files Are Stored

After running `setup_wireguard.sh`, sensitive information is saved to:

```
~/.r2d2_vpn_secrets/
├── vpn_keys.txt          # All keys and configuration (STORE IN KEEPASS)
└── client_wg0.conf.backup # Client config backup
```

**This directory is:**
- ✅ Outside the git repository
- ✅ Protected with `chmod 700` (only you can access)
- ✅ Files protected with `chmod 600` (only you can read)

---

## Storing in KeePass

### Step 1: Open Your KeePass Database

1. Open KeePass on your Windows laptop
2. Unlock your database

### Step 2: Create a New Entry

1. Right-click in KeePass → **Add Entry**
2. **Title:** `R2D2 VPN - Jetson Orin`
3. **Username:** `severin@jetson`
4. **Password:** (leave empty or generate a password for the entry itself)

### Step 3: Add Sensitive Information

Open the file on Jetson:
```bash
cat ~/.r2d2_vpn_secrets/vpn_keys.txt
```

Copy the following fields to KeePass:

#### In KeePass "Notes" or "Additional" Tab:

**Server Keys:**
- Server Private Key: `[paste from vpn_keys.txt]`
- Server Public Key: `[paste from vpn_keys.txt]`

**Client Keys:**
- Client Private Key: `[paste from vpn_keys.txt]`
- Client Public Key: `[paste from vpn_keys.txt]`

**Network Configuration:**
- VPN Server IP: `10.8.0.1`
- VPN Client IP: `10.8.0.2`
- VPN Port: `51820`
- Jetson Local IP: `[your local IP, e.g., 192.168.1.129]`
- Public IP: `[your public IP, e.g., 188.61.209.189]`

#### Or Use KeePass Custom Fields:

1. In KeePass entry, go to **Advanced** tab
2. Click **Add** for each field:
   - `ServerPrivateKey` = `[paste]`
   - `ServerPublicKey` = `[paste]`
   - `ClientPrivateKey` = `[paste]`
   - `ClientPublicKey` = `[paste]`
   - `VPNServerIP` = `10.8.0.1`
   - `VPNClientIP` = `10.8.0.2`
   - `VPNPort` = `51820`
   - `JetsonLocalIP` = `[your local IP]`
   - `PublicIP` = `[your public IP]`

### Step 4: Save KeePass Entry

1. Click **OK** to save the entry
2. Save your KeePass database

---

## Client Configuration File

The `client_wg0.conf` file contains your private key and should be:

1. **Transferred securely** to your Windows laptop (via USB, encrypted transfer, or copy-paste)
2. **Imported into WireGuard** on Windows
3. **Deleted from laptop** after successful import (WireGuard stores it internally)
4. **Stored in KeePass** as backup (copy the file contents to KeePass Notes)

### To Store client_wg0.conf in KeePass:

1. On Jetson, view the file:
   ```bash
   cat /home/severin/dev/r2d2/vpn_config/client_wg0.conf
   ```

2. Copy the entire contents

3. In KeePass entry, paste into **Notes** field with label:
   ```
   === CLIENT CONFIG (client_wg0.conf) ===
   [paste full contents here]
   ```

---

## Verification Checklist

After storing in KeePass, verify:

- [ ] Server Private Key stored
- [ ] Server Public Key stored
- [ ] Client Private Key stored
- [ ] Client Public Key stored
- [ ] Network configuration (IPs, ports) stored
- [ ] Public IP address stored
- [ ] Client config file contents stored
- [ ] KeePass database saved
- [ ] Original files on Jetson are secure (`chmod 600`)

---

## Recovery Procedure

If you need to recover VPN configuration:

1. **Open KeePass** on your laptop
2. **Find entry:** "R2D2 VPN - Jetson Orin"
3. **Copy keys** from KeePass
4. **Recreate files** on Jetson if needed:
   ```bash
   # Server config is in /etc/wireguard/wg0.conf (already exists)
   # Client config can be recreated from KeePass
   ```

---

## Security Best Practices

1. ✅ **Never commit** `.key` files or `client_wg0.conf` to git
2. ✅ **Store all keys** in KeePass (encrypted password manager)
3. ✅ **Delete client config** from laptop after importing to WireGuard
4. ✅ **Use secure transfer** when moving client config to laptop
5. ✅ **Backup KeePass database** regularly
6. ✅ **Keep KeePass database** encrypted and password-protected
7. ✅ **Don't share keys** via email, chat, or unencrypted channels

---

## File Locations Summary

| File | Location | Git Status | Security |
|------|----------|------------|----------|
| **Server Config** | `/etc/wireguard/wg0.conf` | ❌ Not in repo | ✅ Secure (root only) |
| **Server Keys** | `/etc/wireguard/server_*.key` | ❌ Not in repo | ✅ Secure (root only) |
| **Client Keys** | `/etc/wireguard/client_*.key` | ❌ Not in repo | ✅ Secure (root only) |
| **Client Config** | `/home/severin/dev/r2d2/vpn_config/client_wg0.conf` | ❌ Gitignored | ⚠️ Contains private key |
| **Keys Backup** | `~/.r2d2_vpn_secrets/vpn_keys.txt` | ❌ Not in repo | ✅ Secure (chmod 600) |
| **Setup Scripts** | `vpn_config/*.sh` | ✅ Safe to commit | ✅ No keys |
| **Documentation** | `012_VPN_SETUP_AND_REMOTE_ACCESS.md` | ✅ Safe to commit | ✅ No keys |

---

**Created:** December 10, 2025  
**Related:** `012_VPN_SETUP_AND_REMOTE_ACCESS.md`

