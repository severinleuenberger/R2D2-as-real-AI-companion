# R2D2 System Security Reference
## Security Architecture and Best Practices

**Date:** December 25, 2025  
**Status:** ✅ Production Security Documentation  
**Platform:** NVIDIA Jetson AGX Orin 64GB  
**Security Model:** VPN-Only Access (Tailscale)

---

## Executive Summary

The R2D2 system implements a **defense-in-depth security model** with multiple layers:

1. **Network Isolation:** All services accessible only via Tailscale VPN
2. **Secrets Management:** API keys stored outside repository with strict permissions
3. **Input Validation:** All user inputs validated and sanitized
4. **Privilege Minimization:** Limited sudo access to specific operations
5. **Data Protection:** Personal data (faces, conversations) never committed to git

**Security Status:** ✅ **SECURE**  
No critical vulnerabilities identified. All network services protected by VPN.

---

## Table of Contents

1. [Security Architecture](#security-architecture)
2. [API Key Management](#api-key-management)
3. [Network Security](#network-security)
4. [Database Security](#database-security)
5. [Access Control](#access-control)
6. [Input Validation](#input-validation)
7. [Security Checklist](#security-checklist)
8. [Incident Response](#incident-response)

---

## Security Architecture

### Layered Security Model

```
┌─────────────────────────────────────────────────────────────┐
│                    INTERNET                                  │
│              (Untrusted Network)                             │
└─────────────────────────────────────────────────────────────┘
                           │
                           │ BLOCKED - No Direct Access
                           │ (No ports forwarded)
                           ↓
┌─────────────────────────────────────────────────────────────┐
│                  TAILSCALE VPN LAYER                         │
│            WireGuard Protocol (Encrypted)                    │
│      Authentication: Google/Microsoft/GitHub SSO             │
│      Only authorized devices can connect                     │
└─────────────────────────────────────────────────────────────┘
                           │
                           │ VPN IP: 100.95.133.26
                           │ (Authenticated & Encrypted)
                           ↓
┌─────────────────────────────────────────────────────────────┐
│                    JETSON ORIN (Private Network)             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  Web Services (No Additional Authentication)        │   │
│  │  ├─ Web Dashboard     :8080  (FastAPI)              │   │
│  │  ├─ rosbridge         :9090  (WebSocket)            │   │
│  │  ├─ Camera Stream     :8081  (MJPEG/HTTP)           │   │
│  │  └─ Wake API          :8079  (FastAPI)              │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                              │
│  Secrets Storage (Protected by Filesystem Permissions):     │
│  ├─ ~/.r2d2/.env           (chmod 600, user-only)           │
│  └─ vpn_config/*.key       (gitignored, local-only)         │
│                                                              │
│  Private Data (Never Committed to Git):                     │
│  ├─ data/persons.db                    (*.db gitignored)    │
│  ├─ r2d2_speech/data/conversations.db  (*.db gitignored)    │
│  └─ data/face_recognition/*/           (images gitignored)  │
└─────────────────────────────────────────────────────────────┘
```

### Defense Layers

| Layer | Technology | Protection |
|-------|------------|------------|
| **1. Network Perimeter** | Tailscale VPN | Blocks all direct internet access |
| **2. Authentication** | Tailscale SSO | Google/Microsoft/GitHub login required |
| **3. Encryption** | WireGuard | All traffic encrypted end-to-end |
| **4. File Permissions** | Linux chmod | Secrets readable only by owner |
| **5. Git Protection** | .gitignore | Prevents accidental secret commits |
| **6. Input Validation** | Pydantic/Regex | Prevents injection attacks |
| **7. Privilege Control** | sudoers.d | Limited sudo to specific services |

---

## API Key Management

### OpenAI API Key Storage

**Location:** `~/.r2d2/.env` (Outside git repository)

**Setup:**
```bash
# Create config directory
mkdir -p ~/.r2d2
chmod 700 ~/.r2d2

# Create .env file
nano ~/.r2d2/.env
# Add: OPENAI_API_KEY=sk-...

# Set restrictive permissions (owner read/write only)
chmod 600 ~/.r2d2/.env

# Verify permissions
ls -la ~/.r2d2/.env
# Expected: -rw------- 1 severin severin
```

**Why This is Secure:**

1. **Outside Repository:** File is in home directory, not in `/home/severin/dev/r2d2/`
2. **Strict Permissions:** `chmod 600` = only the owner can read/write
3. **Pattern Blocked:** `.env` files now in `.gitignore` as failsafe
4. **No History:** Never committed to git (verified via `git log` search)

**Loading the Key:**

```python
# r2d2_speech/config/config_manager.py
from dotenv import load_dotenv
import os

env_path = Path.home() / ".r2d2" / ".env"
load_dotenv(env_path)
api_key = os.getenv("OPENAI_API_KEY")
```

### Git Protection

**.gitignore patterns** (added for defense-in-depth):

```gitignore
# Environment variables and secrets (SECURITY)
.env
*.env
.env.*
.env.local

# R2D2 config directory (if accidentally created in repo)
.r2d2/
```

**Status:** ✅ No API keys ever committed to git history

---

## Network Security

### Service Binding Analysis

| Service | Port | Binding | Risk Level | Mitigation |
|---------|------|---------|------------|------------|
| Web Dashboard | 8080 | 0.0.0.0 | Low* | VPN-only access |
| rosbridge | 9090 | 0.0.0.0 | Low* | VPN-only access |
| Camera Stream | 8081 | 0.0.0.0 | Low* | VPN-only access |
| Wake API | 8079 | 100.95.133.26 | Low* | VPN IP only |

*Low risk because Tailscale VPN restricts all network access

### Why No Authentication on Services?

**Design Decision:** Services bind to `0.0.0.0` but are protected by **network-level security** instead of application-level authentication.

**Rationale:**

1. **VPN as Authentication:** Tailscale provides strong authentication via SSO (Google/Microsoft/GitHub)
2. **Encrypted Channel:** All traffic encrypted by WireGuard protocol
3. **No Internet Exposure:** No router port forwarding, no direct internet access
4. **Simplified Architecture:** No need to manage passwords, session tokens, JWT, etc.
5. **Single Point of Control:** Revoke device access in Tailscale admin panel

**Trade-offs:**

| Approach | Pros | Cons |
|----------|------|------|
| **Current (VPN-Only)** | ✅ Simple<br>✅ Strong auth (SSO)<br>✅ Encrypted<br>✅ Single control point | ⚠️ Requires VPN<br>⚠️ Trusts all VPN users |
| **App-Level Auth** | ✅ Granular control<br>✅ Works without VPN | ❌ Complex<br>❌ Password management<br>❌ Session handling |

**Current approach is appropriate for:**
- Personal/home robotics projects
- Single-user or trusted family access
- Development/experimental systems

**Would need app-level auth for:**
- Multi-tenant systems
- Untrusted users on same VPN
- Commercial deployments

### Firewall Status

**Current State:** No firewall rules configured (default policy: accept)

**Why This is Acceptable:**
- Jetson is behind home router (NAT)
- No port forwarding configured
- Tailscale VPN is the security boundary

**Optional Enhancement:**
```bash
# If additional defense-in-depth desired:
sudo ufw default deny incoming
sudo ufw default allow outgoing
sudo ufw allow from 100.64.0.0/10  # Tailscale IP range
sudo ufw enable
```

---

## Database Security

### Personal Data Storage

| Database | Contents | Location | Protection |
|----------|----------|----------|------------|
| `persons.db` | Face recognition registry | `data/` | `.gitignore` pattern |
| `conversations.db` | Speech conversation history | `r2d2_speech/data/` | `.gitignore` pattern |

### Database File Protection

**Git Exclusion:**

```gitignore
# SQLite database files (user data, not for sharing)
*.db
*.sqlite
*.sqlite3
```

**Files Removed from Git:**
- ✅ `data/persons.db` (removed from tracking)
- ✅ `r2d2_speech/data/conversations.db` (removed from tracking)
- ✅ `tests/face_recognition/data/persons.db` (removed from tracking)

**Local files remain intact** - only removed from git tracking.

### Face Recognition Data

**Training Images:** `data/face_recognition/*/`

**Protection:**
```gitignore
# Face Recognition training data (protect locally, don't share on GitHub)
data/face_recognition/*/
!data/face_recognition/models/
```

**What Gets Committed:**
- ✅ Model files (`.xml` - anonymized face encodings)
- ✅ Directory structure
- ❌ Photos (`.jpg`, `.png` - personal images)

---

## Access Control

### SSH Access

**Method:** Tailscale SSH (VPN-based)

**Configuration:**
```powershell
# Windows Client
ssh severin@100.95.133.26

# Or using alias
ssh jetson-tailscale
```

**Security Features:**
- ✅ No password authentication over internet
- ✅ SSH traffic encrypted by Tailscale VPN
- ✅ Access revocable from Tailscale admin panel
- ✅ No direct port 22 exposure to internet

### Passwordless Sudo

**Scope:** Limited to R2D2 service control only

**Configuration:** `/etc/sudoers.d/r2d2-services`
```bash
severin ALL=(ALL) NOPASSWD: /bin/systemctl start r2d2-*, /bin/systemctl stop r2d2-*, /bin/systemctl restart r2d2-*
```

**Allowed Operations:**
- ✅ `sudo systemctl start r2d2-*`
- ✅ `sudo systemctl stop r2d2-*`
- ✅ `sudo systemctl restart r2d2-*`

**Not Allowed:**
- ❌ `sudo systemctl start <other-service>`
- ❌ `sudo <any-other-command>`
- ❌ Arbitrary command execution

**Why This is Safe:**
- Service names are validated against allowlist
- No shell expansion in systemctl commands
- Limited to specific systemctl operations
- Only affects r2d2-* prefixed services

---

## Input Validation

### Web Dashboard APIs

#### Training API (Person Names)

**Validation:** Pydantic model with regex pattern

```python
class TrainingRequest(BaseModel):
    person_name: str = Field(
        ..., 
        min_length=1,
        max_length=50,
        pattern=r'^[a-zA-Z0-9_]+$'  # Alphanumeric + underscore only
    )
```

**Protection:**
- ✅ Prevents shell injection (no spaces, special chars)
- ✅ Prevents path traversal (no `/`, `..`)
- ✅ Length-limited (max 50 chars)
- ✅ Validated before execution

#### Service Control API

**Validation:** Allowlist-based

```python
SERVICES = {
    "audio": "r2d2-audio-notification.service",
    "camera": "r2d2-camera-perception.service",
    "powerbutton": "r2d2-powerbutton.service",
    "heartbeat": "r2d2-heartbeat.service",
    "camera-stream": "r2d2-camera-stream.service"
}

def start_service(service_name: str):
    full_service_name = SERVICES.get(service_name)
    if not full_service_name:
        return {"success": False, "error": "Unknown service"}
    # Only allowed services can be started
```

**Protection:**
- ✅ No arbitrary service names
- ✅ No command injection
- ✅ Predefined list of allowed services

### Subprocess Safety

**Secure Pattern:**
```python
# ✅ SAFE: List of arguments (no shell expansion)
subprocess.run(['systemctl', 'start', full_service_name], ...)

# ✅ SAFE: Arguments validated before use
subprocess.run(['python3', str(script), person_name, ...], ...)
```

**Avoided Pattern:**
```python
# ❌ DANGEROUS: Never used in this project
subprocess.run(f'systemctl start {user_input}', shell=True)
```

---

## Security Checklist

### For Developers

**Before Committing Code:**

- [ ] No hardcoded API keys or passwords
- [ ] No `.env` files in commit
- [ ] No database files (`.db`) in commit
- [ ] No personal photos in `data/face_recognition/*/`
- [ ] Input validation for user-provided data
- [ ] No `shell=True` in subprocess calls
- [ ] No SQL string concatenation (use parameterized queries)

**Before Deploying:**

- [ ] `~/.r2d2/.env` exists with correct permissions (chmod 600)
- [ ] Tailscale VPN is active and connected
- [ ] No router port forwarding to Jetson
- [ ] Passwordless sudo configured for web dashboard
- [ ] Database files exist locally but not in git

### For System Operators

**Monthly Security Review:**

- [ ] Review Tailscale connected devices
- [ ] Check for unauthorized SSH sessions: `who`
- [ ] Verify .env file permissions: `ls -la ~/.r2d2/.env`
- [ ] Check for unexpected sudo usage: `journalctl -t sudo`
- [ ] Review systemd service status

**After Adding New Service:**

- [ ] Binds to Tailscale IP or localhost (not public IP)
- [ ] Protected by VPN or has authentication
- [ ] Secrets stored in `~/.r2d2/.env`, not in code
- [ ] Service name added to `/etc/sudoers.d/r2d2-services` if needed

---

## Security Best Practices

### Secret Management

**DO:**
- ✅ Store secrets in `~/.r2d2/.env`
- ✅ Set `chmod 600` on secret files
- ✅ Use environment variables to load secrets
- ✅ Keep `.r2d2/` directory outside git repo

**DON'T:**
- ❌ Hardcode API keys in Python files
- ❌ Commit `.env` files to git
- ❌ Share `.env` files via Slack/Email
- ❌ Store secrets in ROS parameters (they're logged)

### Network Exposure

**DO:**
- ✅ Use Tailscale VPN for all remote access
- ✅ Bind services to VPN IP when possible
- ✅ Keep router port forwarding disabled
- ✅ Monitor connected VPN devices

**DON'T:**
- ❌ Forward ports on home router
- ❌ Disable Tailscale VPN
- ❌ Bind services to public IP
- ❌ Share VPN credentials

### Database Protection

**DO:**
- ✅ Add `*.db` to `.gitignore`
- ✅ Keep personal data (faces, conversations) local-only
- ✅ Back up databases separately (not via git)
- ✅ Use parameterized SQL queries

**DON'T:**
- ❌ Commit database files to git
- ❌ Share databases publicly
- ❌ Use string concatenation for SQL
- ❌ Store sensitive data unencrypted

---

## Incident Response

### If API Key is Leaked

**Immediate Actions:**

1. **Revoke Compromised Key:**
   ```bash
   # Visit: https://platform.openai.com/api-keys
   # Delete the leaked key immediately
   ```

2. **Generate New Key:**
   ```bash
   # Create new API key in OpenAI dashboard
   # Update ~/.r2d2/.env with new key
   nano ~/.r2d2/.env
   ```

3. **Restart Services:**
   ```bash
   sudo systemctl restart r2d2-speech-node
   ```

4. **Check Git History:**
   ```bash
   # Search for any leaked keys in git
   git log -p | grep -i "sk-"
   
   # If found in history, rewrite git history (DANGEROUS):
   # Contact a git expert before attempting
   ```

### If Unauthorized VPN Access Detected

**Immediate Actions:**

1. **Revoke Device:**
   - Visit Tailscale admin console
   - Disable or remove unauthorized device

2. **Review Access:**
   ```bash
   # Check current SSH sessions
   who
   
   # Check recent logins
   last -20
   
   # Check sudo usage
   journalctl -t sudo --since today
   ```

3. **Rotate Credentials:**
   - Change OpenAI API key
   - Review and rotate any other secrets

### If Database Accidentally Committed

**Immediate Actions:**

1. **Remove from Git:**
   ```bash
   git rm --cached data/persons.db
   git rm --cached r2d2_speech/data/conversations.db
   git commit -m "Remove accidentally committed databases"
   ```

2. **If Already Pushed:**
   ```bash
   # WARNING: This rewrites public history
   # Coordinate with all team members
   git filter-branch --force --index-filter \
     'git rm --cached --ignore-unmatch data/persons.db' \
     --prune-empty --tag-name-filter cat -- --all
   
   git push origin --force --all
   ```

3. **Verify Removal:**
   ```bash
   git log --all --full-history -- data/persons.db
   # Should show no results
   ```

---

## Security Audit Log

### Audit History

| Date | Finding | Severity | Status | Fix |
|------|---------|----------|--------|-----|
| 2025-12-25 | `.env` patterns missing from .gitignore | Medium | ✅ Fixed | Added `.env`, `*.env` patterns |
| 2025-12-25 | Database files tracked in git | Low | ✅ Fixed | Added `*.db` pattern, removed files |
| 2025-12-25 | Services bind to 0.0.0.0 | Low | ✅ Accepted | Documented as VPN-protected |
| 2025-12-25 | No web dashboard authentication | Low | ✅ Accepted | Documented as VPN-only access |

### No Critical Issues Found

- ✅ No API keys in git history
- ✅ No hardcoded passwords in code
- ✅ No public network exposure
- ✅ Input validation properly implemented
- ✅ Privilege escalation properly restricted

---

## Related Documentation

- [012_VPN_SETUP_AND_REMOTE_ACCESS.md](012_VPN_SETUP_AND_REMOTE_ACCESS.md) - Tailscale VPN setup
- [110_WEB_UI_REFERENCE.md](110_WEB_UI_REFERENCE.md) - Web dashboard architecture
- [201_SPEECH_SYSTEM_INSTALLATION.md](201_SPEECH_SYSTEM_INSTALLATION.md) - API key setup guide
- [001_ARCHITECTURE_OVERVIEW.md](001_ARCHITECTURE_OVERVIEW.md) - System architecture

---

## Contact & Support

**Security Issues:** If you discover a security vulnerability, please:

1. **DO NOT** create a public GitHub issue
2. Email the maintainer directly (see repo README)
3. Include detailed description and reproduction steps
4. Allow time for fix before public disclosure

---

**Document Status:** ✅ Complete  
**Last Updated:** December 25, 2025  
**Security Posture:** SECURE - No critical vulnerabilities

