# Agent Finalization Guide for R2D2 Project

**Purpose:** This guide is used AFTER your feature has been built, tested, and is working correctly. This covers final verification, documentation, and deployment steps.

**When to Use:** Manually trigger this guide when you've completed development and testing, and are ready to finalize the feature for production deployment.

**Audience:** AI agents performing final deployment steps

---

## Workflow Stage

```
✅ Development Complete
✅ Testing Complete  
✅ Feature Works Manually
→ NOW: Finalize & Deploy (this guide)
```

---

## ⚠️ CRITICAL RULES (READ FIRST!)

### Rule 1: Branch Policy (DEFAULT + EXCEPTIONS)

Default rule:
- User normally works on `main` branch
- `master` branch is **deleted** and must not be used

Exception: Feature-branch workflow (ONLY when explicitly instructed)
- A `golden-*` branch may be used as a read-only stable baseline
- All implementation work MUST happen on a `feat/*` branch created from the golden branch
- The golden branch MUST NOT be modified directly
- Agents may be instructed to NOT commit and NOT push

Rule precedence:
- Task-specific agent instructions OVERRIDE this default rule


### Rule 2: ALWAYS VERIFY BEFORE PUSHING

NOTE:
- This rule applies ONLY when pushing is explicitly allowed
- In feature-branch or agent build workflows, pushing may be disabled


```bash
git branch        # Must show: * main
git log -n 1      # See the commit you're about to push
git status        # Must show: "nothing to commit, working tree clean"
git push origin main
```

### Rule 3: NEVER USE `master:main` SYNTAX
❌ WRONG: `git push origin master:main`  
✅ RIGHT: `git push origin main`

---

## 🚀 FINALIZATION CHECKLIST

### Phase 3: Verification

**Purpose:** Ensure feature survives reboots and runs reliably

- [ ] Service survives restart:
  ```bash
  sudo systemctl restart r2d2-your-service.service
  systemctl status r2d2-your-service.service  # Still active (running)?
  ```
- [ ] **FOR CRITICAL SERVICES: Test with reboot**
  ```bash
  sudo reboot
  ```
- [ ] After reboot, verify service auto-started:
  ```bash
  systemctl status r2d2-your-service.service  # Should be active (running)
  journalctl -u r2d2-your-service.service -n 50  # Check startup logs
  ```
- [ ] Functional testing confirms feature works after reboot
- [ ] Monitor system for 2-5 minutes to ensure stability

**Quick verification commands:**
```bash
# Check all R2D2 services status
systemctl list-units "r2d2-*" --type=service

# Check if service is enabled for auto-start
systemctl is-enabled r2d2-your-service.service  # Must return: enabled

# Watch logs in real-time
journalctl -u r2d2-your-service.service -f
```

---

### Phase 4: Documentation

**Purpose:** Document the feature so others (and future you) can understand and use it

- [ ] Installation steps documented in relevant `NNN_*.md` file
- [ ] Service management commands added to documentation
- [ ] Auto-start behavior documented (which services auto-start, which are manual)
- [ ] Troubleshooting section updated if needed
- [ ] Update `000_INTERNAL_AGENT_NOTES.md` if new patterns emerged

**Documentation checklist:**
- Configuration parameters explained (what they do, valid ranges, defaults)
- Example usage provided
- Common errors and solutions documented
- Dependencies and requirements listed
- Testing procedures documented

---

### Phase 5: Git & Deployment

**Purpose:** Commit and deploy your changes to the repository

#### Security Pre-Commit Check (MANDATORY)

⚠️ **Before ANY git commit, verify no sensitive data is being committed:**

```bash
# Check for real Tailscale IPs (100.x.x.x range)
grep -rE "100\.[0-9]+\.[0-9]+\.[0-9]+" ~/dev/r2d2/*.md

# Check for local network IPs (192.168.x.x range)
grep -rE "192\.168\.[0-9]+\.[0-9]+" ~/dev/r2d2/*.md

# Check for email addresses
grep -rE "[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}" ~/dev/r2d2/*.md
```

**If real IPs or emails are found, replace with placeholders:**
- `100.95.x.x` → `100.x.x.x`
- `192.168.55.1` → `192.168.x.1`
- `user@domain.com` → `user@example.com`

**Security-sensitive items (NEVER commit):**
- ❌ Real IP addresses (use placeholders like `100.x.x.x`)
- ❌ API keys or tokens (use environment variables)
- ❌ SSH key fingerprints
- ❌ Real email addresses
- ❌ Credentials or passwords

**Safe to commit:**
- ✅ Placeholder IPs and generic examples
- ✅ Scripts using environment variables
- ✅ Architecture documentation with sanitized examples

**For detailed security guidelines, see:** `000_INTERNAL_AGENT_NOTES.md` (Security section)

#### Git Commit Checklist

- [ ] **Security check passed** (no real IPs, emails, or credentials)
- [ ] All changes committed with descriptive message
  ```bash
  git add .
  git commit -m "feat: descriptive message about feature"
  ```
- [ ] Branch verified (must be `main` unless explicitly told otherwise)
  ```bash
  git branch  # Must show: * main
  ```
- [ ] Working tree is clean
  ```bash
  git status  # Should show: nothing to commit, working tree clean
  ```
- [ ] Pushed to GitHub
  ```bash
  git push origin main
  ```
- [ ] Verified on GitHub web interface (commit appears in history)

---

## Git Best Practices

### Commit Message Pattern
```
<Type>: <Short summary (50 chars)>

<Body: What changed, why, and measured results>

Example:
---
feat: Add audio volume parameter to audio notification node

- Implement global audio_volume parameter (0.0-1.0)
- Current default: 0.05 (5% - very quiet)
- Tested: Audio plays at 50% volume as expected
- Updated: Both source code and systemd service
- Measured: Service restart successful, no errors
```

**Commit types:**
- `feat:` - New feature
- `fix:` - Bug fix
- `docs:` - Documentation only
- `refactor:` - Code restructuring (no behavior change)
- `perf:` - Performance improvement
- `test:` - Adding tests
- `chore:` - Build process, dependencies, config

### Before Every Push
```bash
git branch -a           # Verify on 'main'
git log --oneline -3    # Check last 3 commits
git status              # Clean working tree?
git push origin main    # Push
```

**Pre-push verification:**
1. No uncommitted changes (`git status` clean)
2. All files added (`git add` complete)
3. Commit message is descriptive
4. On correct branch (`main` or specified feature branch)
5. No sensitive data in commit (API keys, passwords)

---

## Example Workflow: Full Feature Finalization

```bash
# ========================================
# Phase 3: Verification
# ========================================

# Test service restart
sudo systemctl restart r2d2-your-service.service
systemctl status r2d2-your-service.service

# Test reboot (critical services only)
sudo reboot

# After reboot - verify auto-start
systemctl status r2d2-your-service.service
journalctl -u r2d2-your-service.service -n 50

# Functional test (example: check ROS topic)
ros2 topic echo /your/topic -n 5

# Monitor stability
watch -n 2 'systemctl status r2d2-your-service.service'
# (Ctrl+C after 2-5 minutes if stable)

# ========================================
# Phase 4: Documentation
# ========================================

# Update relevant documentation file
vim ~/dev/r2d2/2XX_YOUR_FEATURE.md

# Document service in internal notes (if new patterns)
vim ~/dev/r2d2/000_INTERNAL_AGENT_NOTES.md

# ========================================
# Phase 5: Git & Deployment
# ========================================

# Verify branch
git branch  # Must show: * main

# Add all changes
git add .

# Commit with descriptive message
git commit -m "feat: add your-service with auto-start

- Implemented feature X with parameters Y and Z
- Service auto-starts on boot via systemd
- Tested: survives reboot, runs for 5+ minutes stable
- Documentation: Updated 2XX_YOUR_FEATURE.md with setup and usage
- Verified: No resource conflicts, CPU usage within limits"

# Verify commit
git log -n 1

# Check working tree is clean
git status

# Push to GitHub
git push origin main

# Verify on GitHub
# (Open GitHub web interface and check commit appears)
```

---

## Common Finalization Issues

### Issue: Service fails after reboot

**Symptoms:**
- Service works when started manually
- After reboot, service is `inactive (dead)`

**Diagnosis:**
```bash
systemctl is-enabled r2d2-your-service.service  # Returns: disabled
```

**Solution:**
```bash
sudo systemctl enable r2d2-your-service.service
sudo systemctl start r2d2-your-service.service
```

---

### Issue: Service starts but fails immediately

**Symptoms:**
- `systemctl status` shows `failed` or `exited`

**Diagnosis:**
```bash
journalctl -u r2d2-your-service.service -n 50 --no-pager
```

**Common causes:**
- Missing environment variables (check `Environment=` in service file)
- Wrong working directory (check `WorkingDirectory=` in service file)
- Missing ROS 2 sourcing (add `source /opt/ros/humble/setup.bash` to ExecStartPre)
- File permissions (ensure script is executable: `chmod +x`)

---

### Issue: Git push rejected

**Symptoms:**
```
! [rejected]        main -> main (fetch first)
error: failed to push some refs to 'origin'
```

**Solution:**
```bash
# Pull latest changes first
git pull origin main

# Resolve any conflicts if present
# Then push again
git push origin main
```

---

### Issue: Uncommitted changes blocking push

**Symptoms:**
```
git status shows modified files
```

**Solution:**
```bash
# Add all changes
git add .

# Or selectively add files
git add file1.py file2.md

# Commit
git commit -m "feat: descriptive message"

# Push
git push origin main
```

---

## Why This Guide Exists

**Real incident (Dec 24, 2025):**
- Services were developed and tested manually ✅
- All functionality worked perfectly ✅
- System rebooted 🔄
- **Everything stopped working** ❌
- Root cause: Services were never enabled for auto-start
- Solution: Had to manually install and enable all services

**This guide ensures proper finalization and prevents production breakage.**

---

## Post-Deployment Monitoring

After deployment, monitor for the first 24 hours:

```bash
# Check service status periodically
systemctl status r2d2-your-service.service

# Watch logs for errors
journalctl -u r2d2-your-service.service -f

# Monitor system resources
~/dev/r2d2/tools/minimal_monitor.py

# Check for crashes/restarts
systemctl list-units --failed
```

**Success criteria:**
- Service remains `active (running)` for 24+ hours
- No unexpected restarts (`systemctl status` shows uptime)
- No errors in logs (`journalctl` clean)
- System resource usage within expected bounds
- Feature functionality verified working

---

**This document is for final deployment steps only. For development and testing, refer to `000_INTERNAL_AGENT_NOTES.md`.**

**Last Updated:** December 29, 2025 - Added Security Pre-Commit Check (MANDATORY)

