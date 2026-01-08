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

⚠️ **CRITICAL: Search existing documentation BEFORE writing anything new.**  
Duplicated documentation WILL go out of sync and cause confusion. When in doubt, reference - don't duplicate.

#### Step 1: Documentation Discovery (MANDATORY)

**Before writing ANY documentation, search for existing content:**

```bash
# Search for related documentation by keyword
grep -ri "keyword" ~/dev/r2d2/*.md
grep -ri "service-name" ~/dev/r2d2/*.md

# List all documentation files to review
ls -la ~/dev/r2d2/*.md
```

**If related documentation exists:**
- Add to existing file (preferred) OR
- Add cross-reference: `See [XXX_Feature.md](XXX_Feature.md) for details`
- NEVER duplicate procedures, parameters, or configuration values

**Decision framework:**
- Parameters already documented elsewhere? → Add reference, don't duplicate
- Procedure already exists? → Link to it, don't rewrite
- New feature with some shared components? → Document new parts only, reference existing

---

#### Step 2: Write Documentation (only if needed)

- [ ] **Searched existing docs first** (Step 1 completed)
- [ ] No duplicate content created - using references where content already exists
- [ ] Installation steps documented in relevant `NNN_*.md` file
- [ ] Service management commands added to documentation
- [ ] Auto-start behavior documented (which services auto-start, which are manual)
- [ ] Troubleshooting section updated if needed
- [ ] Update `000_INTERNAL_AGENT_NOTES.md` if new patterns emerged
- [ ] **Date Metadata:** If document contains 'Last Updated', 'update-Date', or 'create date', ensure it is updated to the current date

**What to include in documentation:**
- Configuration parameters explained (what they do, valid ranges, defaults)
- Example usage provided
- Common errors and solutions documented
- Dependencies and requirements listed
- Testing procedures documented

**Final deduplication verification:**
- [ ] Parameters documented? Checked if authoritative source exists first (see table in `000_INTERNAL_AGENT_NOTES.md`)
- [ ] If referencing config values: Added "Source of Truth" reference with file path
- [ ] If procedure already documented elsewhere: Used cross-reference, didn't duplicate content
- [ ] Verified parameter values match actual config files (no drift between docs and code)

---

### Phase 5: Git & Deployment

**Purpose:** Commit and deploy your changes to the repository

#### Security Pre-Commit Check (MANDATORY)

⚠️ **Before ANY git commit, run security checks from Core Rules.**

**For detailed security guidelines:** See [`000_AGENT_CORE_RULES.md`](000_AGENT_CORE_RULES.md) (Security section)

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

### Commit Message Format

```
<type>: <Short summary (50 chars)>

<Body: What changed and why>
```

**Types:** `feat:`, `fix:`, `docs:`, `refactor:`, `perf:`, `test:`, `chore:`

### Pre-Push Checklist

```bash
git branch -a           # Verify on 'main'
git status              # Clean working tree?
git push origin main    # Push
```

**Verify:**
1. No uncommitted changes
2. Commit message is descriptive
3. On correct branch
4. No sensitive data in commit

---

## Common Finalization Issues

### Service fails after reboot

**Diagnosis:** `systemctl is-enabled r2d2-your-service.service` returns `disabled`

**Solution:**
```bash
sudo systemctl enable r2d2-your-service.service
sudo systemctl start r2d2-your-service.service
```

### Service starts but fails immediately

**Diagnosis:**
```bash
journalctl -u r2d2-your-service.service -n 50 --no-pager
```

**Common causes:** Missing env vars, wrong directory, missing ROS 2 sourcing, file permissions

### Git push rejected

**Solution:**
```bash
git pull origin main
# Resolve conflicts if any
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

**Last Updated:** January 8, 2026 - Added date metadata verification checklist item

