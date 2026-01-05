# Diagnostics Documentation Links Feature - Safe Git Merge Plan

**Created:** January 5, 2026  
**Status:** Ready for Execution  
**Purpose:** Safe merge plan for diagnostics documentation links feature with existing developments

---

## Feature Overview

### What Was Implemented

**Diagnostics Page Enhancement - Documentation Links Integration**

Added direct GitHub documentation links to every button on the diagnostics page (`/diagnostics`) to improve discoverability and user education.

**Changes Made:**

1. **HTML Changes** (`web_dashboard/app/templates/diagnostics.html`)
   - Added 📖 Docs links under each Topic Monitor button (16 total)
   - Added 📖 Docs links under each Diagnostic Test button (10 total)
   - Links organized by category: Perception, Audio, Speech, System

2. **CSS Changes** (`web_dashboard/app/static/css/diagnostics.css`)
   - Added `.doc-link` class styling
   - Small, unobtrusive font (0.7rem)
   - Subdued color (#888) with hover effect (#00a8ff)
   - External link indicator (↗) after each link
   - Responsive layout for test button containers

3. **Documentation Update** (`110_WEB_UI_REFERENCE.md`)
   - Updated Diagnostics Page section
   - Added documentation about the new link feature
   - Described user benefits and link targets

**Documentation Link Mapping:**

| Button Category | Documentation Target |
|----------------|---------------------|
| **Perception Topics** | 100_PERCEPTION_STATUS_REFERENCE.md |
| **Audio - person_status, notification_event** | 100_PERCEPTION_STATUS_REFERENCE.md |
| **Audio - master_volume** | 260_VOLUME_CONTROL_REFERENCE.md |
| **Speech Topics (all)** | 200_SPEECH_SYSTEM_REFERENCE.md |
| **System - heartbeat** | 006_SYSTEM_STATUS_AND_MONITORING.md |
| **Test - PulseAudio, Bluetooth** | 261_BLUETOOTH_AUDIO_REFERENCE.md |
| **Test - Audio Playback, Volume** | 260_VOLUME_CONTROL_REFERENCE.md |
| **Test - Speech Status** | 200_SPEECH_SYSTEM_REFERENCE.md |
| **Test - Quick Status** | 005_SYSTEMD_SERVICES_REFERENCE.md |
| **Test - Topic Hz, ROS Nodes** | 001_ARCHITECTURE_OVERVIEW.md |
| **Test - Recognition, Gesture Log** | 100_PERCEPTION_STATUS_REFERENCE.md |

**GitHub Base URL:** `https://github.com/severinleuenberger/R2D2-as-real-AI-companion/blob/main/dev/r2d2/`

**User Benefits:**
- Quick access to detailed technical documentation
- Better understanding of what each topic/test does
- Improved learning and troubleshooting experience
- Links open in new tab, preserving diagnostics page state

---

## Current Git Situation

### Branch State

**Active Branches:**
- `main` - Production branch (contains red status calculation logic)
- `feature/face-tracking-tilt-servo` - Feature branch (contains diagnostics documentation links)

**Current Problem:**
- Currently stuck in middle of failed cherry-pick operation on `main` branch
- Attempted to cherry-pick commit from feature branch to main
- Conflict occurred because `diagnostics.html` and `diagnostics.css` don't exist on main branch
- Files were added in feature branch but never merged to main

**Stashed Changes:**
- `110_WEB_UI_REFERENCE.md` documentation update is stashed on feature branch

**Parallel Developments:**
1. **Red status calculation logic** (already on main) - Must be preserved
2. **Diagnostics documentation links** (on feature branch) - Needs to be merged

### Why Conflict Occurred

```
Feature Branch: "Modify these files"
   ├─ diagnostics.html (exists)
   └─ diagnostics.css (exists)

Main Branch: "What files?"
   ├─ diagnostics.html (DOES NOT EXIST)
   └─ diagnostics.css (DOES NOT EXIST)

Cherry-pick Result: CONFLICT!
```

The diagnostics page files were created on the feature branch but the feature branch was never fully merged to main. Git cannot modify files that don't exist.

---

## Safe Merge Plan - Step by Step

### Strategy

Use **Git patch mechanism** with **backup safety net** for the safest possible merge.

**Why Patch Method?**
- ✅ Safest Git merge mechanism
- ✅ Can be reviewed before applying
- ✅ Clear error messages if conflicts occur
- ✅ Easy to abort and retry

### Prerequisites

- Terminal access to Jetson
- Git repository at `~/dev/r2d2`
- No critical processes running that depend on uncommitted changes

---

### STEP 1: Clean Up Current Git State

**Objective:** Abort failed cherry-pick and return to clean state

**Commands:**
```bash
cd ~/dev/r2d2

# Abort the failed cherry-pick
git cherry-pick --abort

# Switch to main branch
git checkout main

# Verify clean state
git status
```

**Expected Output:**
```
On branch main
Your branch is up to date with 'origin/main'.

nothing to commit, working tree clean
```

**If Problems:**
- If `git status` shows uncommitted changes, review them carefully
- If changes are important, stash them: `git stash push -m "description"`
- If changes are not important, discard them: `git restore .`

---

### STEP 2: Create Safety Backup

**Objective:** Create backup branch as recovery point

**Commands:**
```bash
cd ~/dev/r2d2

# Create backup branch with timestamp
git checkout -b backup-main-$(date +%Y%m%d-%H%M)

# Verify backup created
git branch | grep backup

# Return to main
git checkout main
```

**Expected Output:**
```
Switched to a new branch 'backup-main-20260105-2300'
  backup-main-20260105-2300
Switched to branch 'main'
```

**Safety Net:** If anything goes wrong later, you can recover:
```bash
git checkout backup-main-TIMESTAMP
git checkout -b main-recovery
# Investigate and fix
```

---

### STEP 3: Prepare Feature Branch

**Objective:** Complete all commits on feature branch

**Commands:**
```bash
cd ~/dev/r2d2

# Switch to feature branch
git checkout feature/face-tracking-tilt-servo

# Apply stashed documentation change
git stash pop

# Check what changed
git status

# Stage documentation update
git add 110_WEB_UI_REFERENCE.md

# Commit with descriptive message
git commit -m "docs: Update diagnostics page documentation

- Added documentation about GitHub links feature
- Updated Topic Monitors section
- Updated Diagnostic Tests section
- Described user benefits"

# Verify all changes committed
git status
```

**Expected Output:**
```
On branch feature/face-tracking-tilt-servo
nothing to commit, working tree clean
```

**If Stash Not Found:**
- Check if documentation was already committed: `git log --oneline -3`
- If already committed, skip the stash step

---

### STEP 4: Generate Patch File

**Objective:** Create portable patch from feature branch

**Commands:**
```bash
cd ~/dev/r2d2

# Ensure you're on feature branch
git branch

# Generate patch file
git format-patch main --stdout > /tmp/diagnostics-feature.patch

# Verify patch created
ls -lh /tmp/diagnostics-feature.patch

# Optional: Review patch content
head -50 /tmp/diagnostics-feature.patch
```

**Expected Output:**
```
-rw-rw-r-- 1 severin severin 15K Jan  5 23:00 /tmp/diagnostics-feature.patch
```

**Patch File Contains:**
- All commits from feature branch not in main
- File additions, modifications, deletions
- Commit messages and metadata

---

### STEP 5: Apply Patch to Main

**Objective:** Merge changes to main branch using patch

**Commands:**
```bash
cd ~/dev/r2d2

# Switch to main branch
git checkout main

# Apply patch
git am /tmp/diagnostics-feature.patch
```

**Expected Output (Success):**
```
Applying: feat: Add documentation links to diagnostics topic monitor buttons
Applying: docs: Update diagnostics page documentation
```

**If Conflicts Occur:**
```
error: patch failed: some-file.txt:123
error: some-file.txt: patch does not apply
Patch failed at 0001 feat: Add documentation links

When you have resolved this problem, run "git am --continue".
If you prefer to skip this patch, run "git am --skip" instead.
To restore the original branch and stop patching, run "git am --abort".
```

**To Resolve Conflicts:**
1. Check which files have conflicts: `git status`
2. Edit conflicted files manually
3. Stage resolved files: `git add <file>`
4. Continue patch: `git am --continue`

**To Abort and Retry:**
```bash
git am --abort
# Review patch file, fix issues, try again
```

---

### STEP 6: Verify Merge Success

**Objective:** Confirm all changes applied correctly

**Commands:**
```bash
cd ~/dev/r2d2

# Check commit history
git log --oneline -5

# Verify working tree is clean
git status

# Check files exist
ls -la web_dashboard/app/templates/diagnostics.html
ls -la web_dashboard/app/static/css/diagnostics.css
ls -la 110_WEB_UI_REFERENCE.md

# Quick content verification
head -20 web_dashboard/app/templates/diagnostics.html | grep "doc-link"
```

**Expected Output:**
```
3e56b895 docs: Update diagnostics page documentation
2a1b3c4d feat: Add documentation links to diagnostics topic monitor buttons
[... other commits ...]

On branch main
nothing to commit, working tree clean

-rw-rw-r-- 1 severin severin 8.2K web_dashboard/app/templates/diagnostics.html
-rw-rw-r-- 1 severin severin 5.1K web_dashboard/app/static/css/diagnostics.css
-rw-rw-r-- 1 severin severin 48K 110_WEB_UI_REFERENCE.md
```

**Files Should Contain:**
- `diagnostics.html` - Look for `<a href="https://github.com/...` and `class="doc-link"`
- `diagnostics.css` - Look for `.doc-link` class definition
- `110_WEB_UI_REFERENCE.md` - Look for updated Diagnostics section

---

### STEP 7: Security Pre-Push Checks

**Objective:** Ensure no sensitive data is being committed

**Commands:**
```bash
cd ~/dev/r2d2

# Check for real Tailscale IPs (100.x.x.x)
echo "Checking for Tailscale IPs..."
grep -rE "100\.[0-9]+\.[0-9]+\.[0-9]+" web_dashboard/ 110_WEB_UI_REFERENCE.md

# Check for local network IPs (192.168.x.x)
echo "Checking for local IPs..."
grep -rE "192\.168\.[0-9]+\.[0-9]+" web_dashboard/ 110_WEB_UI_REFERENCE.md

# Check for email addresses
echo "Checking for emails..."
grep -rE "[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}" web_dashboard/ 110_WEB_UI_REFERENCE.md

# Check for API keys or tokens
echo "Checking for potential secrets..."
grep -rE "(api[_-]?key|token|secret|password)" -i web_dashboard/ 110_WEB_UI_REFERENCE.md
```

**Expected Output:**
```
Checking for Tailscale IPs...
(no output - good!)

Checking for local IPs...
(no output - good!)

Checking for emails...
(no output - good!)

Checking for potential secrets...
(should only find variable names, not actual secrets)
```

**If Sensitive Data Found:**
- Replace real IPs with placeholders: `100.95.24.68` → `100.x.x.x`
- Replace emails with examples: `user@domain.com` → `user@example.com`
- Remove any API keys or tokens
- Stage and commit fixes before pushing

**Security Checklist:**
- ❌ Real IP addresses (use `100.x.x.x`, `192.168.x.x`)
- ❌ Email addresses (use `user@example.com`)
- ❌ API keys or tokens
- ❌ SSH key fingerprints
- ❌ Passwords or credentials
- ✅ Placeholder IPs and examples are OK
- ✅ Environment variable names are OK
- ✅ Generic documentation examples are OK

---

### STEP 8: Deploy to Production

**Objective:** Push changes to GitHub main branch

**Commands:**
```bash
cd ~/dev/r2d2

# Final verification - must be on main
git branch

# Review what will be pushed
git log origin/main..HEAD --oneline

# Verify clean working tree
git status

# Push to GitHub
git push origin main
```

**Expected Output:**
```
* main

3e56b895 docs: Update diagnostics page documentation
2a1b3c4d feat: Add documentation links to diagnostics topic monitor buttons

On branch main
nothing to commit, working tree clean

Enumerating objects: 15, done.
Counting objects: 100% (15/15), done.
Delta compression using up to 12 threads
Compressing objects: 100% (8/8), done.
Writing objects: 100% (9/9), 2.45 KiB | 2.45 MiB/s, done.
Total 9 (delta 6), reused 0 (delta 0)
To github.com:severinleuenberger/R2D2-as-real-AI-companion.git
   abc1234..3e56b895  main -> main
```

**If Push Rejected:**
```
! [rejected]        main -> main (fetch first)
error: failed to push some refs to 'origin'
```

**Solution:**
```bash
# Pull latest changes
git pull origin main

# Resolve any conflicts if present
# Then push again
git push origin main
```

**Verify on GitHub:**
1. Open https://github.com/severinleuenberger/R2D2-as-real-AI-companion
2. Go to commits page
3. Verify your commits appear in history
4. Check that file changes are visible

---

### STEP 9: Cleanup and Verification

**Objective:** Remove temporary files and verify deployment

**Commands:**
```bash
cd ~/dev/r2d2

# Remove patch file
rm /tmp/diagnostics-feature.patch

# Verify no temporary files remain
ls -la /tmp/diagnostics-*

# Final status check
git status

# List all branches
git branch -a

# Optional: View the diagnostics page files
cat web_dashboard/app/templates/diagnostics.html | grep -A 2 "doc-link" | head -20
```

**Expected Output:**
```
rm: cannot remove '/tmp/diagnostics-feature.patch': No such file or directory

On branch main
Your branch is up to date with 'origin/main'.

nothing to commit, working tree clean

* main
  backup-main-20260105-2300
  feature/face-tracking-tilt-servo
  ...
```

**Optional: Delete Backup Branch**
```bash
# Only after confirming everything works!
# Wait at least 24 hours before deleting backup
git branch -d backup-main-20260105-2300
```

---

## Safety Procedures and Backup Strategy

### Multiple Safety Nets

**1. Backup Branch**
- Created before any merge operations
- Timestamp-based naming for easy identification
- Can be used for instant rollback

**2. Patch-Based Merge**
- Safest Git merge mechanism
- Portable and reviewable
- Clear error messages
- Easy to abort and retry

**3. Step-by-Step Verification**
- Check status after each step
- Verify files exist and contain expected content
- Confirm clean working tree before pushing

**4. Security Checks**
- Scan for sensitive data before push
- Prevent accidental credential leaks
- Follow R2D2 project security guidelines

**5. Clean Working Tree**
- No uncommitted changes during merge
- All modifications tracked and committed
- Clear audit trail

### Rollback Procedures

**If Merge Goes Wrong (Before Push):**
```bash
# Abort current operation
git am --abort  # or git merge --abort

# Return to backup
git checkout backup-main-TIMESTAMP

# Create recovery branch
git checkout -b main-recovery

# Investigate what went wrong
git log --oneline -10
git diff main main-recovery

# When ready, try again or fix manually
```

**If Issues Discovered After Push:**
```bash
# Create new branch from backup
git checkout backup-main-TIMESTAMP
git checkout -b emergency-fix

# Verify this is the correct state
git log --oneline -5

# Force push to main (DANGEROUS - coordinate with team first!)
git push origin emergency-fix:main --force

# Or better: Create PR and merge properly
git push origin emergency-fix
# Then create pull request on GitHub
```

**Recovery from Backup Branch:**
```bash
# List available backups
git branch | grep backup

# Check backup content
git log backup-main-TIMESTAMP --oneline -5

# Restore from backup
git checkout backup-main-TIMESTAMP
git checkout -b main-restored-$(date +%Y%m%d-%H%M)

# Compare with current main
git diff main main-restored-TIMESTAMP

# If backup is correct, replace main
git branch -D main
git checkout -b main
git push origin main --force  # Coordinate with team!
```

---

## Verification Steps

### Pre-Merge Verification

**Branch State:**
- [ ] Currently on `main` branch
- [ ] Working tree is clean (`git status`)
- [ ] No uncommitted changes
- [ ] Backup branch created successfully

**Feature Branch State:**
- [ ] All changes committed on feature branch
- [ ] Documentation update included
- [ ] No uncommitted changes
- [ ] Patch file generated successfully

### Post-Merge Verification

**Files Exist:**
- [ ] `web_dashboard/app/templates/diagnostics.html` exists
- [ ] `web_dashboard/app/static/css/diagnostics.css` exists
- [ ] `110_WEB_UI_REFERENCE.md` updated

**File Contents Correct:**
- [ ] `diagnostics.html` contains doc-link elements
- [ ] Links point to correct GitHub URLs
- [ ] `.doc-link` class defined in CSS
- [ ] Documentation describes new feature

**Git State Clean:**
- [ ] `git status` shows clean working tree
- [ ] On `main` branch
- [ ] Commits appear in history
- [ ] No merge conflicts remain

### Post-Push Verification

**GitHub Verification:**
- [ ] Commits visible on GitHub
- [ ] File changes displayed correctly
- [ ] No security scan warnings
- [ ] Branch shows as up-to-date

**Functional Verification:**
- [ ] Web dashboard service can be started
- [ ] Diagnostics page loads without errors
- [ ] Documentation links are clickable
- [ ] Links open correct GitHub pages

---

## Security Checks

### Mandatory Pre-Commit Checks

**IP Address Scanning:**
```bash
# Real Tailscale IPs (100.x.x.x range)
grep -rE "100\.[0-9]+\.[0-9]+\.[0-9]+" <files>

# Local network IPs (192.168.x.x range)
grep -rE "192\.168\.[0-9]+\.[0-9]+" <files>
```

**Email Address Scanning:**
```bash
grep -rE "[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}" <files>
```

**Credential Scanning:**
```bash
# Look for potential secrets
grep -rE "(api[_-]?key|token|secret|password)" -i <files>
```

### Security-Sensitive Items (NEVER Commit)

**❌ Forbidden:**
- Real IP addresses (Tailscale, local network)
- API keys or authentication tokens
- SSH key fingerprints or private keys
- Real email addresses
- Passwords or credentials
- Database connection strings with credentials
- Certificate files or keys

**✅ Safe to Commit:**
- Placeholder IPs: `100.x.x.x`, `192.168.x.x`
- Example emails: `user@example.com`
- Environment variable names: `$OPENAI_API_KEY`
- Public documentation links
- Configuration templates without secrets
- Architecture diagrams with sanitized examples

### If Sensitive Data Found

**Immediate Actions:**
1. **DO NOT PUSH** if data is found before push
2. Remove or replace sensitive data
3. Stage and commit fixes
4. Re-run security checks
5. Only push after all checks pass

**Replacement Guidelines:**
```bash
# IP addresses
100.95.24.68 → 100.x.x.x
192.168.55.1 → 192.168.x.1

# Emails
real.user@gmail.com → user@example.com

# API keys
sk-proj-abc123xyz → $OPENAI_API_KEY

# Database strings
postgresql://user:pass@host/db → postgresql://$USER:$PASS@$HOST/$DB
```

**If Already Pushed:**
1. **Immediately** remove sensitive data
2. Create new commit with fixes
3. Push fix immediately
4. Rotate any exposed credentials (API keys, passwords)
5. Document incident in security log
6. Consider using `git filter-branch` if in commit history

### Reference Documentation

For complete security guidelines, see:
- `000_INTERNAL_AGENT_NOTES.md` (Security section)
- `011_SYSTEM_SECURITY.md`
- `000_AGENT_FINALIZATION_GUIDE.md` (Phase 5: Security Pre-Commit Check)

---

## Expected Results

### After Successful Completion

**Git State:**
- ✅ Main branch contains both red status logic and diagnostics links
- ✅ All changes committed with descriptive messages
- ✅ Working tree clean (no uncommitted changes)
- ✅ Pushed to GitHub successfully
- ✅ Backup branch exists for safety

**File State:**
- ✅ `diagnostics.html` has doc links under every button
- ✅ `diagnostics.css` has `.doc-link` styling
- ✅ `110_WEB_UI_REFERENCE.md` documents the feature

**Functional State:**
- ✅ Web dashboard can be started
- ✅ Diagnostics page loads without errors
- ✅ Documentation links work correctly
- ✅ Links open in new tab
- ✅ No JavaScript console errors

**Documentation State:**
- ✅ Feature documented in 110_WEB_UI_REFERENCE.md
- ✅ Merge plan documented in this file (999_diagnostics_documentation_links.md)
- ✅ Git history shows clear commit messages
- ✅ No duplicate documentation created

### Feature Availability

**When Active:**
- Web dashboard service must be running: `sudo systemctl start r2d2-web-dashboard.service`
- Access via: `http://100.x.x.x:8080/diagnostics`
- Links visible under each topic monitor and diagnostic test button

**User Experience:**
- Users see 📖 Docs link under each button
- Links are small and unobtrusive (gray text)
- Hover effect changes color to blue
- Click opens GitHub documentation in new tab
- Diagnostics page remains active (preserved state)

---

## Risk Assessment

### Risk Level: 🟢 LOW

**Why Low Risk:**
- ✅ Patch mechanism is Git's safest merge method
- ✅ Backup branch provides instant rollback
- ✅ Files don't exist on main, so no true merge conflicts
- ✅ Changes are purely additive (no deletions)
- ✅ No modification of existing functionality
- ✅ Security checks prevent data leaks
- ✅ Step-by-step verification catches issues early

### Potential Issues and Mitigation

**Issue 1: Patch Fails to Apply**
- **Probability:** Low (files don't exist on main)
- **Impact:** Moderate (requires manual resolution)
- **Mitigation:** Step-by-step instructions for conflict resolution
- **Fallback:** Abort and use manual file copy method

**Issue 2: Backup Not Created**
- **Probability:** Very Low (explicit step in plan)
- **Impact:** High (harder to rollback)
- **Mitigation:** Mandatory verification after backup creation
- **Fallback:** Use `git reflog` for recovery

**Issue 3: Sensitive Data Committed**
- **Probability:** Very Low (thorough pre-push checks)
- **Impact:** High (security breach)
- **Mitigation:** Multiple security check steps
- **Fallback:** Immediate credential rotation, `git filter-branch`

**Issue 4: Push Conflicts with Remote**
- **Probability:** Low (single developer)
- **Impact:** Low (just need to pull and retry)
- **Mitigation:** Pull before push, resolve conflicts
- **Fallback:** Force push to backup branch, investigate

**Issue 5: Web Dashboard Breaks**
- **Probability:** Very Low (HTML/CSS only, tested)
- **Impact:** Moderate (dashboard inaccessible)
- **Mitigation:** Changes are purely additive, validated syntax
- **Fallback:** Rollback to backup branch, restart service

### Success Criteria

**Merge Successful If:**
- ✅ All commits appear in `git log`
- ✅ `git status` shows clean working tree
- ✅ All files exist with correct content
- ✅ Security checks pass
- ✅ Push completes without errors
- ✅ GitHub shows commits in history

**Feature Working If:**
- ✅ Web dashboard service starts
- ✅ Diagnostics page loads
- ✅ Documentation links are visible
- ✅ Links open correct pages
- ✅ No console errors

---

## Troubleshooting Guide

### Problem: Cherry-pick Won't Abort

**Symptoms:**
- `git cherry-pick --abort` fails
- Error message about uncommitted changes

**Solution:**
```bash
# Check what's uncommitted
git status

# Save changes if important
git stash push -m "temp save"

# Try abort again
git cherry-pick --abort

# Restore if needed
git stash pop
```

### Problem: Patch Application Fails

**Symptoms:**
- `git am` shows "patch does not apply"
- Conflicts in specific files

**Solution:**
```bash
# Check which file has conflict
git status

# View the conflict
cat <file-with-conflict>

# Option 1: Resolve manually
vim <file-with-conflict>
# Edit to resolve conflict
git add <file-with-conflict>
git am --continue

# Option 2: Abort and try different approach
git am --abort
# Use manual file copy method instead
```

### Problem: Files Don't Exist After Merge

**Symptoms:**
- Merge completed but files missing
- `ls` doesn't show expected files

**Solution:**
```bash
# Check if files are in staging area
git ls-files | grep diagnostics

# Check if commit was actually applied
git log --oneline -3

# If commit missing, reapply patch
git am /tmp/diagnostics-feature.patch

# If files in wrong location, check path
find . -name "diagnostics.html"
```

### Problem: Push Rejected

**Symptoms:**
- `git push` fails with "rejected"
- Remote has commits you don't have

**Solution:**
```bash
# Fetch and check remote state
git fetch origin
git log origin/main..HEAD --oneline

# Pull with rebase to preserve your commits
git pull --rebase origin main

# Resolve conflicts if any
# Then push
git push origin main
```

### Problem: Backup Branch Not Found

**Symptoms:**
- Can't find backup branch to rollback
- `git branch` doesn't show backup

**Solution:**
```bash
# List all branches including remote
git branch -a | grep backup

# Use reflog to find previous state
git reflog | head -20

# Recover from reflog
git checkout HEAD@{N}  # Replace N with entry number
git checkout -b recovered-main
```

### Problem: Documentation Links Don't Work

**Symptoms:**
- Links visible but go to 404 page
- Wrong GitHub repository or file path

**Solution:**
```bash
# Check actual link in file
grep "github.com" web_dashboard/app/templates/diagnostics.html

# Verify GitHub URLs are correct
# Should be: https://github.com/severinleuenberger/R2D2-as-real-AI-companion/blob/main/dev/r2d2/

# If wrong, edit and recommit
vim web_dashboard/app/templates/diagnostics.html
# Fix URLs
git add web_dashboard/app/templates/diagnostics.html
git commit -m "fix: Correct GitHub documentation URLs"
git push origin main
```

---

## Quick Reference Commands

### Essential Git Commands

```bash
# Status and branch info
git status                    # Check working tree
git branch                    # List branches
git log --oneline -5          # Recent commits

# Branch operations
git checkout <branch>         # Switch branch
git checkout -b <name>        # Create new branch

# Staging and committing
git add <file>                # Stage file
git add .                     # Stage all changes
git commit -m "message"       # Commit with message

# Merging operations
git cherry-pick --abort       # Abort cherry-pick
git am --abort                # Abort patch application
git am --continue             # Continue after resolving conflict

# Remote operations
git fetch origin              # Fetch remote changes
git pull origin main          # Pull from main
git push origin main          # Push to main

# Safety operations
git stash push -m "desc"      # Save uncommitted changes
git stash pop                 # Restore stashed changes
git reflog                    # View reference log
```

### File Verification Commands

```bash
# Check if files exist
ls -la web_dashboard/app/templates/diagnostics.html
ls -la web_dashboard/app/static/css/diagnostics.css
ls -la 110_WEB_UI_REFERENCE.md

# View file content
cat <file>                    # Full content
head -20 <file>               # First 20 lines
tail -20 <file>               # Last 20 lines
grep "pattern" <file>         # Search in file

# Find files
find . -name "diagnostics.html"
find . -type f -name "*.css" | grep diagnostics
```

### Security Check Commands

```bash
# IP address scans
grep -rE "100\.[0-9]+\.[0-9]+\.[0-9]+" <files>
grep -rE "192\.168\.[0-9]+\.[0-9]+" <files>

# Email scan
grep -rE "[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}" <files>

# Credential scan
grep -rE "(api[_-]?key|token|secret|password)" -i <files>
```

---

## Related Documentation

**R2D2 Project Documentation:**
- `000_AGENT_FINALIZATION_GUIDE.md` - Production deployment procedures
- `000_INTERNAL_AGENT_NOTES.md` - Agent development guidelines
- `110_WEB_UI_REFERENCE.md` - Web UI complete reference
- `111_WEB_UI_INSTALLATION.md` - Web UI installation guide
- `112_WEB_UI_QUICK_START.md` - Web UI quick start
- `011_SYSTEM_SECURITY.md` - Security guidelines

**Feature-Specific Files:**
- `web_dashboard/app/templates/diagnostics.html` - Diagnostics page template
- `web_dashboard/app/static/css/diagnostics.css` - Diagnostics page styling
- `web_dashboard/app/static/js/diagnostics.js` - Diagnostics page logic

**Git References:**
- Git Patches: https://git-scm.com/docs/git-format-patch
- Git AM: https://git-scm.com/docs/git-am
- Git Reflog: https://git-scm.com/docs/git-reflog

---

## Execution Checklist

Use this checklist when executing the merge:

### Pre-Merge
- [ ] Read complete plan
- [ ] Understand current git state
- [ ] Have terminal access
- [ ] No critical processes running

### Merge Process
- [ ] Clean up git state (abort cherry-pick)
- [ ] Create backup branch
- [ ] Prepare feature branch (commit all changes)
- [ ] Generate patch file
- [ ] Apply patch to main
- [ ] Verify merge success
- [ ] Run security checks
- [ ] Push to GitHub
- [ ] Clean up temporary files

### Post-Merge
- [ ] Verify commits on GitHub
- [ ] Check file contents
- [ ] Test web dashboard (optional)
- [ ] Document any issues encountered
- [ ] Keep backup branch for 24+ hours

---

## Version History

**v1.0 - January 5, 2026**
- Initial documentation
- Complete merge plan created
- Safety procedures documented
- Troubleshooting guide added

---

## Notes for Future Reference

**Why This Approach?**
- Parallel development required safe merge strategy
- Patch method chosen for maximum safety
- Backup branch provides recovery point
- Step-by-step verification catches issues early

**Lessons Learned:**
- Feature branches should be merged regularly to avoid complex merge scenarios
- Always create backup before complex git operations
- Security checks must be automated and mandatory
- Clear documentation prevents confusion during execution

**Future Improvements:**
- Consider automated security scanning before push
- Implement pre-commit hooks for security checks
- Set up CI/CD pipeline for automatic testing
- Create merge request workflow for code review

---

**END OF DOCUMENT**

