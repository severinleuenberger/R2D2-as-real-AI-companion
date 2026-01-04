# Testing Safety Infrastructure & WebUI Fixes
## Systematic Verification Guide

**Created:** January 4, 2026  
**Purpose:** Verify all safety and recovery systems are working

---

## What We Created

### 1. Comprehensive Documentation
- **004_BACKUP_AND_RESTORE.md** (2,026 lines) - Single source of truth for ALL recovery procedures

### 2. Safety Scripts (5 scripts in `scripts/safety/`)
- `run_preflight_checks.sh` - Pre-flight verification
- `create_service_backup.sh` - Backup systemd files
- `create_git_safety_tag.sh` - Create safety tag
- `rollback_to_safety_tag.sh` - Rollback to recent tag
- `rollback_to_last_golden.sh` - Nuclear option
- `fix_heartbeat_service.sh` - Fix heartbeat service path

### 3. WebUI Enhancements
- Enhanced `start_web_dashboard.sh` - Auto-starts rosbridge with health checks
- New `test_webui_health.sh` - Smoke test for WebUI
- New `diagnostics.py` API - Hardware test endpoints
- Updated `main.py` - Includes diagnostics router

### 4. Service Hardening
- Script to add resource limits - `add_webui_resource_limits.sh`

---

## Test Plan

### Phase 1: Documentation Verification (2 minutes)

**Test 1.1: Verify 004_BACKUP_AND_RESTORE.md exists and is comprehensive**

```bash
# Check file exists and size
ls -lh ~/dev/r2d2/004_BACKUP_AND_RESTORE.md

# Should show: ~140K, 2026 lines

# Verify new sections present
grep "## Multi-Level Recovery Strategy" ~/dev/r2d2/004_BACKUP_AND_RESTORE.md
grep "## Development Safety Net" ~/dev/r2d2/004_BACKUP_AND_RESTORE.md
grep "## Emergency Rollback Reference" ~/dev/r2d2/004_BACKUP_AND_RESTORE.md
grep "## Helper Scripts Reference" ~/dev/r2d2/004_BACKUP_AND_RESTORE.md

# All should return matches ✅
```

**Test 1.2: Verify redundant docs deleted**

```bash
# These should NOT exist
ls ~/EMERGENCY_ROLLBACK_CARD.md 2>&1
ls ~/dev/r2d2/web_dashboard/RECOVERY_RUNBOOK.md 2>&1
ls ~/dev/r2d2/scripts/safety/README.md 2>&1

# All should show: "No such file or directory" ✅
```

**Expected Result:** One comprehensive document, no duplicates ✅

---

### Phase 2: Safety Scripts Verification (5 minutes)

**Test 2.1: Check all scripts exist and are executable**

```bash
ls -lh ~/dev/r2d2/scripts/safety/

# Should show 6 executable scripts:
# - run_preflight_checks.sh
# - create_service_backup.sh
# - create_git_safety_tag.sh
# - rollback_to_safety_tag.sh
# - rollback_to_last_golden.sh
# - fix_heartbeat_service.sh
# - add_webui_resource_limits.sh

# All should have: -rwxrwxr-x (executable)
```

**Test 2.2: Run pre-flight checks (safe, read-only)**

```bash
~/dev/r2d2/scripts/safety/run_preflight_checks.sh
```

**Expected output:**
```
✅ Passed: 8
❌ Failed: 0

Manual Steps Required:
1. Create service backup: sudo ~/dev/r2d2/scripts/safety/create_service_backup.sh
2. Create git safety tag: ~/dev/r2d2/scripts/safety/create_git_safety_tag.sh
```

**If any failures:** Note them, but continue testing

---

### Phase 3: Test Service Backup (requires sudo)

**Test 3.1: Create service backup**

```bash
sudo ~/dev/r2d2/scripts/safety/create_service_backup.sh
```

**Expected output:**
```
Creating service backup...
Backup directory: /home/severin/r2d2_service_backups/20260104_HHMMSS_pre_webui_changes

✅ Backup complete!

Files backed up:
r2d2-audio-notification.service
r2d2-camera-perception.service
...

To restore:
  sudo cp /home/severin/r2d2_service_backups/20260104_HHMMSS_pre_webui_changes/*.service /etc/systemd/system/
  sudo systemctl daemon-reload
```

**Test 3.2: Verify backup created**

```bash
ls -lh ~/r2d2_service_backups/

# Should show timestamped directory with .service files
```

---

### Phase 4: Test Git Safety Tag

**Test 4.1: Create safety tag**

```bash
~/dev/r2d2/scripts/safety/create_git_safety_tag.sh
```

**Expected output:**
```
🔖 Creating Git Safety Tag
==========================

Current branch: feature/face-tracking-tilt-servo
Current commit: 217b0bfd docs: Update pan/tilt plan

✅ Tag created successfully!

Tag: golden-pre-webui-2026-01-04

To push to remote (recommended):
  git push origin golden-pre-webui-2026-01-04
```

**Test 4.2: Verify tag exists**

```bash
cd ~/dev/r2d2
git tag | grep golden-pre-webui-2026-01-04

# Should show the tag ✅

git show golden-pre-webui-2026-01-04 --no-patch

# Should show tag message with service status
```

**Test 4.3: Optional - Push to remote**

```bash
cd ~/dev/r2d2
git push origin golden-pre-webui-2026-01-04

# Only if you want remote backup of this tag
```

---

### Phase 5: Test WebUI Components

**Test 5.1: Check if WebUI is currently running**

```bash
systemctl status r2d2-web-dashboard --no-pager

# If inactive, that's OK (it's disabled by default)
# If active, we can test it directly
```

**Test 5.2: Start WebUI manually (if not running)**

```bash
cd ~/dev/r2d2/web_dashboard

# Start in background
./scripts/start_web_dashboard.sh &

# Wait 10 seconds for rosbridge to initialize
sleep 10
```

**Test 5.3: Run WebUI health check**

```bash
~/dev/r2d2/web_dashboard/scripts/test_webui_health.sh
```

**Expected output:**
```
✅ Passed: 6/6

✅ All WebUI health checks passed!

WebUI should be accessible at: http://100.x.x.x:8080
```

**Test 5.4: Test API endpoints**

```bash
# Health endpoint
curl http://100.x.x.x:8080/health

# Should return: {"status":"ok","service":"r2d2-web-dashboard"}

# Diagnostics mode endpoint
curl http://100.x.x.x:8080/api/diagnostics/mode

# Should return: {"mode":"parallel",...}

# Services status
curl http://100.x.x.x:8080/api/services/status | python3 -m json.tool | head -20

# Should show JSON with service statuses
```

---

### Phase 6: Test Diagnostics API

**Test 6.1: Camera detection**

```bash
curl -X POST http://100.x.x.x:8080/api/diagnostics/hardware/camera/detect

# Expected: {"status":"PASS","device_found":true,...}
```

**Test 6.2: Camera pipeline**

```bash
curl -X POST http://100.x.x.x:8080/api/diagnostics/hardware/camera/pipeline

# Expected: {"status":"PASS","fps":30.0,...} (if camera-perception running)
```

**Test 6.3: Microphone detection**

```bash
curl -X POST http://100.x.x.x:8080/api/diagnostics/hardware/mic/detect

# Expected: {"status":"PASS","device_found":true,...}
```

**Test 6.4: Speaker detection**

```bash
curl -X POST http://100.x.x.x:8080/api/diagnostics/hardware/speaker/detect

# Expected: {"status":"PASS","device_found":true,...}
```

**Test 6.5: Speaker tone test**

```bash
curl -X POST http://100.x.x.x:8080/api/diagnostics/hardware/speaker/tone

# Should play 800Hz beep - listen for it!
# Expected: {"status":"PASS","message":"Test tone played...",...}
```

**Test 6.6: Bluetooth status**

```bash
curl -X POST http://100.x.x.x:8080/api/diagnostics/hardware/bluetooth/service

# Expected: {"status":"PASS","service_active":true}
```

---

### Phase 7: Test Mode Switching API

**Test 7.1: Check current mode**

```bash
curl http://100.x.x.x:8080/api/diagnostics/mode | python3 -m json.tool
```

**Expected:** Should show "parallel" mode with core services running

**Test 7.2: Verify mode switching exists (don't actually switch yet)**

```bash
# Just check the endpoints exist
curl -X POST http://100.x.x.x:8080/api/diagnostics/mode/debug --head
curl -X POST http://100.x.x.x:8080/api/diagnostics/mode/parallel --head

# Both should return: HTTP/1.1 200 OK (or 403 if mode requirements not met)
```

---

### Phase 8: Verify Fault Isolation

**Test 8.1: Check WebUI can't break core services**

```bash
# Core services should all be running
systemctl status r2d2-camera-perception --no-pager | grep "Active:"
systemctl status r2d2-audio-notification --no-pager | grep "Active:"
systemctl status r2d2-gesture-intent --no-pager | grep "Active:"

# All should show: Active: active (running) ✅

# WebUI runs separately
ps aux | grep "python3 -m app.main"

# Should show WebUI process (different PID from core services)
```

**Test 8.2: Quick UX verification**

```bash
# Run minimal monitor for 30 seconds
~/dev/r2d2/tools/minimal_monitor.py

# Stand in front of camera and move in/out of view
# Should see: BLUE → RED → BLUE transitions
# This confirms core UX is unaffected by WebUI
```

---

## Test Results Checklist

Mark each as you complete:

### Documentation
- [ ] 004_BACKUP_AND_RESTORE.md exists (2,026 lines)
- [ ] New sections present (Multi-Level, Safety Net, Emergency Reference, Helper Scripts)
- [ ] Redundant docs deleted (3 files)
- [ ] Cross-references updated in 110_WEB_UI_REFERENCE.md

### Safety Scripts
- [ ] All 6 scripts exist in `scripts/safety/`
- [ ] All scripts executable (chmod +x)
- [ ] Pre-flight checks run successfully
- [ ] Service backup created successfully
- [ ] Git safety tag created successfully

### WebUI Components
- [ ] start_web_dashboard.sh enhanced with rosbridge health checks
- [ ] test_webui_health.sh smoke test created
- [ ] Diagnostics API created with all endpoints
- [ ] main.py updated with diagnostics router
- [ ] WebUI starts and responds to health endpoint

### API Endpoints
- [ ] GET /api/diagnostics/mode - Returns current mode
- [ ] POST /api/diagnostics/hardware/camera/detect - Camera detection works
- [ ] POST /api/diagnostics/hardware/mic/detect - Microphone detection works
- [ ] POST /api/diagnostics/hardware/speaker/detect - Speaker detection works
- [ ] POST /api/diagnostics/hardware/speaker/tone - Tone plays
- [ ] POST /api/diagnostics/hardware/bluetooth/service - Bluetooth check works

### Fault Isolation
- [ ] WebUI runs in separate process from core services
- [ ] Core services (camera-perception, audio-notification) remain active
- [ ] UX features working (recognition, gestures, speech)
- [ ] minimal_monitor.py shows RED/BLUE transitions

---

## Next Steps After Testing

**If all tests pass:**
1. Optionally push safety tag to remote: `git push origin golden-pre-webui-2026-01-04`
2. Proceed with remaining WebUI implementation (debug HTML page, mode switching UI)
3. Create new golden tag after successful completion

**If any tests fail:**
1. Note which test failed
2. Use appropriate recovery level from 004_BACKUP_AND_RESTORE.md
3. Investigate root cause
4. Fix and re-test

---

**Start here:** `~/dev/r2d2/scripts/safety/run_preflight_checks.sh`

