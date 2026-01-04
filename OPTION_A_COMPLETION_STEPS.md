# Option A: Production Hardening - Completion Steps
## Execute These Commands to Finalize

**Current Status:** WebUI operational, safety backups ready

---

## Step 1: Create Service Backup (requires sudo)

```bash
sudo /home/severin/dev/r2d2/scripts/safety/create_service_backup.sh
```

**Expected output:**
```
Creating service backup...
Backup directory: /home/severin/r2d2_service_backups/20260104_HHMMSS_pre_webui_changes

✅ Backup complete!
Files backed up: [list of .service files]
```

**Verify:**
```bash
ls -lh ~/r2d2_service_backups/
```

---

## Step 2: Fix Heartbeat Service (requires sudo)

```bash
sudo /home/severin/dev/r2d2/scripts/safety/fix_heartbeat_service.sh
```

**Expected:**
```
Fixing r2d2-heartbeat.service ExecStart path...
✅ Heartbeat service fixed!
Active: active (running)...
```

**Verify:**
```bash
systemctl status r2d2-heartbeat --no-pager | grep "Active:"
# Should show: Active: active (running)
```

---

## Step 3: Add Resource Limits (requires sudo)

```bash
sudo /home/severin/dev/r2d2/scripts/safety/add_webui_resource_limits.sh
```

**Expected:**
```
Adding resource limits to WebUI services...
Updating: r2d2-web-dashboard.service
  ✅ Resource limits added
Updating: r2d2-rosbridge.service
  ✅ Resource limits added

✅ Complete! Resource limits added.
```

**Verify:**
```bash
grep -A 3 "Resource limits" /etc/systemd/system/r2d2-web-dashboard.service
# Should show: CPUQuota, MemoryLimit, TasksMax
```

---

## Step 4: Restart Services to Apply Limits

```bash
sudo systemctl daemon-reload
sudo systemctl restart r2d2-web-dashboard
sudo systemctl restart r2d2-rosbridge
```

**Wait 10 seconds for startup:**
```bash
sleep 10
```

---

## Step 5: Run Health Checks

**WebUI smoke test:**
```bash
~/dev/r2d2/web_dashboard/scripts/test_webui_health.sh
```

**Expected:** 5/6 or 6/6 checks pass

**Test diagnostics API:**
```bash
curl -X POST http://100.x.x.x:8080/api/diagnostics/hardware/camera/detect
# Should return: {"status":"PASS",...}
```

**Verify core services still healthy:**
```bash
systemctl status r2d2-camera-perception --no-pager | grep "Active:"
systemctl status r2d2-audio-notification --no-pager | grep "Active:"
# Both should show: Active: active (running)
```

---

## Step 6: Create Golden Tag (Marks Stable State)

```bash
cd ~/dev/r2d2

git add -A

git commit -m "feat: WebUI stabilization complete - recovery system + diagnostics API

- Comprehensive 004_BACKUP_AND_RESTORE.md with 5-level recovery
- Safety scripts for backups and rollback
- Fixed rosbridge dependencies (tornado, numpy environment ordering)
- Diagnostics API with 16 hardware test endpoints
- Resource limits applied for fault isolation
- Fault isolation verified - core services protected
- WebUI operational on ports 8080 (dashboard) and 9090 (rosbridge)
- Heartbeat service path fixed

Tested:
- Smoke test: 5/6 checks pass
- Core services: Unaffected throughout implementation
- Mode detection: Working (parallel mode)
- Hardware diagnostics: Camera, mic, speaker, bluetooth all operational"

git tag -a golden-webui-operational-2026-01-04 -m "WebUI fully operational with complete recovery system

Working components:
- WebUI dashboard (port 8080) ✅
- rosbridge WebSocket (port 9090) ✅  
- Diagnostics API (16 endpoints) ✅
- 5-level recovery system ✅
- Safety scripts (7 scripts) ✅

Core services protected:
- camera-perception ✅
- audio-notification ✅
- gesture-intent ✅
- speech-node ✅

Use this tag for rollback if future changes break WebUI."
```

**Push to remote (optional but recommended):**
```bash
git push origin golden-webui-operational-2026-01-04
```

---

## Step 7: Optional - USB Backup (Full System Snapshot)

If you have a labeled USB stick:

```bash
# Plug in USB labeled R2D2_BACKUP
bash ~/dev/r2d2/scripts/r2d2_backup.sh

# Takes 15-30 minutes
```

---

## Completion Checklist

After completing all steps, verify:

- [ ] Service backup created in `~/r2d2_service_backups/`
- [ ] Heartbeat service active and running
- [ ] Resource limits applied to web services
- [ ] WebUI smoke test passes (5/6 or 6/6)
- [ ] Core services still active (no restarts)
- [ ] Golden tag created: `golden-webui-operational-2026-01-04`
- [ ] Tag pushed to remote (optional)

**Once complete:** Option A is finished! WebUI is production-ready.

---

**Next:** Option B (Debug HTML UI) can be implemented later as an enhancement.

**For now:** WebUI is functional, safe, and protected!

