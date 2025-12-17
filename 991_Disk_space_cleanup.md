# Disk Space Cleanup Documentation

**Date:** December 17, 2025  
**Status:** Ongoing maintenance documentation  
**Purpose:** Track disk space cleanup activities, regular maintenance tasks, and remaining opportunities

---

## Current System Status

**Disk:** 57GB total, currently ~47GB used (~87% capacity, 7.1GB free)  
**Platform:** NVIDIA Jetson AGX Orin 64GB  
**OS:** Ubuntu 22.04 Jammy

---

## What We've Done (December 17, 2025)

### Phase 1: Cache Cleanup (Completed) ✅

**Space Recovered:** ~142MB

1. **Epiphany Browser Cache** - 106MB
   - Location: `~/.cache/epiphany/`
   - Action: Removed with `rm -rf ~/.cache/epiphany`
   - Status: ✅ Deleted - regenerates automatically

2. **Torch Hub Cache** - 34MB
   - Location: `~/.cache/torch/hub/`
   - Action: Removed with `rm -rf ~/.cache/torch/hub`
   - Status: ✅ Deleted - models re-download when accessed

3. **Pip Cache** - 2.3MB
   - Location: `~/.cache/pip/`
   - Action: Purged with `pip cache purge`
   - Status: ✅ Cleaned - packages re-download on install

4. **Old ROS Logs** - ~20MB
   - Location: `~/.ros/log/`
   - Action: Removed logs older than 7 days
   - Command: `find ~/.ros/log -name "*.log" -mtime +7 -delete`
   - Status: ✅ Cleaned - kept last 7 days

### Git Repository Optimization (Completed) ✅

**Action:** Ran standard git maintenance
- `git reflog expire --expire=90.days.ago --all`
- `git prune --expire=90.days.ago`
- `git gc --prune=now`

**Result:** Repository optimized, all objects consolidated into pack files
- **Repository size:** 361MB (optimized, no loose objects)

### Archive to USB (Completed) ✅

**USB Location:** `/media/severin/DAEA-DEBB/r2d2_archive/`

1. **Backups Archived** - 371MB
   - All files copied to USB: `r2d2_archive/backups/`
   - Files: 
     - `r2d2_backup_20251213_105223.tar.gz` (346MB)
     - `r2d2_backup_20251209_083400.tar.gz` (25MB)
     - `backup_log_20251207_090424.txt` (1.1KB)
   - Status: ✅ Archived - originals still on disk (can be removed)

### Previously Removed (Before Documentation)

- **HuggingFace Model Cache** - 2.9GB
  - Location: `~/.cache/huggingface/`
  - Model: faster-whisper-large-v2
  - Status: ✅ Removed - model re-downloads on first use if needed

---

## Regular Maintenance Tasks

### Weekly/Monthly Tasks

#### 1. Clean APT Package Cache (Recommended: Monthly)
**Space Potential:** ~400-500MB

```bash
# Check cache size first
sudo du -sh /var/cache/apt

# Clean APT cache
sudo apt clean

# Or more aggressive cleanup
sudo apt-get clean
sudo apt-get autoclean
sudo apt-get autoremove
```

**Why:** APT caches downloaded .deb files. Safe to remove, re-downloads on next install.

#### 2. Rotate System Logs (Recommended: Monthly)
**Space Potential:** ~150-200MB

```bash
# Check log sizes
sudo du -sh /var/log/* | sort -hr

# Rotate logs (keeps compressed archives)
sudo logrotate -f /etc/logrotate.conf

# Or manually truncate large logs (if needed)
sudo truncate -s 0 /var/log/syslog
sudo truncate -s 0 /var/log/kern.log

# Clean old journal logs
sudo journalctl --vacuum-time=30d  # Keep last 30 days
sudo journalctl --vacuum-size=100M  # Or limit to 100MB
```

**Why:** System logs grow over time. Rotation keeps recent logs, archives old ones.

#### 3. Clean User Caches (Recommended: Monthly)
**Space Potential:** ~100-200MB

```bash
# Browser caches (if using browsers)
rm -rf ~/.cache/epiphany
rm -rf ~/.cache/mozilla
rm -rf ~/.cache/chromium

# Python package caches
pip cache purge

# ROS logs (keep last 7 days)
find ~/.ros/log -name "*.log" -mtime +7 -delete
```

#### 4. Git Repository Maintenance (Recommended: Quarterly)
**Space Potential:** Variable (optimization, not necessarily space recovery)

```bash
cd ~/dev/r2d2
git reflog expire --expire=90.days.ago --all
git prune --expire=90.days.ago
git gc --prune=now
```

**Why:** Keeps repository optimized, consolidates objects.

---

## Remaining Cleanup Opportunities

### High Priority (Safe, Significant Impact)

#### 1. Remove Archived Backups from Disk (371MB) ⭐
**Location:** `~/backups/`
**Status:** Already archived to USB at `/media/severin/DAEA-DEBB/r2d2_archive/backups/`

```bash
# Verify USB archive exists and is complete
ls -lh /media/severin/DAEA-DEBB/r2d2_archive/backups/

# If verified, remove from disk
rm -rf ~/backups/*
```

**Risk:** ✅ Safe - files are backed up on USB  
**Impact:** 371MB

#### 2. Clean APT Package Cache (416MB) ⭐
**Location:** `/var/cache/apt/`

```bash
sudo apt clean
```

**Risk:** ✅ Safe - packages re-download on next install  
**Impact:** 416MB

#### 3. Rotate System Logs (196MB) ⭐
**Location:** `/var/log/`

```bash
# Rotate logs (recommended)
sudo logrotate -f /etc/logrotate.conf

# Or truncate if rotation doesn't help
sudo truncate -s 0 /var/log/syslog
sudo truncate -s 0 /var/log/kern.log
```

**Risk:** ✅ Safe - logs regenerate, old logs archived  
**Impact:** 196MB

#### 4. Remove Duplicate Piper TTS Models (191MB) ⚠️
**Locations:**
- `~/.local/share/piper-tts/` (191MB)
- `~/.local/share/piper_tts/` (191MB)

**Action Required:** First verify which directory path is used in code, then remove duplicate

```bash
# Find which path is referenced
grep -r "piper-tts\|piper_tts" ~/dev/r2d2/r2d2_speech --include="*.py"

# Once verified, remove the unused directory
# (Keep the one that's actually used)
```

**Risk:** ⚠️ Moderate - must verify which is used first  
**Impact:** 191MB

### Medium Priority (Verify Before Removal)

#### 5. OTA Update Packages (343MB) ⚠️
**Location:** `/opt/ota_package/`

**Note:** These may be needed for future Jetson OTA updates. Verify if system updates are complete before removing.

```bash
# Check what's in there
ls -lh /opt/ota_package/

# If no longer needed (verify first!)
sudo rm -rf /opt/ota_package/*
```

**Risk:** ⚠️ Moderate - may be needed for system updates  
**Impact:** 343MB

#### 6. Archive Documentation to USB (~15MB)
**Locations:**
- `~/dev/r2d2/docs/photos/` (15MB)
- `~/dev/r2d2/_ANALYSIS_AND_DOCUMENTATION/` (340KB)

**Action:** Archive to USB, then optionally remove from disk if no longer needed for reference

```bash
USB_ARCHIVE=/media/severin/DAEA-DEBB/r2d2_archive
mkdir -p $USB_ARCHIVE/docs
cp -r ~/dev/r2d2/docs/photos $USB_ARCHIVE/docs/
cp -r ~/dev/r2d2/_ANALYSIS_AND_DOCUMENTATION $USB_ARCHIVE/analysis/
```

**Risk:** ✅ Safe - documentation can be archived  
**Impact:** 15MB

### Low Priority (Small Impact, But Easy)

#### 7. Remove Duplicate Audio Files (~36KB)
**Location:** `~/Voicy_R2-D2 - {2,3,5}.mp3`

**Note:** These already exist in project at `~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/`

```bash
rm ~/Voicy_R2-D2*.mp3
```

**Risk:** ✅ Safe - duplicates exist in project  
**Impact:** 36KB

#### 8. Clean Old Debug Images (~1MB)
**Locations:**
- `~/dev/r2d2/tests/camera/*.jpg`
- `~/dev/r2d2/debug_frame.jpg`
- `~/dev/r2d2/oak_d_photo.jpg`

**Action:** Archive or remove old test/debug images

```bash
# Archive first (optional)
USB_ARCHIVE=/media/severin/DAEA-DEBB/r2d2_archive
mkdir -p $USB_ARCHIVE/test_images
cp ~/dev/r2d2/tests/camera/*.jpg $USB_ARCHIVE/test_images/
cp ~/dev/r2d2/debug_frame.jpg ~/dev/r2d2/oak_d_photo.jpg $USB_ARCHIVE/test_images/

# Then remove (if archived)
rm ~/dev/r2d2/debug_frame.jpg ~/dev/r2d2/oak_d_photo.jpg
find ~/dev/r2d2/tests/camera -name "*debug*.jpg" -o -name "*test*.jpg" -delete
```

**Risk:** ✅ Safe - test images can be regenerated  
**Impact:** ~1MB

---

## System Directories Analysis

### Required Directories (DO NOT DELETE)

- **`/opt/nvidia/`** - 2.4GB - NVIDIA drivers (required for Jetson)
- **`/opt/ros/`** - 283MB - ROS installation (required)
- **`/snap/`** - 5.9GB - Snap packages (required, normal size)
- **`~/.local/share/`** - Virtual environments and model files (required)
- **`~/dev/r2d2/r2d2_speech_env/`** - 1.3GB - Python virtual environment (required)
- **`~/depthai_env/`** - 1.5GB - Python virtual environment (required)
- **`~/dev/r2d2/.git/`** - 361MB - Git repository (required)

### Cache Directories (Can Clean Regularly)

- **`/var/cache/apt/`** - 416MB - APT package cache (safe to clean)
- **`~/.cache/pip/`** - ~2MB - Python pip cache (safe to clean)
- **`~/.cache/torch/`** - Variable - PyTorch cache (safe to clean)
- **`~/.cache/epiphany/`** - Variable - Browser cache (safe to clean)
- **`~/.cache/huggingface/`** - Variable - Model cache (safe to clean, re-downloads)

### Log Directories (Can Rotate/Clean)

- **`/var/log/`** - 202MB - System logs (can rotate)
- **`~/.ros/log/`** - ~77MB - ROS logs (can clean old)
- **Journal logs** - 24MB - systemd journal (can vacuum)

---

## Cleanup Summary by Impact

### Immediate Safe Cleanup (Available Now)
- Remove archived backups from disk: **371MB**
- Clean APT cache: **416MB**
- Rotate system logs: **196MB**
- **Total: ~983MB (~1GB)**

### After Verification
- Remove duplicate Piper TTS: **191MB**
- Remove OTA packages (if not needed): **343MB**
- **Total: ~534MB**

### Archive to USB (Then Remove from Disk)
- Documentation photos: **15MB**
- Test/debug images: **1MB**
- **Total: ~16MB**

### Grand Total Potential
**~1.5GB additional space recovery possible**

---

## Quick Maintenance Script

Create a script for regular maintenance:

```bash
#!/bin/bash
# ~/dev/r2d2/scripts/disk_cleanup.sh

echo "=== Disk Space Cleanup ==="
echo ""

# Clean APT cache
echo "Cleaning APT cache..."
sudo apt clean
echo ""

# Clean pip cache
echo "Cleaning pip cache..."
pip cache purge 2>/dev/null || echo "pip cache already clean"
echo ""

# Clean ROS logs (keep last 7 days)
echo "Cleaning old ROS logs..."
find ~/.ros/log -name "*.log" -mtime +7 -delete
echo ""

# Clean browser caches
echo "Cleaning browser caches..."
rm -rf ~/.cache/epiphany 2>/dev/null
echo ""

# Show disk usage
echo "=== Current Disk Usage ==="
df -h /home/severin | tail -1
```

---

## Notes

- Always verify USB archives before deleting originals
- System logs can be important for debugging - keep recent logs
- Virtual environments are required - never delete
- Git repository size is normal for active projects
- Regular maintenance prevents disk space issues

---

## Last Updated

December 17, 2025

