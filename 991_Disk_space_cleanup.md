# Disk Space Cleanup Documentation

**Date:** December 19, 2025  
**Status:** Ongoing maintenance documentation  
**Purpose:** Track disk space cleanup activities, regular maintenance tasks, and remaining opportunities

---

## Current System Status

**Disk:** 57GB total, currently 46GB used (85% capacity, 8.2GB free)  
**Platform:** NVIDIA Jetson AGX Orin 64GB  
**OS:** Ubuntu 22.04 Jammy  
**Last Check:** December 19, 2025

---

## What Has Been Completed (December 19, 2025)

### Phase 5: Desktop Application Removal ✅

**Space Recovered:** ~500MB

1. **LibreOffice Suite** - ~310MB recovered ✅
   - Action: `sudo apt remove --purge -y libreoffice*`
   - Status: ✅ Removed - not needed on headless AI system

2. **Thunderbird Email Client** - ~244MB recovered ✅
   - Action: `sudo apt remove --purge -y thunderbird`
   - Status: ✅ Removed - not needed on headless AI system

3. **APT Fix** - Fixed dependency issue
   - Issue: `apt autoremove` tried to remove `libavutil-dev` which is needed by NVIDIA's ffmpeg
   - Fix: `sudo apt --fix-broken install -y`
   - Prevention: `sudo apt-mark manual libavutil-dev libavcodec-dev libavformat-dev libswresample-dev libswscale-dev`
   - Status: ✅ Fixed - ffmpeg dev packages marked as manual to prevent future issues

### Phase 4: Major Cache and Source Cleanup ✅

**Space Recovered:** ~1.8GB (from 91% to 88% disk usage)

1. **Cursor debug.log** - 927MB recovered ✅
   - Location: `~/.cursor/debug.log`
   - Before: 927MB
   - After: 0 bytes
   - Action: Truncated with `truncate -s 0`
   - Status: ✅ Cleaned - debug log regenerates as needed

2. **VSCode Copilot Chat Index** - 601MB recovered ✅
   - Location: `~/.vscode-server/data/User/workspaceStorage/*/GitHub.copilot-chat/`
   - Before: 601MB
   - After: Removed
   - Action: `rm -rf ~/.vscode-server/data/User/workspaceStorage/*/GitHub.copilot-chat/local-index.*.db`
   - Status: ✅ Cleaned - index regenerates automatically

3. **depthai-python source repository** - 259MB recovered ✅
   - Location: `~/depthai-python/`
   - Before: 259MB
   - After: Removed
   - Action: `rm -rf ~/depthai-python`
   - Status: ✅ Removed - depthai is installed in venv, source not needed
   - Note: Confirmed depthai package exists in `~/depthai_env/` and `~/.local/`

4. **Pip cache** - Already clean
   - Location: `~/.cache/pip/`
   - Status: ✅ Already purged (0 files to remove)

### Previously Pending (Now Completed)

- ✅ APT package cache cleaned
- ✅ APT package lists cleaned  
- ✅ Firefox snap rev 7421 removed

---

## What Has Been Completed (December 17, 2025)

### Phase 1: Cache Cleanup (Partially Completed) ✅

**Space Recovered:** ~1.0GB

1. **Pip Cache** - ~270MB recovered
   - Location: `~/.cache/pip/`
   - Before: 478MB
   - After: 208MB
   - Action: Purged with `pip cache purge`
   - Status: ✅ Cleaned - packages re-download on install
   - **Note:** Cache had grown significantly from documented 2.3MB

2. **Archived Backups Removed** - 370MB recovered
   - Location: `~/backups/`
   - Action: Removed after verifying USB archive exists
   - USB Archive: `/media/severin/DAEA-DEBB/r2d2_archive/backups/`
   - Status: ✅ Removed - files safely archived on USB

3. **Epiphany Browser Cache** - 106MB (from earlier cleanup)
   - Location: `~/.cache/epiphany/`
   - Status: ✅ Deleted - regenerates automatically

4. **Torch Hub Cache** - 34MB (from earlier cleanup)
   - Location: `~/.cache/torch/hub/`
   - Status: ✅ Deleted - models re-download when accessed

5. **Old ROS Logs** - ~20MB (from earlier cleanup)
   - Location: `~/.ros/log/`
   - Status: ✅ Cleaned - kept last 7 days

### Phase 3: Remove Piper TTS Models (Completed) ✅

**Space Recovered:** 382MB

- **Removed:** `~/.local/share/piper-tts/` (191MB) - empty directory
- **Removed:** `~/.local/share/piper_tts/` (191MB) - confirmed TTS not used
- **Status:** ✅ Both directories removed - TTS functionality not needed

### Git Repository Optimization (Completed) ✅

**Action:** Ran standard git maintenance
- `git reflog expire --expire=90.days.ago --all`
- `git prune --expire=90.days.ago`
- `git gc --prune=now`

**Result:** Repository optimized, all objects consolidated into pack files
- **Repository size:** 361MB (optimized, no loose objects)

### Previously Removed (Before Documentation)

- **HuggingFace Model Cache** - 2.9GB
  - Location: `~/.cache/huggingface/`
  - Model: faster-whisper-large-v2
  - Status: ✅ Removed - model re-downloads on first use if needed

### Total Space Recovered So Far

**~6GB recovered** (from 91% to 85% disk usage, 5.2GB → 8.2GB free)

---

## Remaining Cleanup Opportunities (Ordered by Impact)

### ✅ Recently Completed (December 19, 2025)

- ~~Cursor debug.log~~ - **927MB** ✅ Truncated
- ~~VSCode Copilot Chat Index~~ - **601MB** ✅ Removed
- ~~depthai-python source repo~~ - **259MB** ✅ Removed

### 🔥 Highest Impact - Safe Cleanup (Execute First)

#### 1. Remove Old GNOME Snap Package - **~1.7GB** ⭐⭐⭐
**Location:** `/snap/gnome-42-2204/` (1.2GB) + `/var/lib/snapd/snaps/gnome-42-2204_228.snap` (494MB)

**Why it's safe:**
- Old version replaced by `gnome-46-2404` (currently active)
- Snap system keeps old revisions for rollback, but newer version is in use
- Removing old revisions is standard practice

**Commands:**
```bash
# Remove old snap revision
sudo snap remove --revision=228 gnome-42-2204

# Verify removal
du -sh /snap/gnome-42-2204 /var/lib/snapd/snaps/gnome-42-2204*
```

**Risk:** ✅ **100% Safe** - Old version, newer version is active  
**Impact:** ~1.7GB  
**Priority:** **HIGHEST**

#### 2. Clean APT Package Cache - **416MB** ⭐⭐
**Location:** `/var/cache/apt/`

**Why it's safe:**
- APT caches downloaded .deb files
- These are cached downloads, not installed packages
- Packages re-download automatically on next install

**Command:**
```bash
sudo apt clean
```

**Risk:** ✅ **100% Safe** - Standard Ubuntu maintenance  
**Impact:** 416MB  
**Priority:** **HIGH**

#### 3. Clean APT Package Lists - **298MB** ⭐⭐
**Location:** `/var/lib/apt/lists/` (298MB, 109 files)

**Why it's safe:**
- APT package repository metadata (package lists, Release files)
- Completely regenerated with `apt update`
- Standard Ubuntu maintenance practice

**Commands:**
```bash
# Check current size
sudo du -sh /var/lib/apt/lists

# Clean package lists (regenerates on apt update)
sudo rm -rf /var/lib/apt/lists/*
sudo apt update  # Regenerates lists (optional, can wait until next update)
```

**Risk:** ✅ **100% Safe** - Standard Ubuntu maintenance  
**Impact:** 298MB  
**Priority:** **HIGH**

#### 4. ~~Clean VSCode Server GitHub Copilot Chat Index~~ - **601MB** ✅ COMPLETED
**Location:** `~/.vscode-server/data/User/workspaceStorage/*/GitHub.copilot-chat/local-index.*.db`

**Status:** ✅ **Completed December 19, 2025** - 601MB recovered

#### 5. Remove Disabled Snap Revisions - **~297MB** ⭐
**Locations:**
- `/var/lib/snapd/snaps/firefox_7421.snap` (235MB) - disabled, newer version active
- `/var/lib/snapd/snaps/core24_1226.snap` (62MB) - disabled, newer version active

**Why it's safe:**
- Disabled revisions, newer versions are active
- Standard snap maintenance practice

**Commands:**
```bash
# Remove disabled firefox revision
sudo snap remove --revision=7421 firefox

# Remove disabled core24 revision  
sudo snap remove --revision=1226 core24

# Verify
snap list --all | grep disabled
```

**Risk:** ✅ **100% Safe** - Disabled revisions, newer versions active  
**Impact:** ~297MB  
**Priority:** **MEDIUM-HIGH**

#### 6. Rotate System Logs - **~196MB** ⭐
**Location:** `/var/log/` (263MB total)

**Why it's safe:**
- System logs grow over time
- Rotation keeps recent logs, archives old ones
- Standard Ubuntu maintenance

**Commands:**
```bash
# Rotate logs (recommended)
sudo logrotate -f /etc/logrotate.conf

# Or truncate if rotation doesn't help
sudo truncate -s 0 /var/log/syslog
sudo truncate -s 0 /var/log/kern.log

# Clean old journal logs
sudo journalctl --vacuum-time=30d  # Keep last 30 days
sudo journalctl --vacuum-size=100M  # Or limit to 100MB
```

**Risk:** ✅ **Safe** - Logs regenerate, old logs archived  
**Impact:** ~196MB  
**Priority:** **MEDIUM**

#### 7. Clean Old ROS Logs - **~50-60MB** ⭐
**Location:** `~/.ros/log/` (currently 75MB)

**Why it's safe:**
- ROS logs accumulate over time
- Keep last 7 days for debugging

**Command:**
```bash
find ~/.ros/log -name "*.log" -mtime +7 -delete
```

**Risk:** ✅ **Safe** - Keeps last 7 days  
**Impact:** ~50-60MB  
**Priority:** **MEDIUM**

#### 8. Remove Old Crash Reports - **14MB** ⭐
**Location:** `/var/crash/` (14MB)

**What it is:**
- Application crash dumps from December 12-13, 2025
- Files: ROS2 crashes and Python crash
- Used for debugging but not needed if issues are resolved

**Commands:**
```bash
# List crash files
ls -lh /var/crash/

# Remove old crash reports
sudo rm -f /var/crash/*.crash
```

**Risk:** ✅ **Safe** - Old crash dumps, not needed unless debugging  
**Impact:** 14MB  
**Priority:** **MEDIUM**

#### 9. Vacuum System Journal Logs - **~20-30MB** ⭐
**Location:** System journal (currently 40MB)

**Why it's safe:**
- systemd journal logs (system and application logs)
- Can be reduced to keep only recent logs
- Standard Ubuntu maintenance

**Commands:**
```bash
# Check current journal size
journalctl --disk-usage

# Vacuum to keep last 30 days (or 100MB)
sudo journalctl --vacuum-time=30d
# OR
sudo journalctl --vacuum-size=100M
```

**Risk:** ✅ **Safe** - Standard log rotation  
**Impact:** ~20-30MB (reducing from 40MB to 10-20MB)  
**Priority:** **MEDIUM**

#### 10. Clean Old VSCode Server Logs - **~3MB** ⭐
**Location:** `~/.vscode-server/data/logs/`

**Why it's safe:**
- Old logs from December 9-11, 2025
- Not needed for current debugging

**Command:**
```bash
# Remove logs older than 7 days
find ~/.vscode-server/data/logs -type d -mtime +7 -exec rm -rf {} + 2>/dev/null
```

**Risk:** ✅ **Safe** - Old logs, not needed for debugging  
**Impact:** ~3MB  
**Priority:** **LOW**

#### 11. Purge Removed Package Configs - **~1-5MB** ⭐
**Location:** System-wide (6 packages with "rc" status)

**What it is:**
- Packages marked as "rc" (removed but config files remain)
- Config files left behind after package removal
- Packages: dctrl-tools, grub-common, libsane-hpaio, oem-config, oem-config-gtk, ssl-cert

**Commands:**
```bash
# List removed packages with configs
dpkg -l | grep "^rc"

# Purge all removed package configs
sudo apt-get purge -y $(dpkg -l | grep "^rc" | awk '{print $2}')
```

**Risk:** ✅ **Safe** - Already removed packages, just cleaning configs  
**Impact:** ~1-5MB (minimal, but good housekeeping)  
**Priority:** **LOW**

### ⚠️ Medium Priority (Verify Before Removal)

#### 12. OTA Update Packages - **343MB** ⚠️
**Location:** `/opt/ota_package/`

**Note:** These may be needed for future Jetson OTA updates. Verify if system updates are complete before removing.

**Commands:**
```bash
# Check what's in there
ls -lh /opt/ota_package/

# If no longer needed (verify first!)
sudo rm -rf /opt/ota_package/*
```

**Risk:** ⚠️ **Moderate** - May be needed for system updates  
**Impact:** 343MB  
**Priority:** **MEDIUM** (after verification)

### 📦 Archive to USB (Then Remove from Disk)

#### 13. Archive Documentation Photos - **15MB**
**Locations:**
- `~/dev/r2d2/docs/photos/` (15MB)
- `~/dev/r2d2/_ANALYSIS_AND_DOCUMENTATION/` (424KB)

**Action:** Archive to USB, then optionally remove from disk if no longer needed for reference

**Commands:**
```bash
USB_ARCHIVE=/media/severin/DAEA-DEBB/r2d2_archive
mkdir -p $USB_ARCHIVE/docs
cp -r ~/dev/r2d2/docs/photos $USB_ARCHIVE/docs/
cp -r ~/dev/r2d2/_ANALYSIS_AND_DOCUMENTATION $USB_ARCHIVE/analysis/

# Then remove from disk (if archived)
rm -rf ~/dev/r2d2/docs/photos/*
```

**Risk:** ✅ **Safe** - Documentation can be archived  
**Impact:** 15MB  
**Priority:** **LOW**

#### 14. Clean Old Debug Images - **~1MB**
**Locations:**
- `~/dev/r2d2/tests/camera/*.jpg`
- `~/dev/r2d2/debug_frame.jpg`
- `~/dev/r2d2/oak_d_photo.jpg`

**Action:** Archive or remove old test/debug images

**Commands:**
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

**Risk:** ✅ **Safe** - Test images can be regenerated  
**Impact:** ~1MB  
**Priority:** **LOW**

#### 15. Remove Duplicate Audio Files - **~36KB**
**Location:** `~/Voicy_R2-D2 - {2,3,5}.mp3`

**Note:** These already exist in project at `~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/`

**Command:**
```bash
rm ~/Voicy_R2-D2*.mp3
```

**Risk:** ✅ **Safe** - Duplicates exist in project  
**Impact:** 36KB  
**Priority:** **LOW**

---

## Cleanup Summary by Impact

### Immediate Safe Cleanup (Highest Priority)
1. Remove old GNOME snap: **~1.7GB** ⭐⭐⭐
2. Clean APT package cache: **416MB** ⭐⭐
3. Clean APT package lists: **298MB** ⭐⭐
4. Clean VSCode Copilot index: **601MB** ⭐⭐
5. Remove disabled snap revisions: **~297MB** ⭐
6. Rotate system logs: **~196MB** ⭐
7. Clean old ROS logs: **~50-60MB** ⭐
8. Remove crash reports: **14MB** ⭐
9. Vacuum journal logs: **~20-30MB** ⭐
10. Clean VSCode logs: **~3MB** ⭐
11. Purge removed configs: **~1-5MB** ⭐

**Total Immediate Safe Cleanup: ~3.7GB**

### After Verification
- Remove OTA packages (if not needed): **343MB**

### Archive to USB (Then Remove)
- Documentation photos: **15MB**
- Test/debug images: **1MB**
- Duplicate audio files: **36KB**

### Grand Total Potential
**~4.0GB additional space recovery possible**

---

## System Directories Analysis

### Required Directories (DO NOT DELETE)

- **`/opt/nvidia/`** - 2.4GB - NVIDIA drivers (required for Jetson)
- **`/opt/ros/`** - 283MB - ROS installation (required)
- **`/snap/`** - 5.9GB - Snap packages (required, normal size)
- **`/usr/local/cuda-12.6/`** - 4.4GB - CUDA installation (required for Jetson)
- **`~/.local/share/`** - Virtual environments and model files (required)
- **`~/dev/r2d2/r2d2_speech_env/`** - 1.3GB - Python virtual environment (required)
- **`~/depthai_env/`** - 1.5GB - Python virtual environment (required)
- **`~/dev/r2d2/.git/`** - 361MB - Git repository (required)
- **`~/.vscode-server/`** - 1.7GB - VSCode Server (required for remote development)
- **`~/.cursor-server/`** - 433MB - Cursor Server (required for IDE)

### Cache Directories (Can Clean Regularly)

- **`/var/cache/apt/`** - 416MB - APT package cache (safe to clean)
- **`/var/lib/apt/lists/`** - 298MB - APT package lists (safe to clean)
- **`~/.cache/pip/`** - 208MB - Python pip cache (safe to clean, was 478MB)
- **`~/.cache/torch/`** - Variable - PyTorch cache (safe to clean)
- **`~/.cache/epiphany/`** - Variable - Browser cache (safe to clean)
- **`~/.cache/huggingface/`** - Variable - Model cache (safe to clean, re-downloads)

### Log Directories (Can Rotate/Clean)

- **`/var/log/`** - 263MB - System logs (can rotate)
- **`~/.ros/log/`** - 75MB - ROS logs (can clean old)
- **Journal logs** - 40MB - systemd journal (can vacuum)
- **`/var/crash/`** - 14MB - Crash reports (can remove old)

### Snap Directories (Can Clean Old Revisions)

- **`/snap/gnome-42-2204/`** - 1.2GB - Old GNOME snap (can remove, newer version active)
- **`/var/lib/snapd/snaps/gnome-42-2204_228.snap`** - 494MB - Old revision
- **`/var/lib/snapd/snaps/firefox_7421.snap`** - 235MB - Disabled revision
- **`/var/lib/snapd/snaps/core24_1226.snap`** - 62MB - Disabled revision

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

#### 2. Clean APT Package Lists (Recommended: Monthly)
**Space Potential:** ~250-300MB

```bash
# Clean package lists
sudo rm -rf /var/lib/apt/lists/*
sudo apt update  # Regenerates lists
```

**Why:** Package lists are regenerated on `apt update`. Safe to clean.

#### 3. Rotate System Logs (Recommended: Monthly)
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

#### 4. Clean User Caches (Recommended: Monthly)
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

#### 5. Clean Old Snap Revisions (Recommended: Quarterly)
**Space Potential:** Variable (can be 1GB+)

```bash
# List all snap revisions
snap list --all

# Remove old revisions (keep only active)
sudo snap remove --revision=<OLD_REV> <SNAP_NAME>

# Set system to keep fewer revisions
sudo snap set system refresh.retain=2  # Keep only 2 revisions
```

**Why:** Snap keeps old revisions for rollback. Remove if not needed.

#### 6. Git Repository Maintenance (Recommended: Quarterly)
**Space Potential:** Variable (optimization, not necessarily space recovery)

```bash
cd ~/dev/r2d2
git reflog expire --expire=90.days.ago --all
git prune --expire=90.days.ago
git gc --prune=now
```

**Why:** Keeps repository optimized, consolidates objects.

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

# Clean APT package lists
echo "Cleaning APT package lists..."
sudo rm -rf /var/lib/apt/lists/*
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

# Vacuum journal logs
echo "Vacuuming journal logs..."
sudo journalctl --vacuum-time=30d
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
- Old snap revisions are kept for rollback capability, but removing them is safe if you don't need rollback
- VSCode Server cache can be safely removed - extensions and settings remain intact
- APT lists and cache are regenerated automatically on next update/install

---

## Additional Cleanup Opportunities Discovered (December 19, 2025)

### Tier 2: Safe for Headless/Robot System

#### Docker (Unused) - **~370MB**
- Docker is installed but `/var/lib/docker` is only 4KB (empty)
- Packages: docker-ce, docker-ce-cli, containerd.io, docker-buildx-plugin, docker-compose-plugin
- **Command:** `sudo apt remove docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin -y`
- **Risk:** ✅ Safe if not using Docker

#### fonts-noto-cjk - **91MB**
- CJK (Chinese/Japanese/Korean) fonts - not needed for robot
- **Command:** `sudo apt remove fonts-noto-cjk -y`
- **Risk:** ✅ Safe - only affects CJK text rendering

#### Java OpenJDK 11 - **~250MB**
- Located at `/usr/lib/jvm` (244MB)
- No runtime dependencies found
- **Command:** `sudo apt remove openjdk-11-jdk-headless openjdk-11-jre-headless -y`
- **Risk:** ⚠️ Verify no apps depend on Java first

#### Desktop Apps (Unnecessary on Robot) - OPTIONAL
- ~~**Thunderbird:** ~244MB~~ ✅ REMOVED
- ~~**LibreOffice:** ~310MB~~ ✅ REMOVED
- **Games (aisleriot, gnome-mahjongg, gnome-mines, gnome-sudoku):** ~15MB - `sudo apt remove aisleriot gnome-mahjongg gnome-mines gnome-sudoku -y`
- **Media (shotwell, rhythmbox, totem, cheese, eog):** ~15MB - `sudo apt remove shotwell rhythmbox totem cheese eog -y`
- **GNOME utilities (gnome-calculator, gnome-calendar, gnome-characters, gnome-font-viewer, gnome-logs, gnome-screenshot, seahorse, baobab):** ~10MB
- **Other (remmina, transmission-gtk, deja-dup, gedit, evince, simple-scan, file-roller, yelp, gnome-user-docs):** ~20MB
- **Firefox snap:** ~235MB - `sudo snap remove firefox`

### Tier 3: Development Tools (If Not Needed)

#### Nsight Tools - **~2.2GB**
- Nsight Compute: 919MB (`/opt/nvidia/nsight-compute`)
- Nsight Systems: 818MB (`/opt/nvidia/nsight-systems`)
- Nsight Graphics: 492MB (`/opt/nvidia/nsight-graphics-for-embeddedlinux`)
- **Note:** Only remove if not actively profiling/debugging GPU applications

#### Development Headers - **~300MB**
- `/usr/include/boost`: 162MB
- `libboost1.74-dev`: 138MB
- **Note:** Only remove if not compiling C++ code

### Summary of All Potential Savings

| Category | Completed | Optional (Remaining) |
|----------|-----------|----------------------|
| Caches/Logs | ~2.7GB ✅ | - |
| Desktop Apps | ~0.5GB ✅ | ~0.3GB |
| Docker | - | ~0.4GB |
| Dev Tools | - | ~2.5GB |
| Snaps | ~0.3GB ✅ | ~1.7GB |

**Total recovered:** ~6GB (from 91% to 85%)  
**Total potential additional savings:** ~5GB (optional)

---

## Last Updated

December 19, 2025 - Phase 4 & 5 cleanup completed (~6GB recovered total). System now at 85% disk usage (8.2GB free). LibreOffice and Thunderbird removed. Additional optional cleanup opportunities documented. Backup created at `/media/severin/DAEA-DEBB/temp_backup/`.
