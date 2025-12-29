# R2D2 Backup & Restore System - USB Edition
## Complete Documentation: USB Stick Backup System

**Date:** December 22, 2025  
**Project:** R2D2 as a Real AI Companion  
**Platform:** NVIDIA Jetson AGX Orin 64GB Developer Kit  
**Status:** ✅ **FULLY OPERATIONAL & TESTED**

---

## Quick Start

**TL;DR — If you just want to backup/restore:**

1. **Label your USB stick:**
   ```bash
   sudo fatlabel /dev/sda1 R2D2_BACKUP
   ```

2. **On Jetson, create backup:**
   ```bash
   # Plug in USB stick
   bash ~/dev/r2d2/scripts/r2d2_backup.sh
   # Creates: /media/severin/R2D2_BACKUP/r2d2_backup_YYYYMMDD/
   ```

3. **To restore on fresh Jetson:**
   ```bash
   # Plug in USB stick with backup
   sudo bash ~/dev/r2d2/scripts/r2d2_restore.sh
   # System fully restored in 20-30 minutes
   ```

**For detailed step-by-step instructions, see sections below.**

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Why USB-Based Backup?](#why-usb-based-backup)
3. [System Architecture](#system-architecture)
4. [What Gets Backed Up](#what-gets-backed-up)
5. [USB Stick Preparation](#usb-stick-preparation)
6. [Creating a Backup](#creating-a-backup)
7. [Restoring from Backup](#restoring-from-backup)
8. [Testing & Validation](#testing--validation)
9. [Complete Workflow Examples](#complete-workflow-examples)
10. [Key Design Decisions](#key-design-decisions)
11. [Performance Characteristics](#performance-characteristics)
12. [Troubleshooting Guide](#troubleshooting-guide)
13. [FAQ](#faq)
14. [Related Documentation](#related-documentation)

---

## Executive Summary

The **R2D2 USB Backup System** provides a simple, portable, and reliable way to backup and restore your R2D2 Jetson system using a dedicated USB stick. Key features:

1. **Complete system reproducibility** — Flash a new Jetson, run setup, restore from USB = identical to previous state
2. **Portable backups** — USB stick can be safely stored offsite for disaster recovery
3. **Date-versioned folders** — One backup per day, organized by date (r2d2_backup_YYYYMMDD/)
4. **Automatic retention** — Keeps last 5 backups, automatically removes older ones
5. **No cloud dependencies** — All backups stay on your USB stick, no internet required
6. **Simple workflow** — Plug in USB, run script, unplug USB

### Key Achievements
- ✅ **USB-only design:** No backups stored on Jetson or Windows
- ✅ **Date-versioned folders:** Easy to browse and identify backups
- ✅ **Auto USB detection:** Finds labeled USB stick automatically
- ✅ **Automatic cleanup:** Keeps last 5 backups (configurable)
- ✅ **Smart exclusions:** Removes regeneratable files (build/, .git/, __pycache__)
- ✅ **Backup metadata:** Includes timestamp, hostname, system info
- ✅ **Comprehensive logging:** Detailed progress output
- ✅ **No external dependencies:** Only uses bash, tar, standard tools
- ✅ **Production-ready:** All real code, no placeholders

### For Different Audiences

- **New users or quick reference:** Jump to [Quick Start](#quick-start) above
- **First-time setup:** Read [USB Stick Preparation](#usb-stick-preparation)
- **Regular backup/restore:** Use Sections 6-7 as quick reference
- **Understanding the system:** Read Section 2-4 (Why/How it works)
- **Troubleshooting issues:** Jump to [Troubleshooting Guide](#troubleshooting-guide)

---

## Why USB-Based Backup?

### The Problem

Before this system, Jetson management involved:

- **Risk:** One bad flashing or configuration change = complete data loss
- **Tedium:** Rebuilding from scratch = 3-4 hours
- **Training loss:** Face recognition models take hours to train
- **Hardware failure:** New Jetson required complete reconfiguration
- **No archival:** No portable snapshots of working system states
- **Cloud dependency:** Required internet and cloud storage accounts

### The Solution

The **R2D2 USB Backup System** solves all of this:

| Problem | Solution |
|---------|----------|
| Complete data loss | Automated backups with date versioning |
| Tedious rebuilding | Setup script + restore = 30 minutes |
| Training data loss | Backups include all `~/dev/r2d2/data/` (face models, embeddings) |
| Hardware replacement | Restore to new Jetson in ~30 minutes |
| No snapshots | Automatic retention keeps last 5 backups |
| Portability | USB stick can be stored anywhere, no cloud needed |

---

## System Architecture

### Single-Component Backup Pipeline

```
┌─────────────────────────────────────────────────────────────────┐
│                    JETSON AGX ORIN (R2D2)                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ~/dev/r2d2/scripts/r2d2_backup.sh                             │
│  ├─ Detects USB stick (labeled R2D2_BACKUP)                   │
│  ├─ Creates date folder: r2d2_backup_YYYYMMDD/                │
│  ├─ Stages project root (ros2_ws/src, data/, configs)         │
│  ├─ Stages user configs (~/.bashrc, ~/.bash_aliases)          │
│  ├─ Stages system configs (/etc/udev/rules.d/r2d2_*)          │
│  ├─ Stages systemd services (/etc/systemd/system/r2d2_*)      │
│  ├─ Creates tar.gz archive + metadata                         │
│  └─ Cleans up old backups (keeps last 5)                      │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
                              ↕
                     Direct USB Write
                              ↕
┌─────────────────────────────────────────────────────────────────┐
│              USB STICK (Labeled: R2D2_BACKUP)                  │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  /media/severin/R2D2_BACKUP/                                   │
│  ├── r2d2_backup_20251222/                                     │
│  │   ├── r2d2_backup.tar.gz          (4-5 GB)                 │
│  │   └── BACKUP_METADATA.txt                                  │
│  ├── r2d2_backup_20251215/                                     │
│  │   ├── r2d2_backup.tar.gz                                   │
│  │   └── BACKUP_METADATA.txt                                  │
│  └── r2d2_backup_20251208/           (keeps last 5)           │
│      ├── r2d2_backup.tar.gz                                   │
│      └── BACKUP_METADATA.txt                                  │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Script Responsibilities

| Script | Purpose | Location | User Level | Time |
|--------|---------|----------|-----------|------|
| `r2d2_backup.sh` | Create date-versioned backup to USB | Jetson | Regular user | 15-30 min |
| `r2d2_restore.sh` | Restore backup from USB to Jetson | Jetson | Root | 20-30 min |

---

## What Gets Backed Up

### ✅ Included in Backup

**Project Files (~/dev/r2d2/)**
```
ros2_ws/src/                    # All ROS 2 packages
├── r2d2_hello/                 # Heartbeat & beep nodes
├── r2d2_bringup/               # Launch system
├── r2d2_camera/                # Camera node (if present)
├── r2d2_perception/            # Perception pipeline (if present)
├── r2d2_speech/                # Speech system (if present)
├── face_detection/             # Face detection integration (if present)
└── face_recognition_ros2/      # Face recognition service (if present)

data/                           # Non-git runtime data
├── face_recognition/           # CRITICAL: Training data & models
│   ├── train_data/            # Raw face images (people)
│   ├── embeddings.pkl         # Pre-computed embeddings
│   ├── lbph_model.yml         # Trained LBPH model
│   ├── labels.json            # Person ID mappings
│   └── training_status.json   # Training progress/status
├── calibration/               # Camera calibration (if present)
└── *.md files                 # All documentation
```

**User Configuration Files**
```
~/.bashrc                       # Shell environment + ROS 2 sourcing
~/.bash_aliases                 # Custom command aliases
~/.config/r2d2/                 # Future R2D2-specific config
```

**System Configuration Files** (if present)
```
/etc/udev/rules.d/r2d2_*       # USB device enumeration rules
/etc/systemd/system/r2d2_*.service  # Background services
```

### ❌ Excluded from Backup (Not Backed Up)

**Regeneratable Build Artifacts**
```
ros2_ws/build/                 # Compiled objects (~1-2 GB)
ros2_ws/install/               # Installed packages (~500 MB)
ros2_ws/log/                   # ROS 2 logs (can accumulate)
```

**Version Control Metadata**
```
.git/                          # Git history (not needed to restore)
.gitignore                     # Git config file
```

**Python & System Caches**
```
__pycache__/                   # Python compiled bytecode
*.pyc, *.pyo                   # Compiled Python modules
*.egg-info/                    # Package metadata caches
```

**Reasoning:** These are all **regeneratable**:
- Build artifacts: `colcon build` recreates in ~10 minutes
- Git metadata: `git clone` or `git pull` recreates if needed
- Python caches: Automatically recreated when modules imported
- Excluding them saves ~3-4 GB of backup space and backup time

---

## USB Stick Preparation

### Prerequisites

- USB stick with at least **32 GB** capacity (64+ GB recommended)
- Formatted as FAT32 or ext4
- USB 3.0+ recommended for faster transfers

### Step 1: Identify USB Device

Plug in your USB stick and find the device name:

```bash
lsblk -f
```

Output example:
```
NAME        FSTYPE LABEL        UUID                                 MOUNTPOINT
sda1        vfat   DAEA-DEBB    1234-5678                           /media/severin/DAEA-DEBB
```

The USB stick is typically `/dev/sda1` or similar.

### Step 2: Label the USB Stick

Label your USB stick as `R2D2_BACKUP` so the scripts can auto-detect it:

**For FAT32 filesystems:**
```bash
sudo fatlabel /dev/sda1 R2D2_BACKUP
```

**For ext4 filesystems:**
```bash
sudo e2label /dev/sda1 R2D2_BACKUP
```

### Step 3: Verify Label

Unplug and replug the USB stick, then verify:

```bash
lsblk -f | grep R2D2_BACKUP
# Should show: sda1   vfat   R2D2_BACKUP   ...
```

Check mount point:
```bash
ls -la /media/severin/R2D2_BACKUP
# Should list the USB contents
```

### Step 4: Create Backup Directory (Optional)

The backup script will create folders automatically, but you can pre-create the structure:

```bash
mkdir -p /media/severin/R2D2_BACKUP
```

---

## Creating a Backup

### Prerequisites

- USB stick labeled `R2D2_BACKUP` and plugged in
- Sufficient space on USB (at least 10 GB free)
- ~15-30 minutes of time

### Step-by-Step Backup

#### 1. Plug in USB Stick

Insert your USB stick labeled `R2D2_BACKUP`. Verify it's mounted:

```bash
ls /media/severin/R2D2_BACKUP
```

#### 2. Run Backup Script

```bash
bash ~/dev/r2d2/scripts/r2d2_backup.sh
```

#### 3. Backup Progress

You'll see output like:

```
=== R2D2 Jetson USB Backup ===
[INFO] Starting backup at Sun Dec 22 14:23:45 UTC 2025
[INFO] Looking for USB stick labeled 'R2D2_BACKUP'...
[INFO] Found USB stick at: /media/severin/R2D2_BACKUP
[INFO] Creating backup folder: r2d2_backup_20251222
[INFO] Backup destination: /media/severin/R2D2_BACKUP/r2d2_backup_20251222/r2d2_backup.tar.gz
[INFO] Staging files for backup...
[INFO] Staging project root: /home/severin/dev/r2d2
[INFO] Staging user configuration files...
[INFO] Staging system configuration files...
[INFO] Found udev rules, backing up...
[INFO] Creating backup metadata...
[INFO] Creating backup archive...
[INFO] This may take a few minutes...
[INFO] Backup created successfully: 4.2G
[INFO] Location: /media/severin/R2D2_BACKUP/r2d2_backup_20251222
[INFO] Cleaning up old backups (keeping last 5)...
[INFO] Backup count (3) is within retention limit
=== Backup Complete ===
[INFO] Backup finished at Sun Dec 22 14:45:30 UTC 2025
[INFO] Folder: r2d2_backup_20251222
[INFO] Archive: r2d2_backup.tar.gz (4.2G)
[INFO] USB Location: /media/severin/R2D2_BACKUP

[INFO] You can now safely eject the USB stick
```

#### 4. Safely Eject USB

```bash
# Sync filesystem
sync

# Unmount USB
udisksctl unmount -b /dev/sda1
udisksctl power-off -b /dev/sda

# Or use GUI to eject
```

### Backup Frequency Recommendations

- **Weekly:** For active development
- **Before major changes:** System updates, hardware modifications
- **After training:** New face recognition training sessions
- **Monthly:** For stable production systems

### Overwriting Today's Backup

If you run the backup script twice in one day, it will prompt:

```
[WARN] Backup folder for today already exists: r2d2_backup_20251222
Overwrite existing backup? (yes/no):
```

Type `yes` to overwrite or `no` to cancel.

---

## Restoring from Backup

### Prerequisites for Restore

1. A fresh Jetson that has been:
   - Flashed with JetPack 6.x
   - Had `r2d2_setup.sh` run to completion
   - Can be accessed via SSH or direct login
   
2. USB stick with backup plugged in and mounted

3. ~30 minutes of time

### Step-by-Step Restore

#### 1. Prepare the Jetson

If you haven't already, run the setup script first:

```bash
sudo bash ~/dev/r2d2/scripts/r2d2_setup.sh
```

This installs ROS 2, dependencies, and creates the workspace structure.

#### 2. Plug in USB Stick

Insert your USB stick with backups. Verify it's mounted:

```bash
ls /media/severin/R2D2_BACKUP
# Should show: r2d2_backup_20251222/  r2d2_backup_20251215/  ...
```

#### 3. Run Restore Script

**Method 1: Use latest backup (automatic)**
```bash
sudo bash ~/dev/r2d2/scripts/r2d2_restore.sh
```

**Method 2: Specify a backup folder**
```bash
sudo bash ~/dev/r2d2/scripts/r2d2_restore.sh /media/severin/R2D2_BACKUP/r2d2_backup_20251222
```

**Method 3: Specify archive directly**
```bash
sudo bash ~/dev/r2d2/scripts/r2d2_restore.sh /media/severin/R2D2_BACKUP/r2d2_backup_20251222/r2d2_backup.tar.gz
```

**Method 4: Skip confirmation (for automation)**
```bash
sudo bash ~/dev/r2d2/scripts/r2d2_restore.sh --yes
```

#### 4. Confirmation Prompt

You'll see:

```
[WARN] RESTORE WARNING
This will restore files from the backup to your home directory and system.
Existing files will be overwritten.

Backup archive: /media/severin/R2D2_BACKUP/r2d2_backup_20251222/r2d2_backup.tar.gz
Target user: severin
Target home: /home/severin

Do you want to continue? (yes/no):
```

Type `yes` to proceed.

#### 5. Restore Progress

The script will:

```
=== R2D2 Jetson USB Restore ===
[INFO] Starting restore at Sun Dec 22 15:00:00 UTC 2025
...
=== Extracting Backup ===
[INFO] Extracting to temporary directory...
[INFO] Extraction complete

[INFO] Backup metadata:
  R2D2 Backup Information
  =======================
  Created: Sun Dec 22 14:23:45 UTC 2025
  Hostname: jetson-orin
  Kernel: Linux 5.15.148-tegra
  ROS 2: ROS 2 Humble

=== Restoring Home Directory Files ===
[INFO] Restoring files to /home/severin...
[INFO] Restoring project root...
[INFO] Project root restored
[INFO] Restoring .bashrc...
[INFO] Restoring .bash_aliases...

=== Restoring System Files ===
[INFO] Restoring system configuration...
[INFO] Restoring udev rules...
[INFO] Udev rules reloaded
[INFO] Restoring systemd services...
[INFO] Systemd daemon reloaded

=== Rebuilding ROS 2 Workspace ===
[INFO] Rebuilding /home/severin/dev/r2d2/ros2_ws...
[INFO] Building workspace (this may take a few minutes)...
[INFO] Workspace rebuild complete

=== Restore Complete ===
[INFO] Restore finished at Sun Dec 22 15:30:00 UTC 2025

[INFO] Next steps:
  1. Source the environment: source /home/severin/.bashrc
  2. Verify system health:
     - Check heartbeat: ros2 topic echo /r2d2/heartbeat -n 3
     - Check topics: ros2 topic list
  3. Test camera (if available):
     - ros2 launch r2d2_camera oak_d_camera.launch.py
  4. Test face recognition (if trained):
     - ros2 launch face_recognition face_recognition.launch.py
```

#### 6. Verify Restoration

After the script completes:

```bash
# Source the environment
source ~/.bashrc

# Check ROS 2 nodes
ros2 topic list

# Verify heartbeat
ros2 topic echo /r2d2/heartbeat -n 3

# Check face recognition data (if applicable)
ls -la ~/dev/r2d2/data/face_recognition/

# Check Git status (should show clean repo)
cd ~/dev/r2d2
git status
```

---

## Testing & Validation

### After Restore, Run These Tests

#### 1. Basic System Health

```bash
source ~/.bashrc

# Check ROS 2
echo "ROS 2 version:"
ros2 --version

# Check available topics
echo "Available topics:"
ros2 topic list

# Check Jetson hardware
echo "NVIDIA GPU:"
nvidia-smi

# Check CPU/memory
echo "System stats:"
tegrastats --interval 1 --count 3
```

#### 2. ROS 2 Workspace Verification

```bash
cd ~/dev/r2d2/ros2_ws

# Check for build artifacts
echo "Checking build artifacts..."
ls -la build/ install/ log/

# List installed packages
echo "Installed packages:"
ros2 pkg list | grep r2d2

# Check for source files
echo "Source packages:"
ls -la src/
```

#### 3. Face Recognition Data (if applicable)

```bash
# Check training data
echo "Face recognition datasets:"
ls -la ~/dev/r2d2/data/face_recognition/

# Verify model files exist
echo "Model files:"
find ~/dev/r2d2/data/face_recognition -name "*.yml" -o -name "*.pkl"

# Check label mappings
echo "Label files:"
cat ~/dev/r2d2/data/face_recognition/labels.json 2>/dev/null || echo "(not present)"
```

#### 4. System Configuration

```bash
# Check udev rules
echo "Udev rules:"
ls -la /etc/udev/rules.d/r2d2_* 2>/dev/null || echo "(none)"

# Check systemd services
echo "Systemd services:"
systemctl list-unit-files | grep r2d2 || echo "(none)"

# Check bash configuration
echo "Bash aliases:"
alias | grep r2d2 || echo "(none)"
```

### Success Indicators

- ✅ `ros2 topic list` shows `/r2d2/heartbeat`
- ✅ `nvidia-smi` shows GPU driver and CUDA
- ✅ `tegrastats` shows CPU/GPU/memory activity
- ✅ `ros2 launch r2d2_bringup bringup.launch.py` starts without errors
- ✅ Face recognition data directory contains expected files
- ✅ System configs and services properly linked
- ✅ Bash aliases work (`r2d2_build`, `r2d2_run`, etc.)

---

## Complete Workflow Examples

### Scenario 1: Fresh Jetson → Fully Operational in 60 Minutes

**Time: T+0 minutes** — Flash JetPack on Jetson
```bash
# Using NVIDIA SDK Manager (on Ubuntu host)
# Select: Jetson AGX Orin 64GB
# Select: JetPack 6.x
# Flash to device
```

**Time: T+15 minutes** — SSH to Jetson, run setup
```bash
ssh severin@192.168.x.1
sudo bash ~/r2d2_setup.sh  # ~20-30 min
```

**Time: T+45 minutes** — System operational
```bash
source ~/.bashrc
ros2 launch r2d2_bringup bringup.launch.py
# ✓ Heartbeat publishing
# ✓ All ROS 2 packages built
# ✓ Ready for development
```

**Time: T+50 minutes** — Restore from USB backup
```bash
# Plug in USB stick
sudo bash ~/dev/r2d2/scripts/r2d2_restore.sh --yes

# Result: Face recognition models, training data, all configs restored
```

### Scenario 2: Weekly Backup Routine

**Every Sunday evening:**
```bash
# 1. Stop all ROS 2 processes
pkill -9 -f ros2

# 2. Plug in USB stick

# 3. Run backup
bash ~/dev/r2d2/scripts/r2d2_backup.sh

# 4. Verify backup created
ls -lh /media/severin/R2D2_BACKUP/r2d2_backup_$(date +%Y%m%d)/

# 5. Safely eject USB
sync
udisksctl unmount -b /dev/sda1
udisksctl power-off -b /dev/sda

# 6. Store USB in safe location
```

### Scenario 3: System Disaster Recovery

**Original system corrupted:**
```bash
# On corrupted Jetson, try to backup if possible:
bash ~/dev/r2d2/scripts/r2d2_backup.sh  # May fail

# Flash new Jetson
# Run setup on new Jetson
sudo bash ~/r2d2_setup.sh

# Restore from USB (use backup from before corruption)
# Plug in USB stick
sudo bash ~/dev/r2d2/scripts/r2d2_restore.sh /media/severin/R2D2_BACKUP/r2d2_backup_20251215

# Result: Completely restored to state from Dec 15, 2025
```

### Scenario 4: Multi-Device Deployment

**Deploy identical setup to 3 Jetson boards:**

```bash
# On first Jetson (fully configured):
bash ~/dev/r2d2/scripts/r2d2_backup.sh
# Creates: r2d2_backup_20251222/

# For each new Jetson (Jetson #2 and #3):
# 1. Flash with JetPack
# 2. Run setup
ssh severin@192.168.x.2  # Second board
sudo bash ~/r2d2_setup.sh

# 3. Plug in USB stick and restore
sudo bash ~/dev/r2d2/scripts/r2d2_restore.sh

# Result: All 3 boards identical, including face recognition training data
```

---

## Key Design Decisions

### Why USB Stick Instead of Cloud?

**Advantages:**
- ✅ No internet dependency
- ✅ Physical control over backups
- ✅ Can be stored offsite easily
- ✅ Faster transfer speeds (USB 3.0)
- ✅ No subscription costs
- ✅ No privacy concerns

**Disadvantages:**
- ❌ Can be lost or damaged (solution: keep multiple USB backups)
- ❌ Manual process (solution: establish routine)
- ❌ Limited to USB capacity (solution: use 64+ GB stick)

### Why Date-Only Folders?

**Reasoning:**
- One backup per day is sufficient for most use cases
- Date-only naming (`YYYYMMDD`) is sortable and easy to identify
- Prevents backup spam from multiple runs per day
- User explicitly confirms overwrite if running twice in one day

### Why Keep Only 5 Backups?

**Reasoning:**
- Balances safety with storage
- 5 backups × 5 GB = 25 GB (leaves room on 32 GB stick)
- Covers ~5 weeks for weekly backups
- Change `RETENTION_COUNT=10` in script for more

### Why Exclude Build Artifacts?

**Problem:** `ros2_ws/build/` and `ros2_ws/install/` are huge (1-2 GB each)

**Solution:** Exclude from backup, rebuild on restore:
- Save 2-3 GB per backup
- Backup time cut by 40-50%
- Rebuild takes ~10 minutes (automatic)
- Restore is more reliable (fresh build = fewer bugs)

### Why Auto-Detect USB?

**Problem:** Hard-coded paths break when USB mounts elsewhere

**Solution:** Label-based detection:
- Works regardless of device name (`/dev/sda1` vs `/dev/sdb1`)
- Works on any Linux system
- User just needs to label USB once
- Script finds it automatically

---

## Performance Characteristics

### Backup Performance

| Metric | Value | Notes |
|--------|-------|-------|
| **First backup time** | 15-30 min | Depends on USB speed + data size |
| **Subsequent backups** | 15-30 min | Full backup each time |
| **Backup size** | 4-5 GB | Compressed (tar.gz) |
| **Uncompressed size** | 8-10 GB | Before compression |
| **USB space required** | ~30 GB | For 5 backups + overhead |
| **Compression ratio** | 2:1 | Typical tar.gz compression |

### Restore Performance

| Metric | Value | Notes |
|--------|-------|-------|
| **Extract time** | 2-3 min | Decompress + write to disk |
| **File restore time** | 5 min | Copy home + system files |
| **Workspace rebuild** | 10-15 min | `colcon build` full rebuild |
| **Total restore time** | 20-30 min | From start to ready |
| **Validation time** | 5 min | Post-restore verification |

### USB Transfer Speeds

| USB Version | Write Speed | Backup Time (5 GB) |
|-------------|-------------|-------------------|
| USB 2.0 | 10-20 MB/s | 4-8 minutes |
| USB 3.0 | 50-100 MB/s | 1-2 minutes |
| USB 3.1 | 100-200 MB/s | 30-60 seconds |

**Recommendation:** Use USB 3.0 or newer for faster backups.

---

## Troubleshooting Guide

### Backup Issues

**Problem:** "USB stick 'R2D2_BACKUP' not found!"

```bash
# Check if USB is plugged in
lsblk

# Check current label
lsblk -f | grep sda

# Relabel if needed
sudo fatlabel /dev/sda1 R2D2_BACKUP

# Unplug and replug USB
# Try backup again
```

**Problem:** "No space left on device"

```bash
# Check USB space
df -h /media/severin/R2D2_BACKUP

# Option 1: Delete old backups manually
rm -rf /media/severin/R2D2_BACKUP/r2d2_backup_20251101

# Option 2: Reduce retention count
# Edit script: RETENTION_COUNT=3

# Option 3: Use larger USB stick (64 GB+)
```

**Problem:** "Backup takes too long"

Normal for first backup (15-30 min). To speed up:
- Use USB 3.0+ stick (check for blue connector)
- Ensure USB is plugged into USB 3.0 port on Jetson
- Close other programs using disk I/O

**Problem:** "Backup folder for today already exists"

If you need to backup twice in one day:
- Type `yes` when prompted to overwrite
- Or manually rename existing backup folder first
- Or change script to use timestamp instead of date

### Restore Issues

**Problem:** "USB stick 'R2D2_BACKUP' not found!" (during restore)

Same solution as backup - ensure USB is plugged in and labeled correctly.

**Problem:** "No backups found on USB stick!"

```bash
# Check USB contents
ls -la /media/severin/R2D2_BACKUP

# Verify backup folders exist
ls -la /media/severin/R2D2_BACKUP/r2d2_backup_*/

# If empty, you may need to restore from another source
```

**Problem:** "Restore script fails with permission denied"

```bash
# Restore script requires root
sudo bash ~/dev/r2d2/scripts/r2d2_restore.sh

# Or check file permissions
ls -la ~/dev/r2d2/scripts/r2d2_restore.sh  # Should be executable
chmod +x ~/dev/r2d2/scripts/r2d2_restore.sh
```

**Problem:** "Backup file corrupted"

```bash
# Test archive integrity
tar -tzf /media/severin/R2D2_BACKUP/r2d2_backup_20251222/r2d2_backup.tar.gz > /dev/null

# If fails, try earlier backup
sudo bash ~/dev/r2d2/scripts/r2d2_restore.sh /media/severin/R2D2_BACKUP/r2d2_backup_20251215
```

**Problem:** "Workspace rebuild fails"

```bash
# Clean + full rebuild
cd ~/dev/r2d2/ros2_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build 2>&1 | tee build.log

# Check for specific errors
grep -i error build.log
```

### USB Stick Issues

**Problem:** USB not mounting automatically

```bash
# Check if kernel detects USB
dmesg | tail -20

# Manually mount
sudo mount /dev/sda1 /media/severin/R2D2_BACKUP

# Check filesystem
sudo fsck /dev/sda1  # For ext4
sudo fsck.vfat /dev/sda1  # For FAT32
```

**Problem:** USB mount is read-only

```bash
# Remount as read-write
sudo mount -o remount,rw /dev/sda1 /media/severin/R2D2_BACKUP

# If that fails, filesystem may be corrupted
sudo fsck /dev/sda1
```

---

## FAQ

### Q: Can I backup while ROS 2 is running?

**A:** The backup script is read-only, so technically yes. However, best practice:
```bash
# Stop ROS 2 first
pkill -9 -f ros2
sleep 2

# Then backup
bash ~/dev/r2d2/scripts/r2d2_backup.sh
```

### Q: How do I backup only the training data?

**A:** Extract specific files from backup:
```bash
# Mount USB and extract just data directory
cd /tmp
tar -xzf /media/severin/R2D2_BACKUP/r2d2_backup_20251222/r2d2_backup.tar.gz \
    home_staging/dev/r2d2/data/

# Copy where needed
cp -r home_staging/dev/r2d2/data ~/dev/r2d2/
```

### Q: Can I use multiple USB sticks?

**A:** Yes! Label them all `R2D2_BACKUP` and use different sticks for different backups. The script will detect whichever USB is plugged in.

### Q: What if I want to keep backups indefinitely?

**A:** Modify retention:
```bash
# Edit script
nano ~/dev/r2d2/scripts/r2d2_backup.sh

# Change:
RETENTION_COUNT=5  # to:
RETENTION_COUNT=-1  # (disabled, keeps all)
```

Or use multiple USB sticks and rotate them.

### Q: Can I restore to a different Jetson model?

**A:** Yes, for any Jetson AGX Orin. For other models (Orin Nano, Xavier):
- ✅ Code/data will work
- ⚠️ System configs may need review (udev rules, kernel-specific services)
- ⚠️ DepthAI may need recompiling

### Q: Does restore require internet?

**A:** Mostly no, except:
- `rosdep update` during setup (if first time)
- `apt-get` if installing missing packages
- Otherwise, all files come from backup

### Q: How large are typical backups?

**A:** Around 4-5 GB for the full system including:
- ROS 2 workspace source (~500 MB)
- Face recognition training data (~2-3 GB)
- Configuration and other files (~1 GB)

Backups are compressed (tar.gz), so actual size is similar.

### Q: Can I backup to a USB hard drive instead of flash drive?

**A:** Yes! Any USB storage device works. Just label it `R2D2_BACKUP`. External USB hard drives or SSDs work great for faster transfers.

### Q: What happens if backup is interrupted?

**A:** The script uses atomic operations:
1. Backup is created in a temporary folder
2. Only moved to final location when complete
3. If interrupted, partial backup is cleaned up
4. Old backups are never deleted until new one succeeds

Safe to interrupt (Ctrl+C) if needed.

### Q: Can I backup multiple Jetsons to one USB?

**A:** Yes, but modify the script to include hostname in folder name:
```bash
# Edit script:
BACKUP_FOLDER_NAME="r2d2_backup_${BACKUP_DATE}_$(hostname)"
```

This creates: `r2d2_backup_20251222_jetson-01/`, `r2d2_backup_20251222_jetson-02/`, etc.

---

## Security and Git Integration

### Two-Tier Documentation Strategy

The R2D2 project uses a two-tier approach to balance **security** (not publishing sensitive data) with **recoverability** (not losing critical configurations):

```
┌─────────────────────────────────────────────────────────────┐
│           TIER 1: GIT REPOSITORY (Public/Sanitized)         │
├─────────────────────────────────────────────────────────────┤
│  • Documentation with PLACEHOLDER IPs (100.x.x.x)          │
│  • Generic configuration examples                           │
│  • System architecture and procedures                       │
│  • Scripts (no hardcoded secrets)                          │
│  • Safe to publish to GitHub                               │
└─────────────────────────────────────────────────────────────┘
                              ↕
                   USB Backup bridges the gap
                              ↕
┌─────────────────────────────────────────────────────────────┐
│              TIER 2: USB BACKUP (Private/Complete)          │
├─────────────────────────────────────────────────────────────┤
│  • ~/.ssh/config with REAL Tailscale and USB IPs           │
│  • /etc/ssh/sshd_config with actual settings               │
│  • System configs with real network addresses               │
│  • API keys in environment files                           │
│  • Complete working configurations                          │
│  • NEVER published - stored physically on USB              │
└─────────────────────────────────────────────────────────────┘
```

### What USB Backup Preserves (Security-Sensitive)

The backup script captures these files with **real values**:

| File/Path | Contains | Why Backed Up |
|-----------|----------|---------------|
| `~/.ssh/config` | Real Tailscale IPs, USB network IPs | SSH connection settings |
| `/etc/ssh/sshd_config` | SSH keepalive settings | Connection stability config |
| `/etc/systemd/system/disable-*.service` | Power management fixes | USB/WiFi stability |
| `~/.bashrc`, `~/.bash_aliases` | Environment setup | Shell configuration |
| `/etc/udev/rules.d/r2d2_*` | Device rules | Hardware detection |

### Recovery After Restore

When you restore from USB backup:

1. ✅ **All real IPs restored automatically** - SSH configs work immediately
2. ✅ **All system services restored** - Power management, keepalives active
3. ✅ **No manual re-entry needed** - System works with actual network addresses
4. ✅ **Git documentation provides procedures** - USB provides real values

### Workflow: Secure Git Commits

**Before committing documentation changes to git:**

1. **Ensure USB backup is current** (preserves real configs)
   ```bash
   bash ~/dev/r2d2/scripts/r2d2_backup.sh
   ```

2. **Sanitize documentation** (replace real IPs with placeholders)
   ```bash
   # Check for real IPs
   grep -rE "100\.[0-9]+\.[0-9]+\.[0-9]+" ~/dev/r2d2/*.md
   grep -rE "192\.168\.[0-9]+\.[0-9]+" ~/dev/r2d2/*.md
   
   # Replace with placeholders: 100.x.x.x, 192.168.x.1
   ```

3. **Commit sanitized version to git**
   ```bash
   git add .
   git commit -m "docs: update with sanitized examples"
   git push origin main
   ```

4. **Real values remain safe on USB backup**

### Security Checklist

Before any git operation:

- [ ] USB backup completed (real configs preserved)
- [ ] Documentation sanitized (placeholder IPs only)
- [ ] No API keys or credentials in committed files
- [ ] Email addresses replaced with `user@example.com`

### Why This Matters

| Scenario | Without Two-Tier | With Two-Tier |
|----------|------------------|---------------|
| Git repo leaked | Real IPs exposed, network compromised | Only placeholders visible |
| USB lost | No recovery of settings | Git has procedures, can rebuild |
| New Jetson setup | Manual IP entry | Restore from USB, works immediately |
| Documentation sharing | Must redact manually | Already sanitized in git |

**For detailed security guidelines, see:** `000_INTERNAL_AGENT_NOTES.md` (Security section)

---

## Related Documentation

See also:
- `00_INTERNAL_AGENT_NOTES.md` — Platform-specific quirks & patterns
- `01_R2D2_BASIC_SETUP_AND_FINDINGS.md` — Initial Jetson setup
- `README.md` — Project overview

---

## Summary

The **R2D2 USB Backup System** provides:

✅ **Complete system backup** in 15-30 minutes  
✅ **Fast restoration** to a new Jetson in 20-30 minutes  
✅ **USB-only design** (no cloud dependencies)  
✅ **Date-versioned folders** (easy to browse)  
✅ **Automatic cleanup** (keeps last 5 backups)  
✅ **Auto USB detection** (label-based)  
✅ **Non-destructive** (requires confirmation to overwrite)  
✅ **Portable** (USB can be stored anywhere)

You can now confidently wipe, upgrade, or replace your Jetson knowing that a complete, reproducible backup is always on your USB stick.

---

**Backup & Restore System: USB Edition ✅**
