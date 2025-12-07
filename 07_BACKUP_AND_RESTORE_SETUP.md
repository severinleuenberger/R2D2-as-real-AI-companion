# R2D2 Backup & Restore System Setup
**Complete guide for backing up and restoring your NVIDIA Jetson AGX Orin R2D2 system**

**Date:** December 7, 2025  
**Project:** R2D2 as a Real AI Companion  
**Platform:** NVIDIA Jetson AGX Orin 64GB Developer Kit  
**Status:** ✅ **READY TO USE**

---

## Table of Contents

1. [Overview](#overview)
2. [Why Backup & Restore?](#why-backup--restore)
3. [System Architecture](#system-architecture)
4. [What Gets Backed Up](#what-gets-backed-up)
5. [Fresh Jetson Setup](#fresh-jetson-setup)
6. [Creating a Backup](#creating-a-backup)
7. [Restoring from Backup](#restoring-from-backup)
8. [Windows Integration](#windows-integration)
9. [Testing & Validation](#testing--validation)
10. [Troubleshooting](#troubleshooting)
11. [FAQ](#faq)

---

## Overview

The **R2D2 Backup & Restore System** allows you to:

- **Backup** your entire Jetson system (code, data, configs, training artifacts)
- **Restore** to a fresh or new Jetson in less than 30 minutes
- **Automate** backup from Windows via PowerShell
- **Archive** backups to OneDrive for long-term storage

This system has been designed for **complete reproducibility**: after flashing a new Jetson and running the setup script, you can restore a backup and have your system behave **exactly as before**, including:

- All ROS 2 packages and workspace
- Camera and perception nodes
- Face recognition training data and models
- System configurations (udev rules, systemd services)
- User environment settings (.bashrc, aliases)

---

## Why Backup & Restore?

### Problems Solved

1. **Hardware Replacement**: Buy a new Jetson, flash JetPack, restore your system in ~30 min
2. **Fresh Start**: Wipe everything and rebuild without re-training or re-configuring
3. **Archival**: Store snapshots of working system states
4. **Multi-Device**: Deploy identical setup across multiple Jetson boards
5. **Development Safety**: Backup before risky operations, restore if things break

### Risk Mitigation

- Automated retention (keeps last 10 backups)
- Metadata tracking (shows what was backed up)
- Non-destructive restore (requires confirmation before overwriting)
- System validation after restore

---

## System Architecture

### Three-Part Backup System

```
┌─────────────────┐
│   Jetson AGX    │
│     Orin 64GB   │
├─────────────────┤
│  r2d2_backup.sh │  ─────────> ~/backups/*.tar.gz
│                 │
└─────────────────┘
         ↑
         │ SSH + scp
         │
┌─────────────────┐
│  Windows Laptop │
│ r2d2_backup.ps1 │  ─────────> OneDrive\R2D2\
└─────────────────┘
```

### Three Scripts

| Script | Purpose | Location | User | Privilege |
|--------|---------|----------|------|-----------|
| `r2d2_setup.sh` | Fresh Jetson setup | Jetson | Root | sudo required |
| `r2d2_backup.sh` | Create backup archive | Jetson | Regular user | User level |
| `r2d2_restore.sh` | Restore from backup | Jetson | Root | sudo required |
| `r2d2_backup.ps1` | Trigger backup from Windows | Windows | Any | Admin helpful |

---

## What Gets Backed Up

### ✅ Included

**Project Files**
- `~/dev/r2d2/` — entire project root
  - `ros2_ws/src/` — all ROS 2 packages and source code
  - `data/` — face recognition datasets, embeddings, training outputs
  - Documentation files (00_*.md through 06_*.md)

**User Configuration**
- `~/.bashrc` — shell environment setup
- `~/.bash_aliases` — custom command aliases
- `~/.config/r2d2/` — future R2D2-specific config directory

**System Configuration** (if present)
- `/etc/udev/rules.d/r2d2_*` — USB/device rules
- `/etc/systemd/system/r2d2_*.service` — background services

**Environment & Data**
- DepthAI virtual environment setup references
- Camera calibration files
- Face recognition training models
- All data files (JPEG frames, embeddings, labels)

### ❌ Excluded (Not Backed Up)

These are **automatically excluded** because they're regeneratable:

- `ros2_ws/build/` — ROS 2 build artifacts
- `ros2_ws/install/` — ROS 2 installed packages
- `ros2_ws/log/` — ROS 2 logs
- All `.git/` directories — version control metadata
- `__pycache__/` — Python bytecode caches
- `*.pyc`, `*.pyo` — compiled Python files

**Why excluded?** They're large, regeneratable, and take excessive space. The restore script rebuilds them automatically.

---

## Fresh Jetson Setup

### Prerequisites

- NVIDIA Jetson AGX Orin (64GB recommended)
- Ubuntu 22.04 L4T + JetPack 6.x (freshly flashed)
- SSH access from your host
- `sudo` privileges on the Jetson
- ~30 minutes of time

### Step-by-Step Setup

#### 1. Flash the Jetson

Use NVIDIA SDK Manager on a host machine to flash the Jetson:

```bash
# On Ubuntu-on-USB host
sdkmanager
# Select: Jetson AGX Orin (64GB)
# Select: JetPack 6.x
# Flash the device
```

After flashing, verify with:

```bash
ssh severin@192.168.55.1
nvidia-smi          # Should show NVIDIA GPU driver
tegrastats          # Should show CPU/GPU/memory stats
```

#### 2. Run Setup Script

Copy the setup script to the Jetson and run it:

```bash
# From your host/laptop
scp scripts/r2d2_setup.sh severin@192.168.55.1:~/
ssh severin@192.168.55.1

# On the Jetson
sudo bash r2d2_setup.sh
```

This script will:
- Update all packages
- Install ROS 2 Humble
- Clone the R2D2 repository
- Create and configure the ROS 2 workspace
- Build the base packages
- Configure bash environment
- Set up DepthAI Python environment
- Link any existing system configs

**Expected time:** 20-30 minutes (depends on network and Jetson speed)

#### 3. Verify Setup

After setup completes, verify the system:

```bash
source ~/.bashrc
ros2 topic list          # Should show /r2d2/heartbeat
ros2 launch r2d2_bringup bringup.launch.py
# In another terminal:
ros2 topic echo /r2d2/heartbeat -n 5
```

---

## Creating a Backup

### On the Jetson

#### Manual Backup

```bash
# Method 1: Direct command
bash ~/dev/r2d2/scripts/r2d2_backup.sh

# Method 2: From the project directory
cd ~/dev/r2d2
bash scripts/r2d2_backup.sh
```

**Output:**
```
[INFO] Starting backup at Mon Dec  7 14:23:45 UTC 2025
[INFO] Backup destination: /home/severin/backups/r2d2_backup_20251207_142345.tar.gz
...
[INFO] Backup created successfully: 4.2G
[INFO] Archive: /home/severin/backups/r2d2_backup_20251207_142345.tar.gz
```

#### List Available Backups

```bash
ls -lh ~/backups/r2d2_backup_*.tar.gz
```

#### Automatic Cleanup

The backup script automatically keeps the **last 10 backups**. Older backups are automatically deleted. To change this:

Edit `scripts/r2d2_backup.sh` and modify:

```bash
RETENTION_COUNT=10  # Change this number
```

### From Windows

Use the PowerShell script to backup and copy to OneDrive automatically:

```powershell
# Edit the script first
notepad windows/r2d2_backup.ps1
# Update configuration section if needed

# Run the script
powershell -ExecutionPolicy Bypass -File windows/r2d2_backup.ps1
```

**What it does:**
1. Tests SSH connection to Jetson
2. Triggers `r2d2_backup.sh` on the Jetson
3. Finds the latest backup file
4. Copies it to `C:\Users\SeverinLeuenberger\OneDrive\Daten Severin\_Dev_Git_Projekte\R2D2\`
5. Optionally removes old backups from Jetson to save space
6. Logs everything to a file

---

## Restoring from Backup

### Prerequisites for Restore

1. A fresh Jetson that has been:
   - Flashed with JetPack 6.x
   - Had `r2d2_setup.sh` run to completion
   - Can SSH from your host
   
2. A backup archive available at:
   - `~/backups/r2d2_backup_*.tar.gz` on the Jetson, OR
   - Any path you specify manually

### Step-by-Step Restore

#### 1. Prepare the Jetson

```bash
# From your host
ssh severin@192.168.55.1

# On the Jetson
cd ~/dev/r2d2
```

#### 2. Copy Backup to Jetson (if needed)

If the backup is on your Windows laptop:

```powershell
# On Windows PowerShell
$backup = "C:\Users\SeverinLeuenberger\OneDrive\Daten Severin\_Dev_Git_Projekte\R2D2\r2d2_backup_20251207_142345.tar.gz"
scp "$backup" severin@192.168.55.1:~/backups/
```

#### 3. Run Restore Script

```bash
# Method 1: Use latest backup (automatic)
sudo bash scripts/r2d2_restore.sh

# Method 2: Specify a backup
sudo bash scripts/r2d2_restore.sh ~/backups/r2d2_backup_20251207_142345.tar.gz

# Method 3: Skip confirmation (for scripting)
sudo bash scripts/r2d2_restore.sh --yes ~/backups/r2d2_backup_*.tar.gz
```

#### 4. Confirmation Prompt

You'll see:

```
[WARN] RESTORE WARNING
This will restore files from the backup to your home directory and system.
Existing files will be overwritten.

Backup file: /home/severin/backups/r2d2_backup_20251207_142345.tar.gz
Target user: severin

Do you want to continue? (yes/no):
```

Type `yes` to proceed (or use `--yes` flag to skip).

#### 5. Restore Progress

The script will:

```
=== Extracting Backup ===
...
=== Restoring Home Directory Files ===
[INFO] Restoring project root...
[INFO] Restoring .bashrc...
...
=== Restoring System Files ===
[INFO] Restoring udev rules...
[INFO] Restoring systemd services...
...
=== Rebuilding ROS 2 Workspace ===
[INFO] Building workspace (this may take a few minutes)...
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

## Windows Integration

### Configure PowerShell Script

Edit `windows/r2d2_backup.ps1` and update the configuration section:

```powershell
# CONFIGURATION SECTION

$JetsonUser = "severin"                          # Jetson username
$JetsonHost = "192.168.55.1"                    # Jetson IP or hostname
$JetsonSSHKey = "$env:USERPROFILE\.ssh\id_rsa"  # SSH private key path

# Windows backup destination (IMPORTANT - Do not change unless necessary)
$WindowsBackupPath = "C:\Users\SeverinLeuenberger\OneDrive\Daten Severin\_Dev_Git_Projekte\R2D2"

# Optional settings
$KeepJetsonBackups = "yes"        # Keep copies on Jetson
$RemoveAfterTransfer = "no"       # Auto-delete from Jetson after transfer
```

### SSH Key Setup on Windows

Ensure you have an SSH key pair:

```powershell
# Check if key exists
Test-Path $env:USERPROFILE\.ssh\id_rsa

# If not, generate one
ssh-keygen -t rsa -b 4096 -f "$env:USERPROFILE\.ssh\id_rsa" -N ""

# Copy public key to Jetson
Get-Content "$env:USERPROFILE\.ssh\id_rsa.pub" | ssh severin@192.168.55.1 "cat >> ~/.ssh/authorized_keys"
```

### Run Backup from Windows

```powershell
# Method 1: Simple run
powershell -ExecutionPolicy Bypass -File C:\path\to\r2d2_backup.ps1

# Method 2: With transcript logging
powershell -ExecutionPolicy Bypass -NoExit `
  -Command ". C:\path\to\r2d2_backup.ps1 | Tee-Object -FilePath C:\logs\backup.log"

# Method 3: Scheduled task (automatic daily backup)
# See Windows Task Scheduler for automation
```

### Backup Results

Backups appear in:

```
C:\Users\SeverinLeuenberger\OneDrive\Daten Severin\_Dev_Git_Projekte\R2D2\
├── r2d2_backup_20251207_140000.tar.gz  (4.2 GB)
├── r2d2_backup_20251206_200000.tar.gz  (4.2 GB)
└── r2d2_backup_20251205_200000.tar.gz  (4.1 GB)
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
find ~/dev/r2d2/data/face_recognition -name "*.yml" -o -name "*.xml"

# Check label mappings
echo "Label files:"
cat ~/dev/r2d2/data/face_recognition/labels.json 2>/dev/null || echo "(not present)"
```

#### 4. Camera Verification (optional)

```bash
# Test camera enumeration
lsusb | grep -i oak

# Source ROS environment
source ~/.bashrc

# Launch camera node
timeout 10 ros2 launch r2d2_camera oak_d_camera.launch.py 2>&1 | tail -20

# Check camera topic
ros2 topic list | grep oak
```

#### 5. System Configuration

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

## Troubleshooting

### Backup Issues

**Problem:** "Backup file was not created"

```bash
# Check disk space
df -h
# Expected: At least 5-10GB free in home directory

# Check backup permissions
ls -la ~/backups/
# Should be writable by your user
```

**Problem:** "Backup takes too long"

The first backup may take 15-30 minutes depending on:
- Total project size (~4-5 GB for full system)
- I/O speed (eMMC on Jetson is slower than NVMe)
- Network speed (if copying to Windows)

Subsequent backups are faster because old ones are removed.

**Problem:** "No backup files found"

```bash
# Check if backup script was actually executed
cat ~/backups/  # Should contain r2d2_backup_*.tar.gz files

# Run backup manually with verbose output
bash ~/dev/r2d2/scripts/r2d2_backup.sh
```

### Restore Issues

**Problem:** "SSH connection failed" (from Windows)

```powershell
# Test SSH manually
ssh -i $env:USERPROFILE\.ssh\id_rsa severin@192.168.55.1 "echo OK"

# Check SSH key permissions (should be 600)
ls -la $env:USERPROFILE\.ssh\id_rsa

# Verify Jetson is reachable
ping 192.168.55.1
```

**Problem:** "Restore script fails with permission denied"

```bash
# Restore script requires sudo
sudo bash scripts/r2d2_restore.sh

# Or add your user to sudoers (not recommended for security)
sudo visudo
# Add: severin ALL=(ALL) NOPASSWD: /home/severin/dev/r2d2/scripts/r2d2_restore.sh
```

**Problem:** "Workspace rebuild fails"

```bash
# Clean rebuild
cd ~/dev/r2d2/ros2_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build 2>&1 | tee build.log

# Check for specific errors
grep -i error build.log
```

### Common Errors

| Error | Cause | Solution |
|-------|-------|----------|
| `scp: not found` | OpenSSH not installed on Windows | Install [Git for Windows](https://git-scm.com/download/win) |
| `Permission denied` | SSH key not in authorized_keys | Run key setup step above |
| `Backup file corrupted` | Network interruption during copy | Re-run backup and copy steps |
| `No space left on device` | Jetson storage full | Clean old backups, increase storage |
| `Illegal instruction` | DepthAI ARM issue | Ensure `export OPENBLAS_CORETYPE=ARMV8` in bashrc |

---

## FAQ

### Q: How large are typical backups?

**A:** Around 4-5 GB for the full system including:
- ROS 2 workspace source (~500 MB)
- Face recognition training data (~2-3 GB)
- Configuration and other files (~1 GB)

Backups are compressed (tar.gz), so actual size on disk is similar.

### Q: Can I backup to a USB drive instead of OneDrive?

**A:** Yes! Modify `windows/r2d2_backup.ps1`:

```powershell
$WindowsBackupPath = "D:\R2D2_Backups"  # USB drive letter
```

Or use the Jetson script directly and manually copy files.

### Q: How often should I backup?

**A:** Recommended frequency:

- **Daily**: After major training sessions or configuration changes
- **Weekly**: Regular development activity
- **Before**: Any risky operations (system updates, hardware changes)
- **After**: Completing milestones (new face recognition training, etc.)

### Q: Can I restore to a different Jetson board?

**A:** Yes! The restore system is **hardware-agnostic** (for Jetson boards). Just:

1. Flash the new Jetson with JetPack 6.x
2. Run `r2d2_setup.sh`
3. Transfer backup and run `r2d2_restore.sh`

Note: Some hardware-specific configs (udev rules, systemd services) may need review if the new board is different.

### Q: What if I want to keep multiple versions of a backup?

**A:** The script keeps 10 backups automatically. To keep more:

Edit `scripts/r2d2_backup.sh`:

```bash
RETENTION_COUNT=20  # Keep last 20 instead of 10
```

Or manually copy old backups to a separate archive:

```bash
cp ~/backups/r2d2_backup_20251207_140000.tar.gz ~/backups/archive/
```

### Q: Can I restore only certain files (not the whole system)?

**A:** The current script restores everything. For partial restore:

```bash
# Extract backup manually
mkdir temp
tar -xzf r2d2_backup_20251207_140000.tar.gz -C temp

# Copy only what you need
cp -r temp/home_staging/dev/r2d2/data ~/dev/r2d2/
```

### Q: How do I backup to a remote server (instead of OneDrive)?

**A:** Modify the PowerShell script to use `scp` to your server:

```powershell
$RemoteServer = "user@backup.example.com:/backups/r2d2"
scp "${latestBackup}" "${RemoteServer}/"
```

Or use `rclone` for cloud storage (Google Drive, AWS S3, etc.).

### Q: What about backing up the ROS 2 workspace install/build?

**A:** Intentionally excluded because:

1. **Huge**: 1-2 GB per architecture
2. **Regeneratable**: `colcon build` rebuilds in ~10 minutes
3. **Non-portable**: Build artifacts are architecture-specific

The restore script automatically rebuilds the workspace.

### Q: Can I backup while the robot is running?

**A:** Not recommended, but the script is **read-only**. For safety:

```bash
# Stop all ROS 2 processes first
pkill -9 -f ros2
sleep 2

# Then backup
bash ~/dev/r2d2/scripts/r2d2_backup.sh
```

---

## Summary

The **R2D2 Backup & Restore System** provides:

✅ **Complete system backup** in 15-30 minutes  
✅ **Fast restoration** to a new Jetson in 20-30 minutes  
✅ **Windows automation** via PowerShell  
✅ **OneDrive integration** for cloud archival  
✅ **Automatic cleanup** (keeps last 10 backups)  
✅ **Non-destructive** (requires confirmation to overwrite)  
✅ **Validated** (includes post-restore verification steps)

You can now confidently wipe, upgrade, or replace your Jetson knowing that a complete, reproducible backup is always available.

---

## Related Documentation

See also:
- `01_R2D2_BASIC_SETUP_AND_FINDINGS.md` — Initial Jetson setup
- `02_CAMERA_SETUP_DOCUMENTATION.md` — OAK-D camera integration
- `06_FACE_RECOGNITION_TRAINING_AND_STATUS.md` — Training data structure
- `README.md` — Project overview
