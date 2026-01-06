# NVMe SSD Storage Expansion Plan

**Date:** January 5, 2026  
**Hardware:** WD Blue SN5000 500GB NVMe (M.2 2280, PCIe 4.0)  
**Platform:** NVIDIA Jetson AGX Orin 64GB  
**Status:** Ready for execution

---

## Safety Guarantees

| What | Protection |
|------|------------|
| Boot system | Untouched - eMMC boot partition not modified |
| OS installation | Untouched - no system files changed |
| User data | Backed up to USB before any changes |
| Services | Preserved via symlinks - original paths work |
| Rollback | Documented for every phase |

```mermaid
flowchart TD
    subgraph protected [eMMC - Protected]
        Boot[Boot Partition]
        OS[Ubuntu OS]
    end
    
    subgraph added [NVMe - New Storage]
        Data[/data mount]
        Venvs[Python Envs]
        Cache[Caches]
    end
    
    subgraph safety [Safety Layer]
        Backup[USB Backup]
        Symlinks[Symlinks]
    end
    
    Backup -->|restore if needed| protected
    Symlinks -->|redirect| Data
    Data --> Venvs
    Data --> Cache
```

---

## Phase 0: Pre-Migration USB Backup

**Before ANY changes, backup critical directories to USB:**

```bash
# Mount USB drive
USB_MOUNT=/media/severin/DAEA-DEBB

# Create backup directory with timestamp
BACKUP_DIR="$USB_MOUNT/nvme_migration_backup_$(date +%Y%m%d)"
mkdir -p "$BACKUP_DIR"

# Backup directories that will be moved
echo "Backing up depthai_env (2GB)..."
tar -czf "$BACKUP_DIR/depthai_env.tar.gz" -C ~ depthai_env

echo "Backing up r2d2_speech_env (1.3GB)..."
tar -czf "$BACKUP_DIR/r2d2_speech_env.tar.gz" -C ~/dev/r2d2 r2d2_speech_env

echo "Backing up .cache..."
tar -czf "$BACKUP_DIR/dot_cache.tar.gz" -C ~ .cache

echo "Backing up .bashrc..."
cp ~/.bashrc "$BACKUP_DIR/bashrc.backup"

echo "Backing up /etc/fstab..."
sudo cp /etc/fstab "$BACKUP_DIR/fstab.backup"

# Verify backups exist
ls -lh "$BACKUP_DIR"
echo "Backup complete!"
du -sh "$BACKUP_DIR"
```

**Verification:** Confirm all .tar.gz files exist (~3.5GB total)

**Rollback:** N/A - this is the safety net

---

## Phase 1: Physical Installation

1. Power off: `sudo shutdown -h now`
2. Disconnect power cable
3. Remove bottom cover with Phillips screwdriver
4. Insert SSD into M.2 Key-M slot at 30-degree angle
5. Press down and secure with screw
6. Replace cover, reconnect power, boot

**Verification:** System boots normally from eMMC

**Rollback:** Remove SSD if hardware issues

---

## Phase 2: Partition and Format

```bash
# Verify drive detected
lsblk | grep nvme
# Expected: nvme0n1  465.8G  disk

# Create partition
sudo parted /dev/nvme0n1 --script mklabel gpt
sudo parted /dev/nvme0n1 --script mkpart primary ext4 0% 100%

# Format
sudo mkfs.ext4 -L r2d2-data /dev/nvme0n1p1
```

**Verification:** `lsblk -f /dev/nvme0n1` shows ext4 partition

**Rollback:** Not needed - new drive only

---

## Phase 3: Temporary Mount Test

```bash
sudo mkdir -p /data
sudo mount /dev/nvme0n1p1 /data

# Test read/write
sudo touch /data/test && sudo rm /data/test
df -h /data
```

**Verification:** Mount works, read/write works

**Rollback:** `sudo umount /data`

---

## Phase 4: Permanent Mount via fstab

```bash
# Get UUID
UUID=$(sudo blkid -s UUID -o value /dev/nvme0n1p1)
echo "UUID is: $UUID"

# Backup fstab
sudo cp /etc/fstab /etc/fstab.pre-nvme

# Add entry with nofail for safe boot
echo "UUID=$UUID  /data  ext4  defaults,noatime,nofail  0  2" | sudo tee -a /etc/fstab

# TEST BEFORE REBOOT
sudo mount -a
df -h /data
```

**Verification:** `mount -a` succeeds, `/data` shows in `df`

**Rollback:**
```bash
sudo nano /etc/fstab  # Delete the UUID line
# Or restore: sudo cp /etc/fstab.pre-nvme /etc/fstab
```

---

## Phase 5: Migrate Directories

```bash
# Set ownership
sudo chown -R severin:severin /data

# Create structure
mkdir -p /data/venvs /data/cache/pip /data/cache/huggingface /data/cache/torch /data/models /data/projects

# Migrate depthai_env
mv ~/depthai_env /data/venvs/
ln -s /data/venvs/depthai_env ~/depthai_env
# VERIFY:
source ~/depthai_env/bin/activate && python -c "import depthai; print('OK')" && deactivate

# Migrate r2d2_speech_env
mv ~/dev/r2d2/r2d2_speech_env /data/venvs/
ln -s /data/venvs/r2d2_speech_env ~/dev/r2d2/r2d2_speech_env
# VERIFY:
source ~/dev/r2d2/r2d2_speech_env/bin/activate && python -c "print('OK')" && deactivate

# Migrate cache
mv ~/.cache /data/cache/user_cache
ln -s /data/cache/user_cache ~/.cache
```

**Verification:** Each symlink works, Python imports succeed

**Rollback per directory:**
```bash
rm ~/depthai_env
tar -xzf /media/severin/DAEA-DEBB/nvme_migration_backup_*/depthai_env.tar.gz -C ~/
```

---

## Phase 6: Environment Variables for Future Growth

Add to `~/.bashrc`:

```bash
# === NVMe Storage Redirect ===
export PIP_CACHE_DIR=/data/cache/pip
export HF_HOME=/data/cache/huggingface
export TRANSFORMERS_CACHE=/data/cache/huggingface/transformers
export TORCH_HOME=/data/cache/torch
export XDG_CACHE_HOME=/data/cache
# === End NVMe Redirect ===
```

Apply: `source ~/.bashrc`

**Verification:** `echo $PIP_CACHE_DIR` shows `/data/cache/pip`

**Rollback:** Remove lines from ~/.bashrc

---

## Phase 7: Final Verification

```bash
sudo reboot

# After reboot:
df -h / /data
ls -la ~/depthai_env
source ~/depthai_env/bin/activate && python -c "import depthai" && deactivate
echo $PIP_CACHE_DIR
```

---

## Complete Rollback Procedure

If anything goes wrong, restore everything:

```bash
USB_BACKUP=/media/severin/DAEA-DEBB/nvme_migration_backup_*

# Restore .bashrc
cp $USB_BACKUP/bashrc.backup ~/.bashrc
source ~/.bashrc

# Restore directories
rm ~/depthai_env
tar -xzf $USB_BACKUP/depthai_env.tar.gz -C ~/

rm ~/dev/r2d2/r2d2_speech_env
tar -xzf $USB_BACKUP/r2d2_speech_env.tar.gz -C ~/dev/r2d2/

rm ~/.cache
tar -xzf $USB_BACKUP/dot_cache.tar.gz -C ~/

# Restore fstab
sudo cp $USB_BACKUP/fstab.backup /etc/fstab

sudo reboot
```

---

## Future Development Conventions

| Type | Location | How |
|------|----------|-----|
| New Python venvs | /data/venvs/ | `python3 -m venv /data/venvs/name` |
| Large models | /data/models/ | Manual placement |
| Caches | /data/cache/ | Automatic via env vars |
| Project data | /data/projects/ | Manual placement |

**Example: Creating new virtual environment**
```bash
# Create on NVMe
python3 -m venv /data/venvs/my_new_project

# Symlink for convenience (optional)
ln -s /data/venvs/my_new_project ~/dev/my_project/venv
```

---

## Expected Results

| Metric | Before | After |
|--------|--------|-------|
| eMMC Used | 88% (47GB) | ~75% (~43GB) |
| eMMC Free | 6.7GB | ~10GB |
| NVMe Used | N/A | ~4GB |
| NVMe Free | N/A | ~460GB |
| Total Storage | 57GB | 557GB |

---

## Execution Checklist

- [ ] Phase 0: USB Backup complete
- [ ] Phase 1: SSD physically installed
- [ ] Phase 2: Partitioned and formatted
- [ ] Phase 3: Temporary mount tested
- [ ] Phase 4: fstab configured, `mount -a` successful
- [ ] Phase 5: Directories migrated, symlinks verified
- [ ] Phase 6: Environment variables added to ~/.bashrc
- [ ] Phase 7: Final verification after reboot
- [ ] Documentation updated with actual UUID used
- [ ] Backup USB drive stored safely

---

## Notes

**Date Executed:** _[To be filled]_  
**UUID Used:** _[To be filled]_  
**Issues Encountered:** _[To be filled]_  
**Final Disk Usage:** _[To be filled]_

---

**End of Document**

