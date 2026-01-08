# R2D2 Agent Core Rules

**Purpose:** Minimal essential instructions always loaded with every agent query. Task-specific guides are loaded on-demand.

**Last Updated:** January 5, 2026

---

## Identity & Environment

**Platform:** NVIDIA Jetson AGX Orin 64GB (ARM64, aarch64)  
**Workspace:** `/home/severin/dev/r2d2`  
**Execution Context:** You execute DIRECTLY on the Jetson (not via SSH)

---

## Critical Rules (ALWAYS Follow)

### 1. UX Changes: ALWAYS ASK FIRST ⚠️

Before implementing ANY user-facing change, you MUST consult the user:

**UX-Affecting Areas:**
- Audio feedback (sounds, volume, timing)
- Visual feedback (LED patterns, colors, brightness)
- Gesture recognition behavior
- Conversation flow and timeouts
- Speech system personalities
- Web dashboard layout
- Recognition thresholds and state transitions

**Consultation Process:**
1. Read [`000_UX_AND_FUNCTIONS.md`](000_UX_AND_FUNCTIONS.md) for current spec
2. Present options with UX impact and tradeoffs
3. Wait for user confirmation

**Example:**
```
UX CONSULTATION REQUIRED:
This feature affects [area].
Current spec shows: [behavior]
Options:
- A: [description + impact]
- B: [description + impact]
Which do you prefer?
```

### 2. Security: NO Sensitive Data in Git ❌

**NEVER commit:**
- Real IP addresses (use `100.x.x.x`, `192.168.x.1`)
- API keys or tokens
- SSH key fingerprints
- Real email addresses (use `user@example.com`)
- Credentials or passwords

**Before EVERY commit:**
```bash
grep -rE "100\.[0-9]+\.[0-9]+\.[0-9]+" ~/dev/r2d2/*.md
grep -rE "192\.168\.[0-9]+\.[0-9]+" ~/dev/r2d2/*.md
grep -rE "[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}" ~/dev/r2d2/*.md
```

### 3. Git Workflow: Main + Tags 🏷️

**Simple Workflow (For Beginners):**

1. **Work on `main` branch** (default)
2. **Create golden tags** when everything works
3. **Rollback if needed** using tags

**Creating a Golden Tag:**
```bash
# After testing and confirming everything works:
git tag -a golden-YYYY-MM-DD -m "Brief description of what's working"
git push origin golden-YYYY-MM-DD
```

**Viewing All Golden Tags:**
```bash
git tag -l "golden-*"
```

**Rolling Back to a Golden Point:**
```bash
# Create a backup branch first (optional but safe):
git branch backup-before-rollback

# Roll back to the golden tag:
git reset --hard golden-YYYY-MM-DD

# If you also want to update GitHub (use with caution):
git push --force origin main
```

**Important Notes:**
- Tags are **permanent checkpoints** you can always return to
- They're simpler than branches (no merging needed)
- Golden tags = "known good states"
- Never use `master` branch (deleted)

**Advanced Users:** Task-specific instructions may override with feature branches

### 4. Code is Truth 🔍

**When docs conflict with code: CODE IS CORRECT**

- Update docs to match code, never vice versa
- Check actual source files, not documentation
- Config files (`.yaml`) are authoritative for parameters

### 5. Environment Setup (ARM-Specific) 🔧

**ALWAYS run in this order:**
```bash
source ~/depthai_env/bin/activate
source ~/.bashrc
source ~/dev/r2d2/ros2_ws/install/setup.bash
export OPENBLAS_CORETYPE=ARMV8
```

**Critical:** `OPENBLAS_CORETYPE=ARMV8` prevents "Illegal instruction" crash on ARM.

**Troubleshooting ARM issues:** See [`103_PERCEPTION_STATUS_TROUBLESHOOTING.md`](103_PERCEPTION_STATUS_TROUBLESHOOTING.md)

### 6. GPU-First Development 🚀

**Leverage the AI hardware: Use GPU for compute-intensive workloads**

The Jetson AGX Orin has a 2048-core Ampere GPU (504 CUDA cores) delivering 20-50x performance improvements for the right workloads.

**GPU-Acceleratable Workloads:**
- Machine learning inference (STT, TTS, vision models)
- Image/video processing
- Large matrix operations
- Signal processing

**For implementation details:** See [`002_HARDWARE_GPU_ACCELERATION.md`](002_HARDWARE_GPU_ACCELERATION.md)

**Quick test:** `sudo /home/severin/r2d2-gpu-run.sh test`

### 7. Storage Usage Guidelines 💾

**CRITICAL RULE: Always use NVMe SSD (/data) for new development, NOT eMMC**

**Implementation Date:** January 8, 2026

The R2D2 system uses a two-tier storage architecture:
- **eMMC (57GB):** Boot, OS, source code, small configs
- **NVMe (/data - 500GB):** Virtual environments, models, caches, project data

#### Storage Decision Matrix

| Data Type | Location | Reasoning |
|-----------|----------|-----------|
| **Python virtual environments** | `/data/venvs/` | Large (100MB-2GB each), frequently updated |
| **ML model files** | `/data/models/` | Very large (100MB-10GB), infrequently changed |
| **Cache directories** | `/data/cache/` | Grows over time, safe to delete |
| **Project data/datasets** | `/data/projects/` | Large data files, project-specific |
| **Conversation logs/databases** | `/data/projects/r2d2/` | Grows over time, important for memory |
| **Source code** | `~/dev/r2d2/` (eMMC) | Small, needs backup, version controlled |
| **Config files** | `~/dev/r2d2/config/` (eMMC) | Small, critical, needs backup |
| **Documentation** | `~/dev/r2d2/` (eMMC) | Small, version controlled |
| **ROS 2 workspace** | `~/dev/r2d2/ros2_ws/` (eMMC) | Small source, build artifacts can use /data |

#### Default Paths for New Development

**When creating new Python virtual environments:**
```bash
python3 -m venv /data/venvs/my_new_project
ln -s /data/venvs/my_new_project ~/dev/my_project/venv  # Optional convenience link
```

**When downloading ML models:**
```bash
# Models auto-download to /data/cache/ via environment variables:
# - HF_HOME=/data/cache/huggingface
# - TRANSFORMERS_CACHE=/data/cache/huggingface/transformers
# - TORCH_HOME=/data/cache/torch

# For manual placement:
cp large_model.bin /data/models/
```

**When creating project data directories:**
```bash
mkdir -p /data/projects/my_project/datasets
mkdir -p /data/projects/my_project/output
```

**When building ROS 2 packages:**
```bash
# Source code stays on eMMC (already there)
cd ~/dev/r2d2/ros2_ws
colcon build  # Build artifacts stay on eMMC (manageable size)
```

#### Environment Variables (Already Configured)

These are set in `~/.bashrc`:
```bash
export PIP_CACHE_DIR=/data/cache/pip
export HF_HOME=/data/cache/huggingface
export TRANSFORMERS_CACHE=/data/cache/huggingface/transformers
export TORCH_HOME=/data/cache/torch
export XDG_CACHE_HOME=/data/cache
```

#### What NOT to Store on NVMe

- Boot files (stay on eMMC)
- System packages (stay on eMMC)
- Small config files (stay on eMMC for easy backup)
- Git repositories (stay on eMMC, version controlled)

#### Monitoring Storage Usage

```bash
# Check eMMC usage
df -h /

# Check NVMe usage
df -h /data

# Check specific directories
du -sh /data/venvs/*
du -sh /data/cache/*
du -sh /data/models/*
```

#### Agent Instructions

When proposing new code or features:
1. **Default to NVMe** for any data that will grow >100MB
2. **Use eMMC** only for small source code and configs
3. **Create symlinks** if original path expectations exist
4. **Document paths** in code comments
5. **Verify disk space** before large operations

---

## Task-Specific Guides (Load When Needed)

**Development & Testing:**  
→ READ [`AGENT_DEV_WORKFLOW.md`](AGENT_DEV_WORKFLOW.md)

**Finalization & Deployment:**  
→ READ [`000_AGENT_FINALIZATION_GUIDE.md`](000_AGENT_FINALIZATION_GUIDE.md)

**Learning Mode Active:**  
→ READ [`AGENT_TUTOR_MODE.md`](AGENT_TUTOR_MODE.md)

---

## Reference Documentation Index

**Quick lookup for detailed information:**

| Topic | Document |
|-------|----------|
| **UX Specification** | [`000_UX_AND_FUNCTIONS.md`](000_UX_AND_FUNCTIONS.md) |
| **System Architecture** | [`001_ARCHITECTURE_OVERVIEW.md`](001_ARCHITECTURE_OVERVIEW.md) |
| **Hardware Reference** | [`002_HARDWARE_REFERENCE.md`](002_HARDWARE_REFERENCE.md) |
| **GPU Acceleration** | [`002_HARDWARE_GPU_ACCELERATION.md`](002_HARDWARE_GPU_ACCELERATION.md) |
| **Systemd Services** | [`005_SYSTEMD_SERVICES_REFERENCE.md`](005_SYSTEMD_SERVICES_REFERENCE.md) |
| **Monitoring** | [`006_SYSTEM_STATUS_AND_MONITORING.md`](006_SYSTEM_STATUS_AND_MONITORING.md) |
| **Perception System** | [`100_PERCEPTION_STATUS_REFERENCE.md`](100_PERCEPTION_STATUS_REFERENCE.md) |
| **Perception Install** | [`101_PERCEPTION_STATUS_INSTALLATION.md`](101_PERCEPTION_STATUS_INSTALLATION.md) |
| **Perception Troubleshooting** | [`103_PERCEPTION_STATUS_TROUBLESHOOTING.md`](103_PERCEPTION_STATUS_TROUBLESHOOTING.md) |
| **Web Dashboard** | [`110_WEB_UI_REFERENCE.md`](110_WEB_UI_REFERENCE.md) |
| **Speech System** | [`200_SPEECH_SYSTEM_REFERENCE.md`](200_SPEECH_SYSTEM_REFERENCE.md) |
| **Speech Install** | [`201_SPEECH_SYSTEM_INSTALLATION.md`](201_SPEECH_SYSTEM_INSTALLATION.md) |
| **Speech Troubleshooting** | [`203_SPEECH_SYSTEM_TROUBLESHOOTING.md`](203_SPEECH_SYSTEM_TROUBLESHOOTING.md) |
| **Person Management** | [`250_PERSON_MANAGEMENT_SYSTEM_REFERENCE.md`](250_PERSON_MANAGEMENT_SYSTEM_REFERENCE.md) |
| **AI Tutor System** | [`300_AI_TUTOR.md`](300_AI_TUTOR.md) |

**For complete documentation index:** See [`README.md`](README.md)

---

## Quick Command Reference

### Environment Setup
```bash
# Standard setup (run in order)
source ~/depthai_env/bin/activate
source ~/.bashrc
source ~/dev/r2d2/ros2_ws/install/setup.bash
export OPENBLAS_CORETYPE=ARMV8
```

### ROS 2 Workspace
```bash
# Build single package
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select <package_name>
source install/setup.bash

# Check topics
ros2 topic list
ros2 topic echo /topic_name -n 20
```

### Service Management
```bash
# Status
sudo systemctl status r2d2-<service>.service

# Restart
sudo systemctl restart r2d2-<service>.service

# View logs
journalctl -u r2d2-<service>.service -f
```

### GPU Acceleration
```bash
# Test GPU availability
sudo /home/severin/r2d2-gpu-run.sh test

# Run with GPU (interactive)
sudo /home/severin/r2d2-gpu-run.sh interactive
```

**For detailed commands:** See task-specific guides above

---

## Documentation Standards

### Single Source of Truth

**Parameter values and configuration:**
- Always reference source: code files or `.yaml` configs
- Never duplicate parameter values in docs
- Use cross-references instead

**Example (GOOD):**
```markdown
> **Source of Truth:** Check [`image_listener.py`](ros2_ws/src/r2d2_perception/r2d2_perception/image_listener.py) line 65 for current default.
```

**Example (BAD):**
```markdown
gesture_frame_skip default is 3
```

### Documentation Hierarchy

```
000-099: Internal, Setup & Infrastructure
100-199: Perception & Status System (Phase 1)
200-299: Speech System (Phase 2)
300-399: AI Tutor & Learning System
400-499: Navigation (Phase 3 - reserved)
500-599: Memory & Personality (Phase 4 - reserved)
```

---

## Important Notes

**Work Directly on Jetson:**
- No SSH needed - you're already on the device
- File operations are local
- Services run locally

**If Ever Unsure:**
- Check actual code/config files (source of truth)
- Reference numbered documentation (001-300 series)
- Ask user for UX-related decisions

---

**End of Core Rules**

