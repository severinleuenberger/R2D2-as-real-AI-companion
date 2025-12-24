# Internal Agent Notes for R2D2 Project

**Purpose:** Quick reference guide for AI agents working on this R2D2 project. This is FOR AGENTS, not for end users. Users read the 0XX-numbered docs; agents use THIS file to understand context, architecture, and how to work efficiently.

**Audience:** AI agents, developers who need quick context

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

## 🖥️ Execution Environment (READ THIS FIRST!)

### WHERE ARE WE?

**YOU ARE EXECUTING DIRECTLY ON THE JETSON AGX ORIN**

- ✅ All commands run **ON the Jetson itself** (not via SSH)
- ✅ The Jetson is **BOTH** the development machine AND deployment target
- ✅ Terminal commands execute in the local Jetson shell
- ✅ `/home/severin/dev/r2d2` is the local workspace on Jetson storage
- ✅ All systemd services run locally on the Jetson

### What This Means For You

**File paths are local:**
- `~/dev/r2d2/` = `/home/severin/dev/r2d2/` on Jetson's NVMe SSD
- No need for `scp`, `ssh`, or remote file transfer
- Direct file access via standard file operations

**Services are local:**
- `systemctl` manages services on this Jetson
- `ros2` commands execute on this Jetson
- `tegrastats` shows THIS machine's stats

**You have full access:**
- Root privileges available via `sudo`
- Direct hardware access (camera, GPIO, I2S audio)
- Real-time sensor data from local devices

### Remote Access Context (For Understanding Only)

**Tailscale VPN is configured but NOT used by agents:**
- Allows Windows laptop to access Jetson for monitoring
- Used for web dashboard access from remote location
- **Agents do NOT connect via Tailscale** - you're already on the Jetson
- Think of it like a security camera - allows viewing, but camera runs independently

**The Jetson operates autonomously:**
- All ROS 2 nodes run locally
- Speech processing happens on local GPU
- Face recognition uses local camera
- No cloud dependency for core functions (except OpenAI API for speech)

### Quick Environment Check

If you're ever unsure, these commands confirm you're on the Jetson:

```bash
# Check hostname (should show: r2d2)
hostname

# Check architecture (should show: aarch64)
uname -m

# Check NVIDIA GPU (should show: Orin)
tegrastats | head -n 1

# Check project location
pwd  # Should be somewhere under /home/severin/
```

---

## Quick Command Reference

### Environment Setup (CORRECT ORDER MATTERS!)
```bash
# ALWAYS IN THIS ORDER - Order matters on ARM!
source ~/depthai_env/bin/activate      # DepthAI first!
source ~/.bashrc                        # Then bash config
source ~/dev/r2d2/ros2_ws/install/setup.bash  # Finally ROS 2
export OPENBLAS_CORETYPE=ARMV8          # Critical for Jetson ARM
```

### Build & Run
```bash
# Standard build (one package)
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select <package_name>

# Clean rebuild (when cache is stale)
rm -rf build install log && colcon build --packages-select <package_name>

# Launch audio notifications
ros2 launch r2d2_audio audio_notification.launch.py

# Launch camera perception
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py

# Check ROS 2 topics
ros2 topic list
ros2 topic echo /topic_name -n 20
ros2 topic hz /topic_name -w 5

# Systemd services
sudo systemctl status r2d2-audio-notification.service
sudo systemctl restart r2d2-audio-notification.service
journalctl -u r2d2-audio-notification.service -f
```

### Debug & Troubleshooting
```bash
# Process stuck?
pkill -9 -f ros2 && sleep 2

# Inspect topic data
ros2 topic echo <topic> -n 1

# Check performance
top                    # CPU/memory
tegrastats             # Jetson thermal/power
```

---

## System Architecture

### Software Stack
```
ROS 2 Humble
    ├── r2d2_audio (audio notifications)
    ├── r2d2_perception (image processing)
    └── r2d2_bringup (camera + perception launch)
         ├── DepthAI Python SDK (OAK-D camera control)
         ├── OpenCV (image processing)
         └── NumPy/SciPy (numerical computing)

Linux ALSA
    └── PAM8403 Amplifier → Speaker
```

### ROS 2 Node Layout
```
r2d2_camera_perception (launches on boot)
  ├── OAK-D Camera → /oak/rgb/image_raw (30 Hz)
  ├── Perception Node → /r2d2/perception/brightness (13 Hz)
  └── Face Recognition Node → /r2d2/perception/person_id (varies)

r2d2-audio-notification.service (systemd service)
  ├── Subscribes: /r2d2/perception/person_id
  ├── Plays: Audio files via ffplay
  └── Publishes: /r2d2/audio/notification_event
```

### Package Structure
```
ros2_ws/src/
├── r2d2_audio/
│   ├── audio_notification_node.py (subscribes to person_id, plays audio)
│   ├── audio_player.py (MP3 playback utility)
│   ├── assets/audio/ (MP3 files: Voicy_R2-D2 - 2.mp3, 5.mp3)
│   └── launch/audio_notification.launch.py
│
├── r2d2_perception/
│   ├── image_listener.py (face recognition, brightness analysis)
│   └── launch/perception.launch.py
│
└── r2d2_bringup/
    └── launch/r2d2_camera_perception.launch.py (main launch file)
```

---

## Platform & Hardware Reference

### Hardware Fixed Constants
| Item | Value | Notes |
|------|-------|-------|
| Platform | NVIDIA Jetson AGX Orin 64GB | ARM64, not x86 |
| OS | Ubuntu 22.04 Jammy | Jetson-specific image |
| ROS 2 | Humble | Required version |
| Camera | OAK-D Lite AF | 1920×1080 @ 30 FPS |
| Camera Serial | 19443010E1D30C7E00 | Specific unit |
| Audio Output | Jetson J511 Pin 9 (HPO_L) via I2S | PAM8403 → 8Ω speaker |
| Python | 3.10.6 | System or venv |
| Project Root | ~/dev/r2d2 | NOT /opt, NOT /home/user |

### Critical ARM Issue
**Symptom:** `Illegal instruction (core dumped)` when running Python  
**Cause:** OpenBLAS optimized for wrong CPU architecture  
**Fix:** `export OPENBLAS_CORETYPE=ARMV8` BEFORE any ROS 2 commands  
**Prevention:** Add to ~/.bashrc

### Project Directory Structure
```
~/dev/r2d2/
├── ros2_ws/                                # ROS 2 workspace
│   ├── src/r2d2_*/                        # Source packages
│   ├── build/, install/, log/             # Build artifacts
│   └── install/setup.bash                 # Source for ROS 2 env
│
├── tests/camera/                          # Test outputs
│   ├── perception_debug.jpg               # RGB capture (1920×1080)
│   └── perception_debug_gray.jpg          # Grayscale (640×360)
│
├── 000_INTERNAL_AGENT_NOTES.md            # This file
├── 001_ARCHITECTURE_OVERVIEW.md           # System design
├── README.md                              # Main overview
└── _ANALYSIS_AND_DOCUMENTATION/           # Analysis & reference
    └── (detailed diagnostic files)
```

---

## Documentation Standards

### What Gets Documented (In 0XX_ to 999_ Files)
✅ All the important main Documentations!
✅ User-facing setup and configuration  
✅ How to run and test the system  
✅ What each parameter does  
✅ Configuration examples  
✅ Troubleshooting guide  

### What Goes in _ANALYSIS_AND_DOCUMENTATION/
✅ Detailed analysis and diagnostics  
✅ Cost breakdowns and research  
✅ Technical deep-dives  
✅ Historical notes and process documents  
✅ (Link these from main docs!)

### What Goes in _TEMP/ (Temporary Scratch Folder - NOT Tracked)
✅ Session logs and build reports  
✅ Debugging artifacts and verification reports  
✅ Draft analysis (work-in-progress)  
✅ Agent working notes during sessions  
✅ **These files are NEVER committed to git (folder is in .gitignore)**  
✅ **AGENTS: Ignore any files in _TEMP/ when reading context or instructions**

### Temporary File Management (Industry Best Practice)
- **_TEMP/ folder:** For temporary scratch work (gitignored)
- **Root-level TEMP_*.md:** Also gitignored (legacy cleanup)
- **Permanent docs:** Use numbered 000-999 files or _ANALYSIS_AND_DOCUMENTATION/

### Documentation Workflow
1. Work in _TEMP/ for session logs, debug reports, build verification
2. Extract key findings to main docs (000-999 numbered files)
3. Move detailed analysis to _ANALYSIS_AND_DOCUMENTATION/ for permanent storage
4. Clean up _TEMP/ files when session is complete (auto-ignored anyway)
5. Commit only permanent content to git

**Rule for Agents:** When reading project context or looking for instructions:
- ✅ READ: Main docs (000-999), _ANALYSIS_AND_DOCUMENTATION/
- ❌ IGNORE: _TEMP/ - these are session artifacts, not source of truth

---

## Documentation Hierarchy & File Organization (UPDATED DEC 2025)

### Documentation Structure Overview

```
/home/severin/dev/r2d2/
├── 000-099: Internal, Setup & Infrastructure
│   ├── 000_INTERNAL_AGENT_NOTES.md (this file - for AI agents)
│   ├── 001_ARCHITECTURE_OVERVIEW.md (complete system architecture)
│   ├── 002_HARDWARE_REFERENCE.md (all hardware specifications)
│   ├── 003-010: Jetson setup, backup/restore, system config
│   └── ...
├── 100-199: Perception & Status System (Phase 1)
│   ├── 100_PERCEPTION_STATUS_REFERENCE.md (complete technical reference)
│   ├── 101-103: Installation, quick start, troubleshooting
│   ├── 110-111: Web dashboard architecture and documentation
│   └── ...
├── 200-299: Speech System (Phase 2)
│   ├── 200_SPEECH_SYSTEM_REFERENCE.md (complete technical reference)
│   ├── 201-204: Installation, quick start, troubleshooting, customization
│   └── ...
├── 300-399: Gesture System (Phase 2)
│   ├── 300_GESTURE_SYSTEM_OVERVIEW.md (system architecture)
│   ├── 303_GESTURE_TRAINING_GUIDE.md (user training guide)
│   └── ...
├── 400-499: Reserved for Navigation System (Phase 3)
├── 500-599: Reserved for Memory & Personality (Phase 4)
├── README.md (project overview - main entry point)
├── LICENSE
└── r2d2_power_button_simple.py (systemd service script)

_ANALYSIS_AND_DOCUMENTATION/  (≤15 supplementary docs only)
├── README.md (index of supplementary docs - REQUIRED)
├── QUICK_START.md
├── AUDIO_QUICK_REFERENCE.md
├── COMPUTE_COST_ANALYSIS.md
├── INTEGRATION_GUIDE.md
├── OPERATIONS_CHECKLIST.md
└── QUICK_ANSWERS.md

_ARCHIVE/  (Historical documents - NOT for active reference)
├── analysis_history_2025-12-24/ (60+ old Phase 1-2 analysis docs)
├── misc_files_2025-12-24/ (debug images, audio samples)
└── (future dated folders...)

_TEMP/  (Transient work only - gitignored)
├── cleanup_audit.md
├── CONTENT_EXTRACTION_PLAN.md
└── (temporary analysis - DELETE when task complete)

scripts/  (Organized by function)
├── deprecated/phase1-development/ (old fix/install scripts)
├── install/ (installation and setup scripts)
├── start/ (launch scripts for services)
├── test/ (testing scripts)
└── util/ (monitoring and utility scripts)

docs/  (Supporting materials)
├── photos/ (hardware photos for README)
└── tools/ (utility documents like VIEW_DIAGRAMS.md)

ros2_ws/  (ROS 2 workspace - standard structure)
├── src/ (source packages)
├── build/ (generated - gitignored)
├── install/ (generated - gitignored)
└── log/ (generated - gitignored)

data/  (Runtime data and models)
├── persons.db (Person Registry database)
├── face_recognition/models/ (trained LBPH models)
├── gesture_recognition/models/ (trained SVM classifiers)
└── conversations/ (speech conversation logs)
```

### Documentation Rules & Best Practices

#### Numbered Files (000-999) - SINGLE SOURCE OF TRUTH

**Purpose:** All authoritative, permanent documentation

**Rules:**
- ✅ **Primary reference** - All main documentation lives here
- ✅ **Persistent** - Never delete, only update or mark deprecated
- ✅ **Version controlled** - All changes tracked in git
- ✅ **Cross-referenced** - Link only to other numbered docs or README
- ❌ **Do NOT reference** `_ANALYSIS_AND_DOCUMENTATION/` from numbered docs

**Naming Convention:**
- `000-099`: Internal guides, setup, infrastructure
- `100-199`: Phase 1 (Perception & Status)
- `200-299`: Phase 2 (Speech & Conversation)
- `300-399`: Phase 2 (Gesture Recognition)
- `400-499`: Phase 3 (Navigation - reserved)
- `500-599`: Phase 4 (Memory & Personality - reserved)

#### _ANALYSIS_AND_DOCUMENTATION/ - SUPPLEMENTARY ONLY

**Purpose:** Quick references, checklists, FAQs (NOT primary docs)

**Rules:**
- ✅ **Max 15 files** - Keep lean and focused
- ✅ **README.md required** - Must index all files in folder
- ✅ **Quick reference only** - Cheat sheets, checklists, summaries
- ❌ **Not primary source** - Numbered docs (000-999) are authoritative
- ❌ **Do NOT extract from here** - These supplement, not replace main docs

**Allowed Types:**
- Quick start guides (for experienced users)
- Command cheat sheets (audio, ROS, systemd)
- Operations checklists (daily procedures)
- FAQ documents (common questions)
- Performance benchmarks (detailed analysis)
- Integration templates (for adding features)

#### _ARCHIVE/ - HISTORICAL ONLY

**Purpose:** Historical documents, old analysis, deprecated code

**Rules:**
- ✅ **Dated folders** - Use `YYYY-MM-DD` format
- ✅ **README per folder** - Document what was archived and why
- ❌ **Never reference** - These are archaeology, not current work
- ❌ **Never active use** - If needed again, copy to correct location

**Archive Types:**
- Implementation history (completed feature docs)
- Old analysis documents (superseded by numbered docs)
- Debug/diagnostic files (outdated troubleshooting)
- Deprecated scripts (replaced by systemd services)

#### _TEMP/ - TRANSIENT WORK ONLY

**Purpose:** Work-in-progress, scratch notes, session artifacts

**Rules:**
- ✅ **Gitignored** - Never committed to repository
- ✅ **Delete when done** - Clean up after task completion
- ❌ **Never reference** - These are scratch work, not documentation
- ❌ **Agents ignore** - Do not read for project context

**Usage:**
- Session logs and build reports
- Debugging artifacts during development
- Draft analysis before finalizing
- Cleanup plans and audit reports (temporary)

### File Movement & Cleanup Guidelines

**When to archive a file:**
1. Analysis complete and findings extracted to numbered docs
2. Implementation finished and documented in main reference
3. Feature superseded by newer implementation
4. Diagnostic/troubleshooting no longer relevant

**How to archive:**
```bash
# Create dated folder
mkdir -p _ARCHIVE/description_YYYY-MM-DD/

# Move files
mv <file1> <file2> ... _ARCHIVE/description_YYYY-MM-DD/

# Create README
cat > _ARCHIVE/description_YYYY-MM-DD/README.md << 'EOF'
# Description Archive - YYYY-MM-DD

**Purpose:** <why these were archived>

**Contents:**
- <file1>: <description>
- <file2>: <description>
...

**Extracted to:** <numbered doc references>

**Status:** Archived - historical reference only
EOF
```

**When to delete (not archive):**
- Generated files (build artifacts)
- True temporary files (session logs)
- Duplicate copies of tracked files
- Empty or placeholder files

### Documentation Maintenance Checklist

**Monthly (or after major feature completion):**
- [ ] Review `_ANALYSIS_AND_DOCUMENTATION/` - ensure ≤15 files
- [ ] Check for duplicate information across numbered docs
- [ ] Archive completed implementation/analysis docs
- [ ] Update `_ANALYSIS_AND_DOCUMENTATION/README.md`
- [ ] Clean `_TEMP/` folder (should always be minimal)

**After Each Major Feature:**
- [ ] Extract key findings to numbered docs (000-999)
- [ ] Move implementation notes to `_ARCHIVE/`
- [ ] Update relevant reference docs
- [ ] Delete temporary analysis files

**Before Each Git Commit:**
- [ ] Verify `_TEMP/` is gitignored (should not appear in `git status`)
- [ ] Check no loose files in root (scripts, images, audio)
- [ ] Ensure all numbered docs are properly cross-referenced
- [ ] Update README.md if new numbered docs added

---

## Git Best Practices

### Commit Message Pattern
```
<Type>: <Short summary (50 chars)>

<Body: What changed, why, and measured results>

Example:
---
feat: Add audio volume parameter to audio notification node

- Implement global audio_volume parameter (0.0-1.0)
- Current default: 0.05 (5% - very quiet)
- Tested: Audio plays at 50% volume as expected
- Updated: Both source code and systemd service
- Measured: Service restart successful, no errors
```

### Before Every Push
```bash
git branch -a           # Verify on 'main'
git log --oneline -3    # Check last 3 commits
git status              # Clean working tree?
git push origin main    # Push
```

---

## Service Management & Resource Optimization

### Auto-Start Configuration (Production Deployment)

**Services that auto-start on boot:**
1. ✅ `r2d2-camera-perception.service` (camera + face + gesture recognition)
2. ✅ `r2d2-audio-notification.service` (audio alerts + LED + logger)
3. ✅ `r2d2-gesture-intent.service` (gesture-to-speech control)
4. ✅ `r2d2-heartbeat.service` (system health monitoring)
5. ✅ `tailscaled.service` (VPN access)
6. ⚠️ `r2d2-powerbutton.service` (optional - if physical button installed)

**Services that remain on-demand (manual start):**
1. ❌ `r2d2-rosbridge.service` (web dashboard WebSocket bridge)
2. ❌ `r2d2-web-dashboard.service` (FastAPI REST API + UI)
3. ❌ `r2d2-camera-stream.service` (MJPEG video stream - **device conflict with camera-perception**)
4. ❌ `r2d2-speech.service` (Phase 2 speech system - high cost, infrequent use)

**Resource Usage Summary:**
- **Auto-start services:** 16-26% CPU, ~400 MB RAM (core functionality)
- **All services active:** 26-39% CPU, ~600 MB RAM (includes web dashboard + speech)
- **Available headroom:** 74-84% CPU, ~63.5 GB RAM (for future features)

**⚠️ Critical Rule: camera_stream_node and camera_node are MUTUALLY EXCLUSIVE**
- Both require exclusive camera access (OAK-D Lite via DepthAI SDK)
- NEVER auto-start camera_stream (causes device conflict with core perception)
- camera_stream is ONLY for web dashboard preview (on-demand via API)

### Service Control Commands

**Start/stop web dashboard:**
```bash
# Start (rosbridge + FastAPI)
~/dev/r2d2/scripts/start_web_dashboard.sh

# Stop
~/dev/r2d2/scripts/stop_web_dashboard.sh
```

**Speech system management:**
```bash
# Start (via launcher script)
~/dev/r2d2/launch_ros2_speech.sh

# Check status
ros2 lifecycle get /speech_node

# Stop
ros2 lifecycle set /speech_node deactivate
ros2 lifecycle set /speech_node cleanup
```

**Enable/disable auto-start:**
```bash
# Enable service
sudo systemctl enable <service-name>

# Disable service (but keep installed)
sudo systemctl disable <service-name>

# Check if enabled
systemctl is-enabled <service-name>
```

### Parameter Tuning Guidelines

**Face Recognition (if CPU constrained):**
- Increase `recognition_frame_skip` (default: 2) → 3 or 4 for ~2-3% CPU savings
- Trade-off: Slower recognition response (~150ms slower per frame skip increment)

**Gesture Recognition (if CPU constrained):**
- Increase `gesture_frame_skip` (default: 5) → 7 or 10 for lower CPU usage
- Trade-off: Reduced responsiveness

**Audio Alerts (if silent environment needed):**
- Set `audio_volume:=0.0` to mute all audio alerts
- LED status still works (visual feedback only)

**Speech Auto-Shutdown (cost optimization):**
- Adjust `auto_shutdown_timeout_seconds` (default: 35)
- Longer timeout (60-300s) = fewer API reconnections but higher cost
- Shorter timeout (20-30s) = more aggressive cost savings

**Current settings are already well-optimized. Only adjust if specific needs arise.**

**For detailed optimization analysis, see:** `_ANALYSIS_AND_DOCUMENTATION/006_OPTIMIZATION_ANALYSIS.md` (archived)

---

## For Future Agents Reading This

### Recent Fixes & Important History

**December 24, 2025 - Speech Service Path Fix:**
- **Issue:** Speech system wouldn't start after index finger gesture (Phase 4 → Phase 5 blocked)
- **Root Cause:** `r2d2-speech-node.service` ExecStart pointed to old path `/home/severin/dev/r2d2/start_speech_node.sh`
- **Solution:** Updated to correct path `/home/severin/dev/r2d2/scripts/start/start_speech_node.sh`
- **Context:** Scripts were moved to `scripts/start/` during December 2025 documentation cleanup, but service files were not all updated
- **Status:** Fixed and working. gesture_intent_node now successfully calls start_session service.
- **Remaining:** 4 other services still need path updates (audio-notification, heartbeat, rosbridge, camera-stream)

**December 24, 2025 - Documentation Reorganization:**
- Moved `SYSTEM_STATUS_AND_MONITORING.md` to `006_SYSTEM_STATUS_AND_MONITORING.md` (official 000-999 series)
- Created `005_SYSTEMD_SERVICES_REFERENCE.md` (complete service documentation)
- Updated `001_ARCHITECTURE_OVERVIEW.md` with System Components Map
- Documented `tools/minimal_monitor.py` (comprehensive one-line monitoring tool)
- All changes on branch: `cleanup/docs-2025-12-24`

**December 2025 - Script Organization:**
- All startup scripts moved to `scripts/start/` directory
- Systemd service files need ExecStart path updates to match
- See `005_SYSTEMD_SERVICES_REFERENCE.md` for complete path audit

### What You Need to Know From This File
- Critical git rules (always use main branch)
- Environment setup order (DepthAI → bashrc → ROS2)
- System architecture (nodes, packages, topics)
- How to modify, test, and validate changes
- Troubleshooting common issues
- Documentation management rules

### When You're Done Working
1. Test your changes thoroughly
2. Validate with checklist above
3. Update documentation if needed
4. Clean up `_TEMP/` folder
5. Commit with descriptive message
6. Push to main branch
7. Verify on GitHub

---

## System Monitoring and Diagnostics

### Freeze Monitor System
**Status:** ✅ Active since December 16, 2025

The Jetson experiences periodic freezes (every 30-60 minutes). A comprehensive monitoring system is now in place:

- **Service:** `freeze-monitor.service` (systemd)
- **Logs:** `/var/log/freeze_logs/` (5 separate log files)
- **Interval:** Every 10 seconds
- **Documentation:** `050_FREEZE_MONITOR_SYSTEM.md`

**After a freeze:**
```bash
# Quick analysis
tail -100 /var/log/freeze_logs/summary.log
tail -50 /var/log/freeze_logs/hardware.log
tail -50 /var/log/freeze_logs/kernel.log
```

**Service management:**
```bash
systemctl status freeze-monitor
journalctl -u freeze-monitor -f
```

---

## File Location Rules & Documentation Hierarchy

### System Scripts in /home/severin/ (DO NOT MOVE)

These scripts MUST remain in `/home/severin/` as they are referenced by systemd services and autostart:

**Freeze Monitor System:**
- `/home/severin/freeze_monitor.py` - Main monitor script (freeze-monitor.service)
- `/home/severin/install_freeze_monitor.sh` - Installer
- `/home/severin/apply_3day_retention.sh` - Log retention config
- `/home/severin/apply_14day_retention.sh` - Log retention config

**Display Configuration:**
- `/home/severin/set-lg-resolution.sh` - Autostart display config (referenced in ~/.config/autostart/)
- `/home/severin/set-3440x1440.sh` - Resolution helper

**GPU Utilities:**
- `/home/severin/r2d2-gpu-run.sh` - GPU container helper (documented in 007)
- `/home/severin/test_gpu_speech.sh` - GPU test script

⚠️ **DO NOT MOVE THESE FILES** - They are active system components with hardcoded references.

### Documentation Hierarchy (STRICTLY ENFORCED)

#### Project Root (dev/r2d2/)
```
dev/r2d2/
├── README.md (EXCEPTION - GitHub standard)
├── 000-999_*.md (Authoritative main documentation)
└── docs/ (Supplementary documentation organized by category)
```

#### Numbered Documentation (000-999)
- **Purpose:** Authoritative, high-level documentation
- **Naming:** `NNN_DESCRIPTIVE_NAME.md` (e.g., `005_SYSTEMD_SERVICES_REFERENCE.md`)
- **Location:** Project root only
- **Rules:**
  - Use lowercase with underscores
  - Keep focused and well-organized
  - Reference supplementary docs in `docs/` when needed

#### Supplementary Documentation (docs/)
```
docs/
├── hardware/       # Wiring diagrams, hardware guides
│   └── led_wiring.md
├── troubleshooting/ # Detailed troubleshooting guides
│   ├── tailscale_vpn.md (consolidated from 5 files)
│   └── freeze_monitor.md
├── setup/          # Detailed setup and installation guides
│   └── gpu_acceleration.md (consolidated from 4 files)
├── reference/      # Best practices, reference materials
│   └── jetson_best_practices.md
├── photos/         # Build photos and images
└── tools/          # Tool-specific documentation
```

#### Scripts Organization
```
scripts/
├── start/          # Service startup scripts (referenced by systemd)
├── install/        # Installation scripts
│   ├── freeze_monitor/
│   └── display/
├── util/           # Helper utilities
└── deprecated/     # Archived/obsolete scripts
    └── troubleshooting/
```

#### Other Folders
- **`_ARCHIVE/`** - Historical documentation, archived files (git-tracked)
- **`_TEMP/`** - Work-in-progress, scratch files (gitignored)
- **`_ANALYSIS_AND_DOCUMENTATION/`** - Quick reference docs (max 15 files)
- **`tools/`** - Active monitoring and utility scripts
- **`ros2_ws/`** - ROS 2 workspace (standard structure)
- **`data/`** - Training data, models

### Rules for AI Agents

#### BEFORE Creating Any .md File:
1. Is it authoritative documentation? → `NNN_NAME.md` in project root
2. Is it hardware/wiring? → `docs/hardware/`
3. Is it troubleshooting? → `docs/troubleshooting/`
4. Is it setup/installation? → `docs/setup/`
5. Is it reference/best practices? → `docs/reference/`
6. Is it implementation history? → `_ARCHIVE/`
7. Is it work-in-progress? → `_TEMP/` (gitignored)

#### BEFORE Creating Any Script:
1. Does it start a service? → `scripts/start/`
2. Is it for installation/setup? → `scripts/install/{category}/`
3. Is it a utility/helper? → `scripts/util/`
4. Is it obsolete/debug only? → `scripts/deprecated/`

#### Consolidation Rules:
- ✅ Consolidate redundant docs (e.g., 4 GPU docs → 1)
- ✅ Use consistent naming (lowercase_underscore.md)
- ✅ Organize by function in subdirectories
- ❌ Never create loose files in `/home/severin/`
- ❌ Never create multiple docs for same topic
- ❌ Never skip proper folder organization

### Verification Checklist

Before committing documentation changes:
- [ ] No project files in `/home/severin/` (except system scripts above)
- [ ] All numbered docs follow `NNN_NAME.md` pattern
- [ ] Supplementary docs organized in `docs/{category}/`
- [ ] Scripts organized in `scripts/{function}/`
- [ ] No redundant documentation (consolidated if needed)
- [ ] Cross-references updated and valid
- [ ] README.md reflects current state

---

**This document is a living reference. Update when patterns change or new tools/patterns emerge.**

**Last Updated:** December 24, 2025 - Added file location rules and documentation hierarchy
