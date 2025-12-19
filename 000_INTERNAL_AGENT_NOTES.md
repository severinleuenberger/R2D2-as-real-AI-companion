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

### What Goes in _TEMP/
✅ Agent working notes (temporary)  
✅ Draft analysis (work-in-progress)  
✅ Command output captures  
✅ Multi-step work tracking  
✅ (Delete when done - never commit!)

### Documentation Workflow
1. Work in `_TEMP/` for drafts and working notes
2. Extract key findings to main docs (000-060)
3. Link detailed analysis in `_ANALYSIS_AND_DOCUMENTATION/`
4. Delete `_TEMP/` files when complete
5. Commit only permanent content to git

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

## For Future Agents Reading This

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

**This document is a living reference. Update when patterns change or new tools/patterns emerge.**

**Last Updated:** December 16, 2025
