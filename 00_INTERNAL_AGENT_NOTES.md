# Internal Agent Notes for R2D2 Project

**Purpose:** Quick reference guide for AI agents and developers working on R2D2. Documents patterns, quirks, and optimization tips that aren't in the formal documentation.

**Last Updated:** December 7, 2025  
**For:** Current and future Claude/AI agents, developers familiar with ROS 2

---

## ⚠️ CRITICAL RULES (READ FIRST!)

### Rule 1: ALWAYS USE `main` BRANCH
- User only uses `main` branch
- `master` branch is **deleted** and must not be used
- Any commits on `master` will be orphaned and lost
- Before any `git push`: Run `git branch` and verify `* main`

### Rule 2: ALWAYS VERIFY BRANCH BEFORE PUSHING
```bash
# BEFORE every push, run:
git branch        # Must show: * main
git log -n 1      # See the commit you're about to push
git status        # Must show: "nothing to commit, working tree clean"
# THEN push:
git push origin main
```

### Rule 3: NEVER USE `master:main` SYNTAX
❌ WRONG: `git push origin master:main`  
✅ RIGHT: `git push origin main`

### Rule 4: IF YOU ACCIDENTALLY COMMIT TO `master`
Stop and alert user. Do not push. Merge to main and delete master:
```bash
git checkout main
git merge master
git branch -d master
git push origin main
git push origin --delete master
```

---

## Quick Reference

### Environment Setup (Correct Order!)
```bash
# ALWAYS IN THIS ORDER - Order matters on ARM!
source ~/depthai_env/bin/activate      # DepthAI first!
source ~/.bashrc                        # Then bash config
source ~/dev/r2d2/ros2_ws/install/setup.bash  # Finally ROS 2
export OPENBLAS_CORETYPE=ARMV8          # Critical for Jetson ARM (prevents illegal instructions)
```

### Standard Build Pattern
```bash
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select <package_name>  # Standard, not full build
```

### Clean Rebuild (Debugging Tool)
```bash
rm -rf build install log && colcon build --packages-select <package_name>
```
**When to use:** Build cache appears stale, mysterious cmake errors, file not found errors

---

## Platform-Specific Knowledge

### Hardware Fixed Constants
| Item | Value | Notes |
|------|-------|-------|
| Platform | NVIDIA Jetson AGX Orin 64GB | Not x86, ARM architecture |
| OS | Ubuntu 22.04 Jammy | Not Focal, not Jammy LTS |
| ROS 2 Version | Humble | Not Jazzy, not Foxy |
| Camera Model | OAK-D Lite Auto Focus | 1920×1080 @ 30 FPS |
| Camera Serial | 19443010E1D30C7E00 | Specific unit |
| Project Root | ~/dev/r2d2 | NOT /home/user, NOT /opt |
| Python | 3.10.6 | Via system or venv |

### Critical ARM Issue
**Symptom:** `Illegal instruction` error when running Python
**Cause:** OpenBLAS running on wrong CPU architecture
**Fix:** `export OPENBLAS_CORETYPE=ARMV8` BEFORE running any ROS 2 code
**Prevention:** Add to ~/.bashrc or sourcing script

### Path Conventions
```
~/dev/r2d2/                           # Project root
├── ros2_ws/                          # ROS 2 workspace
│   ├── src/                          # Source packages
│   ├── build/                        # Build artifacts (ignore)
│   ├── install/                      # Installed packages
│   └── log/                          # ROS 2 logs
├── tests/camera/                     # Test output location
│   └── perception_debug*.jpg         # Always here
├── 01_R2D2_BASIC_SETUP_AND_FINDINGS.md
├── 02_CAMERA_SETUP_DOCUMENTATION.md
└── 03_PERCEPTION_SETUP_DOCUMENTATION.md
```

---

## Development Workflows

### Standard ROS 2 Workflow
1. Edit code in ~/dev/r2d2/ros2_ws/src/<package>/
2. Build: `colcon build --packages-select <package>`
3. Source: `source install/setup.bash`
4. Test: `ros2 launch <package> <launch_file>.py` or `ros2 run`

### When Something Breaks (Debugging Sequence)
1. **Process stuck?** → `pkill -9 -f ros2 && sleep 2 && retry`
2. **Build cache stale?** → `rm -rf build install log && colcon build`
3. **Topic not found?** → `ros2 topic list` to verify publisher is running
4. **Topic has data but wrong format?** → `ros2 topic echo <topic> -n 1` to inspect
5. **FPS too low?** → Check CPU with `top` or GPU thermal with `tegrastats`

### Testing Patterns
```bash
# Always timeout long tests to prevent hanging
timeout 15 <long_command>

# Process cleanup between tests
pkill -9 -f ros2 && sleep 2

# Capture N samples (not streaming)
ros2 topic echo /topic_name -n 20

# Measure rate across multiple windows
ros2 topic hz /topic_name -w 5
```

---

## Performance Baseline Expectations

| Metric | Expected | When Lower = Problem |
|--------|----------|---------------------|
| Startup time | <2 seconds | Check thermal |
| FPS (image processing) | 12-13 Hz | Check CPU load or resolution |
| Memory (Python node) | ~50 MB baseline | Leak? Check for unbounded lists |
| Brightness value range | Vary 1-3 points | Fluctuating > 5 points = lighting changes |
| Topic publish jitter | 0.006-0.024s std dev | Very stable = healthy system |

### Known Variations
- **First 3-5 seconds:** FPS may be lower during initialization (normal)
- **Brightness on startup:** May be 10-20 points lower until AGC stabilizes
- **Memory climb first minute:** Python module loading, then stable (normal)

---

## Camera & Perception Node Specifics

### OAK-D Lite Output Format
- Topic: `/oak/rgb/image_raw`
- Format: `sensor_msgs/Image` with encoding `bgr8`
- Resolution: 1920×1080 (raw from camera)
- Rate: ~30 FPS (camera hardware)
- Frame ID: `oak_d_rgb_camera_optical_frame`

### Image Processing Pipeline (Current)
```
Input: 1920×1080 BGR @ 30 FPS
  ↓
Downscale: 640×360 (11% of pixel count)
  ↓
Grayscale: Single channel via cv2.cvtColor()
  ↓
Brightness: np.mean(gray_image) → 0-255 value
  ↓
Output: Float32 on /r2d2/perception/brightness @ ~13 Hz
```

### Brightness Value Interpretation
- **0-30:** Very dark (night, shadowed)
- **60-90:** Dark indoor
- **125-140:** Well-lit indoor (typical office)
- **150-200:** Very bright (outdoor, bright lamps)
- **230-255:** Overexposed (bright sunlight, reflective surfaces)

**Current test environment:** 132-136 (well-lit room)

### Debug Frame Locations
```
/home/severin/dev/r2d2/tests/camera/perception_debug.jpg       # RGB, 1920×1080, ~470 KB
/home/severin/dev/r2d2/tests/camera/perception_debug_gray.jpg  # Grayscale, 640×360, ~30 KB
```

---

## Git Workflow (Current Setup)

### ⚠️ CRITICAL: Branch Configuration
```
Local:   main
Remote:  origin/main  (single source of truth)
Tracking: main → origin/main

❌ DO NOT USE: master branch (deleted - user only uses main)
❌ DO NOT PUSH: git push origin master:main (wrong branch!)
✅ DO USE: git push origin main (correct)
```

**Why this matters:** User only uses `main` branch. Commits on `master` will be orphaned and user won't see them. This happened multiple times - FIX: Always work on `main`.

### Standard Flow (CORRECT)
```bash
git add <files>
git commit -m "Description with WHY and implementation details"
git push origin main
```

**Check before any git push:**
```bash
git branch -a           # Verify you're on 'main'
git log --oneline -3    # See last 3 commits
git status              # Clean working tree?
```

### Commit Message Pattern
Include:
- What changed
- Why it matters
- Measured results (if applicable)
- Learning points (if novel)

Example:
```
Add brightness behavior validation test results

- Document live hardware test execution (December 5, 2025)
- Publishing rate: 12.8 Hz average (12.5-13.5 Hz range)
- Brightness values: 132-136 range (0-255 scale), mean 134.1
- Grayscale debug frame: 640×360, 29.9 KB, successfully verified
- Learning: Image processing pipeline confirmed working correctly
```

---

## Common Issues & Solutions

### Issue: "topic [X] does not appear to be published yet"
**Likely cause:** Publisher node not running or not subscribed yet  
**Check:**
```bash
ros2 node list                    # Is publisher running?
ros2 topic list                   # Does topic exist?
ros2 topic info <topic>           # How many subscribers?
```
**Fix:** Start the publisher node first, give it 1-2 seconds to initialize

### Issue: "Illegal instruction (core dumped)"
**Likely cause:** OpenBLAS on wrong ARM architecture  
**Fix:**
```bash
export OPENBLAS_CORETYPE=ARMV8
# Then run again
```

### Issue: Build cache seems stale (CMakeError or file not found)
**Standard fix:**
```bash
rm -rf build install log
colcon build --packages-select <package>
```
**When to try:** After renaming files, after .gitignore changes, after package.xml edits

### Issue: "permission denied" on file write
**Check:** Path exists and is writable
```bash
ls -la /home/severin/dev/r2d2/tests/camera/
touch /home/severin/dev/r2d2/tests/camera/test.txt  # Can you write?
```

### Issue: ROS 2 commands seem hung or don't work
**Quick fix:**
```bash
pkill -9 -f ros2
sleep 2
# Retry your command
```

---

## Documentation Standards for This Project

### Format Conventions
- Use markdown with clear hierarchy (# ## ### ####)
- Tables for specifications, commands, results
- Code blocks with language tags (bash, python, cmake)
- Bold for emphasis, links for cross-references
- Include actual measured values, not theoretical

### What Gets Documented
✅ **Document:**
- How to set up and run (in 01, 02, 03)
- What was tested and results (in 03 Comprehensive Testing)
- Code patterns and "why" decisions
- Actual error messages encountered
- Solutions that worked (anti-patterns too!)

❌ **Don't document:**
- Generic ROS 2 info (link to official docs instead)
- Standard Linux commands (assume reader knows)
- Third-party library internals (document only OUR usage)

---

## For Future AI Agents Working Here

### I (Claude) Know From Context
✅ You'll know from reading 01, 02, 03:
- Hardware setup
- Camera integration
- Perception pipeline

### I (Claude) Learn From Working With Severin
✅ Through interaction, I learn:
- Debugging patterns that work
- Performance baselines
- When to try clean rebuild vs incremental
- File naming conventions
- Commit message style
- Communication preferences

### What This File Adds
✅ Quick reference without reading all 3 docs:
- "What's the env sourcing order?" → See top
- "When should I clean rebuild?" → See Debugging Sequence
- "What's a good brightness value?" → See Brightness Interpretation
- "How do I know FPS is healthy?" → See Performance Baseline

---

## Evolution Log

### Changes Made (This Session - Dec 5, 2025)
- ✅ Fixed git master/main branch confusion
- ✅ Renamed docs with 01_, 02_, 03_ prefixes
- ✅ Added documentation links to README.md
- ✅ Executed brightness behavior validation tests (12.8 Hz, 132-136 brightness)
- ✅ Documented complete perception pipeline with image processing
- ✅ Created this internal notes file for future reference

### Known Limitations / TODOs
- [ ] Depth stream integration not yet implemented
- [ ] Bright/dark scene extremes not fully tested (only well-lit indoor)
- [ ] Edge detection not added
- [ ] Only single camera (no fusion)

---

## Quick Command Reference

```bash
# Setup (do once per terminal session)
source ~/depthai_env/bin/activate && source ~/.bashrc && cd ~/dev/r2d2/ros2_ws && source install/setup.bash

# Build
colcon build --packages-select r2d2_perception

# Clean rebuild (debugging)
rm -rf build install log && colcon build --packages-select r2d2_perception

# Test camera + perception
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py

# Test camera + perception with grayscale debug
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py save_debug_gray_frame:=true

# Monitor brightness
ros2 topic echo /r2d2/perception/brightness -n 20

# Check rate
ros2 topic hz /r2d2/perception/brightness -w 5

# Cleanup if stuck
pkill -9 -f ros2 && sleep 2
```

---

**This document is a living reference. Update it when patterns change, new quirks are discovered, or best practices evolve.**
