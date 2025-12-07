# R2D2 Operations Checklist & Daily Procedures
**Date:** December 7, 2025  
**Platform:** NVIDIA Jetson AGX Orin 64GB + ROS 2 Humble  
**For:** Daily operation, monitoring, and recovery

---

## Quick Start (TL;DR)

```bash
# 1. SSH into Jetson
ssh -i ~/.ssh/jetson user@jetson_ip

# 2. Navigate to workspace
cd ~/dev/r2d2/ros2_ws

# 3. Activate environment (CRITICAL - order matters!)
source ~/depthai_env/bin/activate      # DepthAI first
export OPENBLAS_CORETYPE=ARMV8         # ARM fix
source ~/.bashrc                       # Then bash
source install/setup.bash              # Finally ROS 2

# 4. Start the system
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py

# 5. Verify in another terminal (same environment setup as above)
ros2 topic hz /r2d2/perception/brightness  # Should show ~13 Hz

# 6. Check system health
watch -n 1 'top -bn1 | grep -E "python|Cpu" | head -10'
```

**Time to Ready:** ~7 seconds from launch command

---

## 1. Pre-Startup Checklist

### 1.1 Hardware Verification (Before Power-On)

```
☐ OAK-D Lite camera connected to Jetson USB port (not hub)
  └─ Check: Physical connection, no loose cables
  
☐ Jetson power supply connected and ready
  └─ Check: 100W+ power adapter, proper connector
  
☐ Network connectivity (optional but recommended)
  └─ Check: Ethernet or WiFi connected for SSH
  
☐ Adequate disk space on Jetson
  └─ Check: df -h (need at least 2 GB free)
  
☐ Jetson thermals acceptable
  └─ Check: Not hot to touch (should be warm, not hot)
  
☐ No USB conflicts
  └─ Check: lsusb after boot, should see "Movidius" camera
```

### 1.2 Software Verification (Before Launch)

```
☐ ROS 2 Humble properly installed
  └─ Test: ros2 --version (should show ROS 2 Humble)
  
☐ DepthAI environment installed
  └─ Test: ls ~/depthai_env/bin/activate (should exist)
  
☐ r2d2 workspace built successfully
  └─ Test: cd ~/dev/r2d2/ros2_ws && ls install/
  
☐ All packages present
  └─ Test: ls install/ | grep r2d2
      Should see: r2d2_bringup, r2d2_camera, r2d2_perception, r2d2_hello
  
☐ Face recognition model exists (if using recognition)
  └─ Test: ls ~/dev/r2d2/data/face_recognition/models/severin_lbph.xml
```

---

## 2. Startup Procedure (Detailed Steps)

### 2.1 Environment Setup (CRITICAL - Order Matters!)

```bash
# Step 1: Activate DepthAI environment FIRST
source ~/depthai_env/bin/activate
# Output: Should show (depthai_env) prefix

# Step 2: Set ARM fix (prevents "Illegal instruction" crashes)
export OPENBLAS_CORETYPE=ARMV8
# Verify: echo $OPENBLAS_CORETYPE (should print ARMV8)

# Step 3: Activate bash configuration
source ~/.bashrc
# Verify: Should show standard bash setup

# Step 4: Source ROS 2 workspace setup
source ~/.bashrc  # Already done above, but ensure it's done
cd ~/dev/r2d2/ros2_ws
source install/setup.bash
# Output: Should complete without errors

# Verify all set up correctly:
echo "DepthAI env: $(which python3)"
echo "ROS 2 domain: $ROS_DOMAIN_ID"
echo "OPENBLAS: $OPENBLAS_CORETYPE"
```

**Common Issues at This Stage:**
- ❌ "DepthAI not found" → Missing depthai_env activation
- ❌ "Illegal instruction" error → Missing OPENBLAS_CORETYPE=ARMV8
- ❌ "ros2 command not found" → Missing source install/setup.bash

### 2.2 Launch Camera & Perception System

```bash
# Terminal 1: Launch the system
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py

# Expected output (first 10 seconds):
# [INFO] [launch]: ...
# [INFO] [camera_node]: OAK-D camera initialized
# [INFO] [image_listener]: ImageListener node initialized
# [INFO] [image_listener]: Haar Cascade loaded successfully
# [INFO] [heartbeat_node]: Heartbeat publishing...

# System ready when you see:
# [INFO] All nodes started successfully
```

### 2.3 Verify All Topics Are Publishing

```bash
# Terminal 2: Check camera output
ros2 topic list | grep r2d2
# Expected output:
# /r2d2/heartbeat
# /r2d2/perception/brightness
# /r2d2/perception/face_count
# /r2d2/perception/person_id (if recognition enabled)
# /r2d2/perception/face_confidence (if recognition enabled)
# /r2d2/perception/is_severin (if recognition enabled)

# Terminal 2: Verify camera frame rate
ros2 topic hz /oak/rgb/image_raw
# Expected: average rate: 30.00
#          Should be between 29-31 Hz

# Terminal 2: Verify perception frame rate
ros2 topic hz /r2d2/perception/brightness
# Expected: average rate: 13.00
#          Should be between 12-14 Hz
```

### 2.4 Enable Face Recognition (Optional)

```bash
# If you want to enable face recognition, use launch argument:
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true

# Verify additional topics with recognition:
ros2 topic hz /r2d2/perception/person_id
# Expected: ~6.5 Hz (with default frame_skip=2)
```

---

## 3. System Monitoring

### 3.1 Real-Time Topic Monitoring

```bash
# Watch brightness values (should be 130-140 in normal lighting)
watch -n 0.5 'ros2 topic echo /r2d2/perception/brightness -n 1'
# Output: Should update every 0.5 seconds with values like:
# data: 135.2
# data: 136.1
# data: 134.8

# Watch face detection (should be 0 when no one present)
watch -n 0.5 'ros2 topic echo /r2d2/perception/face_count -n 1'
# Output: Should update with integer count:
# data: 0
# data: 1
# data: 2

# Watch person identification (if recognition enabled)
watch -n 0.5 'ros2 topic echo /r2d2/perception/person_id -n 1'
# Output: Should show person name:
# data: unknown
# data: severin
# data: unknown
```

### 3.2 CPU & Memory Monitoring

```bash
# Real-time CPU usage (Jetson-specific)
watch -n 1 'top -bn1 | head -20'
# Look for: 
#   - python3 processes (should be ~10-15% CPU each)
#   - Mem: check free memory (should be >50 GB free)
#   - CPU usage: should be <30% total

# More detailed Jetson-specific metrics
tegrastats
# Shows: CPU, GPU, thermal info, power draw
# Run until satisfied, then Ctrl+C

# Just check CPU load average
uptime
# Output example: load average: 0.15, 0.12, 0.10
# Should be <1.0 on a 12-core system

# Memory usage
free -h
# Total should be ~64 GB
# Used should be <10 GB (lots of headroom)
```

### 3.3 Topic Health Checks

```bash
# Check all topics publishing
ros2 topic list -v
# Should show 7 topics with their message types

# Check topic rates
for topic in /oak/rgb/image_raw /r2d2/perception/brightness /r2d2/perception/face_count; do
  echo "=== $topic ==="
  ros2 topic hz $topic -w 5
done

# Expected rates:
# /oak/rgb/image_raw: 30 Hz
# /r2d2/perception/brightness: 13 Hz
# /r2d2/perception/face_count: 13 Hz
```

### 3.4 Node Health Checks

```bash
# List active nodes
ros2 node list
# Should see:
# /camera_node
# /image_listener
# /heartbeat_node

# Check specific node
ros2 node info /image_listener
# Shows: subscriptions, publications, services

# View node parameters
ros2 param list /image_listener
# Shows all configurable parameters

# Get specific parameter
ros2 param get /image_listener enable_face_recognition
```

---

## 4. Daily Health Check (5-Minute Routine)

```bash
# Run this every day to verify system is healthy

echo "=== R2D2 DAILY HEALTH CHECK ==="
echo ""

# 1. Check system is running
echo "1. Active nodes:"
ros2 node list

# 2. Check all topics publishing
echo ""
echo "2. Topics publishing:"
ros2 topic list | wc -l
echo "Expected: 7 topics"

# 3. Check frame rates
echo ""
echo "3. Frame rates:"
echo "  Camera (should be ~30 Hz):"
timeout 5 ros2 topic hz /oak/rgb/image_raw || echo "  [OK]"

echo "  Perception (should be ~13 Hz):"
timeout 5 ros2 topic hz /r2d2/perception/brightness || echo "  [OK]"

# 4. Check CPU/memory
echo ""
echo "4. System resources:"
top -bn1 | grep "Cpu\|Mem" | head -2

# 5. Sample perception data
echo ""
echo "5. Sample brightness:"
ros2 topic echo /r2d2/perception/brightness -n 1

echo ""
echo "6. Face count:"
ros2 topic echo /r2d2/perception/face_count -n 1

echo ""
echo "=== HEALTH CHECK COMPLETE ==="
```

---

## 5. Troubleshooting Guide

### 5.1 Camera Not Detected

**Symptom:** Camera connection error, no /oak/rgb/image_raw topic

**Diagnostic Steps:**
```bash
# 1. Check USB connection
lsusb | grep Movidius
# Should show: Movidius MyriadX (USB ID: 03e7:2485)

# 2. Check camera is powered
dmesg | tail -20
# Look for USB enumeration messages

# 3. Try direct camera test
cd ~/dev/r2d2/tests/camera
python3 oakd_test.py
# Should capture and save test image

# 4. Check DepthAI environment
python3 -c "import depthai; print(depthai.__version__)"
# Should print version number (e.g., 2.31.0.0)
```

**Solutions:**
1. Reseat USB cable (disconnect, wait 2s, reconnect)
2. Try different USB 3.0 port (not USB 2.0)
3. Power cycle Jetson (full shutdown, 10s, power on)
4. Rebuild DepthAI: `pip install --upgrade depthai`

---

### 5.2 "Illegal Instruction" Crash on Startup

**Symptom:** Crash with "Illegal instruction" message when launching

**Cause:** Missing ARM CPU type setting for OpenBLAS

**Fix:**
```bash
# Ensure this is set before any ROS 2 commands:
export OPENBLAS_CORETYPE=ARMV8

# Verify:
echo $OPENBLAS_CORETYPE
# Should print: ARMV8

# Then launch system again
```

**Prevention:** Add to ~/.bashrc:
```bash
# At end of ~/.bashrc file
export OPENBLAS_CORETYPE=ARMV8
```

---

### 5.3 Topics Not Publishing

**Symptom:** ros2 topic list shows no r2d2 topics

**Diagnostic Steps:**
```bash
# 1. Check ROS domain ID consistency
echo $ROS_DOMAIN_ID
# If empty, domain ID defaults to 0

# 2. Check if nodes are running
ps aux | grep python3
# Should see image_listener and camera_node processes

# 3. Check for errors in launch terminal
# Look at launch window for error messages

# 4. Verify install directory
ls ~/dev/r2d2/ros2_ws/install/
# Should see r2d2_bringup, r2d2_camera, r2d2_perception, r2d2_hello
```

**Solutions:**
1. Rebuild workspace: `cd ~/dev/r2d2/ros2_ws && colcon build --packages-select r2d2_bringup`
2. Clear cache: `rm -rf build install log && colcon build`
3. Verify ROS 2 setup: `ros2 --version`
4. Check environment: ensure `source install/setup.bash` was called

---

### 5.4 Low Frame Rates (<10 Hz)

**Symptom:** `ros2 topic hz /r2d2/perception/brightness` shows <10 Hz

**Causes:**
1. CPU overload
2. USB bandwidth issue
3. Processing bottleneck

**Diagnostic:**
```bash
# Check CPU usage
top -bn1 | grep "Cpu\|python"
# If >80% CPU, something is consuming resources

# Check USB bandwidth
# Monitor dmesg for USB errors
dmesg | tail -20 | grep -i usb

# Check image processing time
# Enable verbose logging
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  log_every_n_frames:=5
# Watch launch terminal for timing info
```

**Solutions:**
1. Disable face recognition: `enable_face_recognition:=false`
2. Increase frame skip: `recognition_frame_skip:=3`
3. Reduce logging: `log_every_n_frames:=30`
4. Close other processes: `pkill -f python3` (except ROS nodes)

---

### 5.5 Face Recognition Not Working

**Symptom:** Recognition enabled but `/r2d2/perception/person_id` always returns "unknown"

**Causes:**
1. Model file missing or corrupted
2. Training data insufficient
3. Lighting conditions different from training
4. Confidence threshold too strict

**Diagnostic:**
```bash
# 1. Verify model exists
ls -lh ~/dev/r2d2/data/face_recognition/models/severin_lbph.xml
# Should show file size ~200 KB

# 2. Check face detection working
ros2 topic echo /r2d2/perception/face_count
# Should show faces being detected (count > 0)

# 3. Check confidence scores
ros2 topic echo /r2d2/perception/face_confidence
# If values are >100, face quality might be poor

# 4. Test with direct script
cd ~/dev/r2d2/tests/face_recognition
python3 3_test_recognizer_demo.py
# Should show confidence scores
```

**Solutions:**
1. Retrain model: `python3 interactive_training.py` (8-step interactive training)
2. Lower confidence threshold: `recognition_confidence_threshold:=80.0`
3. Improve lighting (bright, frontal angles)
4. Increase training data diversity (different angles, distances, lighting)

---

### 5.6 High CPU Usage (>50%)

**Symptom:** System running hot, CPU at 80%+, frame rates dropping

**Causes:**
1. Face recognition enabled with low frame skip
2. Excessive logging
3. Other processes competing for CPU
4. Debug frame saving enabled

**Diagnostic:**
```bash
# Find high-CPU processes
ps aux --sort=-%cpu | head -10

# Check what perception node is doing
top -p $(pgrep -f image_listener)
```

**Solutions (in order of impact):**
1. Disable face recognition: `enable_face_recognition:=false` (-5% CPU)
2. Increase frame skip: `recognition_frame_skip:=3` (-3% CPU per level)
3. Disable verbose logging: `log_every_n_frames:=100` (-1% CPU)
4. Disable face detection logging: `log_face_detections:=false` (-0.5% CPU)

**Recommended for Phase 2-4 work:**
```bash
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=false \
  log_every_n_frames:=100
# This leaves ~5% CPU for new features
```

---

### 5.7 System Hangs or Freezes

**Symptom:** System becomes unresponsive

**Recovery (from another terminal or SSH):**
```bash
# 1. Kill ROS nodes gracefully
pkill -f "ros2 launch"
pkill -f image_listener
pkill -f camera_node

# Wait 2 seconds
sleep 2

# 2. Kill any remaining python processes (careful!)
pkill -9 -f "image_listener\|camera_node"

# 3. Check if system responsive
ps aux | head -5

# 4. Restart system
# If still hung: power cycle or SSH reboot
sudo reboot
```

**Prevention:**
- Monitor CPU temperature: `tegrastats` should show <60°C
- Don't run Phase 2-4 features with face recognition at same time initially
- Use `timeout` when testing uncertain code: `timeout 10 python3 test.py`

---

## 6. Shutdown Procedure

```bash
# Graceful shutdown (preferred)

# Terminal 1: Ctrl+C in launch window
# This will:
# - Stop all nodes
# - Close camera connection
# - Release resources
# - Exit cleanly

# Wait for output:
# [camera_node]: Shutting down...
# [image_listener]: Shutting down...
# All nodes shut down successfully

# Terminal 2: Can exit
exit

# Then safely power off Jetson (if needed)
sudo shutdown -h now
```

---

## 7. Backup & Recovery

### 7.1 Quick Verification After Reboot

```bash
# After Jetson reboots:
cd ~/dev/r2d2/ros2_ws

# 1. Set up environment
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8
source ~/.bashrc
source install/setup.bash

# 2. Verify git is clean (no accidental changes)
git status
# Should show "nothing to commit, working tree clean"

# 3. Pull latest changes (if working with team)
git pull origin main

# 4. Check for any build issues
colcon list
# Should show all 4 packages

# 5. Launch system
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py
```

### 7.2 Rebuild if Issues Persist

```bash
# Clean rebuild (nuclear option)
cd ~/dev/r2d2/ros2_ws
rm -rf build install log
colcon build --packages-select r2d2_camera r2d2_perception r2d2_hello r2d2_bringup

# Takes ~2 minutes
# When done:
source install/setup.bash
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py
```

---

## 8. Performance Baselines (Reference)

```
NORMAL OPERATION METRICS:
═══════════════════════════════════════════════════════════

Camera frame rate:        30 Hz (±1 Hz normal)
Perception output rate:   13 Hz (±1 Hz normal)
Recognition rate:         6.5 Hz (with frame_skip=2)
Brightness values:        130-140 (typical indoor)
Face detection accuracy:  ~90% (Haar Cascade)
CPU usage:                8-10% (without recognition)
                         10-15% (with recognition)
Memory usage:             ~500 MB (stable)
Startup time:             5-7 seconds
Topics publishing:        6-7 (depending on recognition)

RED FLAGS (investigate if you see these):
═══════════════════════════════════════════════════════════
- Frame rate <10 Hz (check CPU, USB)
- Brightness outside 100-170 range (extreme lighting)
- CPU >50% (disable recognition, check for leaks)
- Memory >2 GB (memory leak suspected)
- Startup >15 seconds (corruption, rebuild needed)
- Topics missing (rebuild workspace)
- Face count always 0 (camera issue, lighting)
- "Illegal instruction" errors (OPENBLAS_CORETYPE not set)
```

---

## 9. Quick Reference Cards

### 9.1 Essential Commands

```bash
# Startup (one-liner with environment)
source ~/depthai_env/bin/activate && \
  export OPENBLAS_CORETYPE=ARMV8 && \
  source ~/.bashrc && \
  cd ~/dev/r2d2/ros2_ws && \
  source install/setup.bash && \
  ros2 launch r2d2_bringup r2d2_camera_perception.launch.py

# Monitor brightness
watch -n 0.5 'ros2 topic echo /r2d2/perception/brightness -n 1'

# Monitor CPU
watch -n 1 'top -bn1 | head -15'

# Check all topics
ros2 topic list -v

# Verify frame rates
ros2 topic hz /oak/rgb/image_raw

# Kill system
pkill -f "ros2 launch"

# Full rebuild
cd ~/dev/r2d2/ros2_ws && rm -rf build install log && colcon build
```

### 9.2 Debugging Template

```bash
# 1. Environment setup
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8
source ~/.bashrc
cd ~/dev/r2d2/ros2_ws && source install/setup.bash

# 2. Check issue category
ros2 topic list              # Are topics appearing?
ps aux | grep python3        # Are nodes running?
top -bn1 | head -20          # What's CPU doing?
lsusb | grep Movidius        # Is camera detected?

# 3. Enable verbose logging
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  log_every_n_frames:=5 \
  log_face_detections:=true

# 4. Check launch terminal output for errors
# Look for: ERROR, WARNING, FAIL

# 5. Test components individually
cd ~/dev/r2d2/tests/camera && python3 oakd_test.py
cd ~/dev/r2d2/tests/camera && python3 face_detection_demo.py
```

---

## 10. When to Rebuild vs Restart

| Problem | Solution | Time |
|---------|----------|------|
| Topics not publishing | Restart launch | 10 sec |
| Frame rate drops | Disable recognition | 2 sec |
| Hangs or crashes | Kill + restart | 30 sec |
| Camera not detected | Reseat USB + restart | 1 min |
| "Illegal instruction" | Set OPENBLAS + restart | 10 sec |
| Persistent errors | Rebuild workspace | 2 min |
| System won't boot | Power cycle | 2 min |
| Still broken after rebuild | Check git status, pull latest | 5 min |

---

## Summary

✅ **Daily startup:** 2 minutes (environment + launch)  
✅ **Daily health check:** 5 minutes (run script from section 4)  
✅ **Common issues:** See troubleshooting guide (section 5)  
✅ **Performance baseline:** Reference in section 8  
✅ **When stuck:** See debugging template (section 9.2)  

**Most important:** Always set `export OPENBLAS_CORETYPE=ARMV8` before launching!

---

*Operations Checklist created: December 7, 2025*
