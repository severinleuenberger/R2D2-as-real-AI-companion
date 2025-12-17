# Gesture System Implementation & Debug Guide

**Created:** December 17, 2025  
**Goal:** Enable gesture-controlled speech activation with audio feedback  
**Current Status:** 95% complete - missing speech_node service installation

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Current State Analysis](#current-state-analysis)
3. [Root Cause](#root-cause)
4. [Implementation Steps](#implementation-steps)
5. [Testing Procedures](#testing-procedures)
6. [Verification Checklist](#verification-checklist)
7. [Troubleshooting Guide](#troubleshooting-guide)
8. [System Architecture](#system-architecture)
9. [Critical Files Reference](#critical-files-reference)
10. [Command Reference](#command-reference)

---

## Executive Summary

### What Works
- Camera streaming at 12 Hz ✅
- Face detection functioning ✅
- Gesture recognition 100% accurate ✅
- gesture_intent_node running and configured ✅
- All parameters fixed and correct ✅
- Audio playback tested and working ✅

### What's Missing
- **speech_node is NOT running** ❌
- No systemd service for speech_node ❌
- Cannot test complete workflow without speech_node ❌

### What You Need to Do
1. Install speech_node systemd service (5 minutes)
2. Verify person_status topic publishing (2 minutes)
3. Test complete gesture workflow (5 minutes)
4. Verify auto-start on reboot (5 minutes)
5. Document final working configuration (10 minutes)

**Total estimated time: 30 minutes**

---

## Current State Analysis

### Running Services (systemd)
```bash
✅ r2d2-camera-perception.service - ACTIVE
✅ r2d2-gesture-intent.service - ACTIVE
❌ r2d2-speech-node.service - DOES NOT EXIST
```

### Running ROS2 Nodes
```bash
✅ /oak_d_camera
✅ /image_listener
✅ /gesture_intent_node
✅ /audio_notification_node
✅ /r2d2_heartbeat_node
❌ /speech_node - NOT RUNNING
```

### ROS2 Topics Status
```bash
✅ /oak/rgb/image_raw - Publishing at 12 Hz
✅ /r2d2/perception/gesture_event - Publishing (fist, index_finger_up)
❓ /r2d2/perception/is_target_person - NOT verified
❓ /r2d2/audio/person_status - NOT verified (CRITICAL!)
```

### ROS2 Services Status
```bash
❌ /r2d2/speech/start_session - Exists but no provider (hangs)
❌ /r2d2/speech/stop_session - Exists but no provider (hangs)
```

### Parameters Verified
```bash
✅ target_person_name: severin
✅ target_person_gesture_name: severin
✅ gesture_model_path: .../severin_gesture_classifier.pkl
✅ enable_gesture_recognition: True
✅ enable_face_recognition: True
✅ audio_feedback_enabled: True
✅ auto_shutdown_timeout_seconds: 35.0
```

---

## Root Cause

### Primary Issue
**speech_node is not running**, therefore:
- `/r2d2/speech/start_session` service has no provider
- `/r2d2/speech/stop_session` service has no provider
- gesture_intent_node cannot trigger speech activation
- No audio beeps play (beeps only play on session status change)

### Secondary Issue (CRITICAL BLOCKER!)
**gesture_intent_node has a GATE that blocks all gestures unless person_status == "red"**

```python
# Line 206-211 in gesture_intent_node.py
if self.person_status != "red":
    # ALL GESTURES ARE IGNORED!
    return
```

**This means:**
- Even if speech_node is running, gestures will be ignored unless person_status == "red"
- person_status comes from `/r2d2/audio/person_status` topic
- Published by `audio_notification_node`
- **Must verify this topic is publishing!**

### Person Status Color Meanings
- **RED** = Target person recognized → Gestures ENABLED ✅
- **BLUE** = No person or unknown person → Gestures BLOCKED ❌
- **GREEN** = Speech session active → Gestures BLOCKED ❌

---

## Implementation Steps

### Step 1: Verify Prerequisites

Before installing speech_node, verify the system is ready:

```bash
# 1. Check all current services are running
systemctl status r2d2-camera-perception
systemctl status r2d2-gesture-intent

# 2. Verify gesture detection works
cd /home/severin/dev/r2d2
bash WATCH_GESTURES.sh
# Make gestures - should see "fist" and "index_finger_up"
# Press Ctrl+C to stop

# 3. CRITICAL: Verify person_status topic is publishing
cd /home/severin/dev/r2d2/ros2_ws
source install/setup.bash
timeout 5 ros2 topic echo /r2d2/audio/person_status
# Expected: Should see JSON with "status": "red" or "blue"
# If nothing appears, this is a BLOCKER - see troubleshooting
```

**CHECKPOINT:** Do NOT proceed unless person_status topic is publishing!

---

### Step 2: Install speech_node Service

```bash
# Navigate to project directory
cd /home/severin/dev/r2d2

# Run installation script
bash INSTALL_SPEECH_NODE_SERVICE.sh

# Expected output:
# ✅ Service is RUNNING
# ✅ /r2d2/speech/start_session service available
# ✅ /r2d2/speech/stop_session service available
```

**If installation fails:**
- Check logs: `sudo journalctl -u r2d2-speech-node -n 50`
- Verify OpenAI API key is configured
- See troubleshooting section below

---

### Step 3: Verify speech_node is Running

```bash
# Check systemd service status
systemctl status r2d2-speech-node
# Expected: "active (running)"

# Check ROS2 node
cd /home/severin/dev/r2d2/ros2_ws
source install/setup.bash
ros2 node list | grep speech_node
# Expected: /speech_node

# Verify services are available
ros2 service list | grep speech
# Expected:
# /r2d2/speech/start_session
# /r2d2/speech/stop_session

# Test service call (should NOT hang)
timeout 5 ros2 service call /r2d2/speech/start_session std_srvs/srv/Trigger
# Expected: Response within 5 seconds (not timeout)
```

**CHECKPOINT:** speech_node must be running before testing gestures!

---

### Step 4: Verify Person Status

This is CRITICAL - gestures are blocked if person_status != "red"

```bash
# Monitor person status in real-time
cd /home/severin/dev/r2d2/ros2_ws
source install/setup.bash

# Stand in front of camera
ros2 topic echo /r2d2/audio/person_status

# Expected output when you're recognized:
# data: '{"status": "red", "person_name": "severin"}'

# Expected output when no one in view:
# data: '{"status": "blue"}'
```

**If person_status is never "red":**
- Face recognition may not be working
- Check camera-perception logs: `sudo journalctl -u r2d2-camera-perception -n 50`
- Verify face model exists: `ls -la /home/severin/dev/r2d2/data/face_recognition/models/`
- See troubleshooting section

**CHECKPOINT:** You must see person_status = "red" when standing in front of camera!

---

### Step 5: Test Gesture Workflow

**Prerequisites:**
- speech_node running ✅
- person_status showing "red" ✅

**Test procedure:**

```bash
# Optional: Monitor gesture_intent_node logs in separate terminal
sudo journalctl -u r2d2-gesture-intent -f

# Stand in front of camera
# Verify LED is RED (person recognized)

# Test 1: Start speech with index finger up
# Action: Raise index finger ☝️
# Expected:
#   - Hear START beep (Voicy_R2-D2 - 16.mp3)
#   - LED turns GREEN
#   - Log shows: "☝️ Index finger up detected → Starting conversation"

# Test 2: Stop speech with fist
# Action: Make fist ✊
# Expected:
#   - Hear STOP beep (Voicy_R2-D2 - 20.mp3)
#   - LED turns back to RED
#   - Log shows: "✊ Fist detected → Stopping conversation"

# Test 3: Auto-shutdown (35 second watchdog)
# Action: Start speech, then make NO gestures for 35 seconds
# Expected:
#   - After 35s: Hear STOP beep
#   - LED turns back to RED
#   - Log shows: "⏰ Watchdog timeout → Auto-stopping"
```

**If no beeps heard:**
- Check audio output device
- Test audio manually: `bash /home/severin/dev/r2d2/TEST_AUDIO_BEEP.sh`
- Check gesture_intent_node logs for person_status blocks
- See troubleshooting section

---

### Step 6: Verify Auto-Start on Reboot

```bash
# Reboot the system
sudo reboot

# After reboot, check all services auto-started
systemctl status r2d2-camera-perception
systemctl status r2d2-gesture-intent
systemctl status r2d2-speech-node

# All should show: "active (running)"

# Check ROS2 nodes
cd /home/severin/dev/r2d2/ros2_ws
source install/setup.bash
ros2 node list
# Expected:
# /oak_d_camera
# /image_listener
# /gesture_intent_node
# /speech_node
# /audio_notification_node

# Test gesture workflow again (Step 5)
```

---

## Testing Procedures

### Quick Test Suite

Run these commands to verify everything is working:

```bash
# 1. Service Status Check
bash /home/severin/dev/r2d2/VERIFY_AUTO_START.sh

# 2. Gesture Detection Test
bash /home/severin/dev/r2d2/WATCH_GESTURES.sh
# Make gestures, should see events
# Ctrl+C to stop

# 3. Person Status Test
cd /home/severin/dev/r2d2/ros2_ws
source install/setup.bash
timeout 10 ros2 topic echo /r2d2/audio/person_status
# Should see "red" when you're in view

# 4. Manual Speech Service Test
bash /home/severin/dev/r2d2/TEST_SPEECH_SERVICE.sh
# Should hear two beeps (start and stop)

# 5. Complete Workflow Test
# Follow Step 5 procedure above
```

---

## Verification Checklist

Copy this checklist and mark items as you complete them:

### Pre-Installation Checks
- [ ] r2d2-camera-perception service is running
- [ ] r2d2-gesture-intent service is running
- [ ] Gestures detected when watching `/r2d2/perception/gesture_event`
- [ ] person_status topic publishes "red" when standing in camera view
- [ ] Audio playback works (tested with TEST_AUDIO_BEEP.sh)

### speech_node Installation
- [ ] Ran INSTALL_SPEECH_NODE_SERVICE.sh successfully
- [ ] r2d2-speech-node service is active
- [ ] /speech_node appears in `ros2 node list`
- [ ] /r2d2/speech/start_session service responds (doesn't hang)
- [ ] /r2d2/speech/stop_session service responds (doesn't hang)

### Gesture Workflow Tests
- [ ] LED is RED when standing in camera view
- [ ] Index finger up gesture triggers START beep
- [ ] LED turns GREEN after start gesture
- [ ] Speech service becomes active
- [ ] Fist gesture triggers STOP beep
- [ ] LED returns to RED after stop gesture
- [ ] Speech service becomes inactive
- [ ] Auto-shutdown works after 35 seconds of inactivity

### Auto-Start Verification
- [ ] All three services auto-start after reboot
- [ ] All ROS2 nodes appear after reboot
- [ ] Gesture workflow works after reboot
- [ ] No manual intervention needed

### Documentation
- [ ] Updated system documentation with final configuration
- [ ] Documented any issues encountered and solutions
- [ ] Verified all helper scripts work as expected

---

## Troubleshooting Guide

### Issue 1: speech_node Won't Start

**Symptoms:**
- Service fails to start
- Logs show API key errors
- Service crashes immediately

**Solutions:**

```bash
# Check logs
sudo journalctl -u r2d2-speech-node -n 100

# Common issues:

# 1. Missing or invalid OpenAI API key
# Check: grep -r "openai_api_key" /home/severin/dev/r2d2/ros2_ws/src/r2d2_speech/
# Fix: Ensure API key is configured in launch file or environment

# 2. Audio device not available
# Check: aplay -l
# Fix: Update sink_device parameter in launch file

# 3. Database path issues
# Check: ls -la ~/dev/r2d2/r2d2_speech/data/
# Fix: Create directory if missing: mkdir -p ~/dev/r2d2/r2d2_speech/data/

# 4. Python dependencies missing
# Fix: pip install openai websockets pyaudio
```

---

### Issue 2: person_status Topic Not Publishing

**Symptoms:**
- `ros2 topic echo /r2d2/audio/person_status` shows nothing
- Gestures are always ignored

**Solutions:**

```bash
# Check if audio_notification_node is running
ros2 node list | grep audio_notification

# If missing, check camera-perception service
sudo journalctl -u r2d2-camera-perception -n 50 | grep audio_notification

# Check face recognition is enabled
cd /home/severin/dev/r2d2/ros2_ws
source install/setup.bash
ros2 param get /image_listener enable_face_recognition
# Should be: Boolean value is: True

# Check face model exists
ls -la /home/severin/dev/r2d2/data/face_recognition/models/severin_lbph.xml

# If model missing, retrain:
cd /home/severin/dev/r2d2/tests/face_recognition
bash train_with_service_management.sh
```

---

### Issue 3: person_status Always Shows "blue"

**Symptoms:**
- Topic publishes but always shows `{"status": "blue"}`
- Face is detected but not recognized

**Solutions:**

```bash
# Check recognition parameters
ros2 param get /image_listener target_person_name
# Should be: String value is: severin

ros2 param get /image_listener recognition_confidence_threshold
# Default: 150.0 (lower is stricter)

# If recognition fails, may need to:
# 1. Lower confidence threshold (make recognition less strict)
# 2. Retrain face model with more varied images
# 3. Improve lighting conditions

# Check logs for recognition attempts
sudo journalctl -u r2d2-camera-perception -n 100 | grep -i "recognized\|confidence"
```

---

### Issue 4: Gestures Detected but No Beeps

**Symptoms:**
- Gestures appear in `/r2d2/perception/gesture_event`
- No audio beeps play
- No visible response

**Debug steps:**

```bash
# 1. Check person_status (MOST COMMON CAUSE)
ros2 topic echo /r2d2/audio/person_status
# Must show "red" for gestures to work!

# 2. Check gesture_intent_node logs
sudo journalctl -u r2d2-gesture-intent -f
# Should show: "☝️ Index finger up detected" or similar
# If shows: "Gesture ignored: person_status=blue" → Fix person_status first!

# 3. Check speech_node is running
ros2 node list | grep speech_node

# 4. Test speech services manually
ros2 service call /r2d2/speech/start_session std_srvs/srv/Trigger
# Should hear beep within 2 seconds

# 5. Check audio output
bash /home/severin/dev/r2d2/TEST_AUDIO_BEEP.sh

# 6. Check cooldown periods
# If gestures trigger but nothing happens, may be in cooldown
# Wait 2-3 seconds between gestures
```

---

### Issue 5: Audio Too Loud

**Symptoms:**
- Beeps play but are too loud

**Solutions:**

```bash
# Option 1: Adjust system volume
alsamixer
# Navigate with arrow keys, M to mute/unmute, ESC to exit

# Option 2: Reduce volume for specific device
amixer set Master 50%

# Option 3: Edit gesture_intent_node to add volume control
# (Would require code changes - not recommended)
```

---

### Issue 6: Auto-Shutdown Doesn't Work

**Symptoms:**
- Speech stays active even when person leaves
- Watchdog timer doesn't trigger

**Debug steps:**

```bash
# Check watchdog is enabled
ros2 param get /gesture_intent_node auto_shutdown_enabled
# Should be: Boolean value is: True

# Check timeout
ros2 param get /gesture_intent_node auto_shutdown_timeout_seconds
# Should be: Double value is: 35.0

# Monitor gesture_intent logs
sudo journalctl -u r2d2-gesture-intent -f
# Should show watchdog status updates every ~10 seconds
# Look for: "⏰ Watchdog: Person absent, starting timer"

# Check person_status changes
ros2 topic echo /r2d2/audio/person_status
# When you leave view, should change from "red" to "blue"
```

---

### Issue 7: Services Don't Auto-Start After Reboot

**Symptoms:**
- After reboot, services are "inactive"
- Manual start required

**Solutions:**

```bash
# Check service is enabled
systemctl is-enabled r2d2-speech-node
# Should show: enabled

# If disabled, enable it
sudo systemctl enable r2d2-speech-node

# Check for service failures
systemctl status r2d2-speech-node
# Look for: "failed" or errors

# Check service logs
sudo journalctl -u r2d2-speech-node -n 100

# Verify service dependencies
# speech-node should start AFTER camera-perception
# Check "After=" line in service file
cat /etc/systemd/system/r2d2-speech-node.service | grep "After="
```

---

## System Architecture

### Service Dependency Chain

```
Network Available
  ↓
┌─────────────────────────────────────┐
│ r2d2-camera-perception.service      │
│ - oak_d_camera                      │
│ - image_listener                    │
│ - audio_notification_node           │
└─────────────────────────────────────┘
  ↓ publishes topics
  ↓
┌─────────────────────────────────────┐
│ r2d2-gesture-intent.service         │
│ - gesture_intent_node               │
│   Subscribes:                       │
│   - /r2d2/perception/gesture_event  │
│   - /r2d2/audio/person_status       │
└─────────────────────────────────────┘
  ↓ calls services
  ↓
┌─────────────────────────────────────┐
│ r2d2-speech-node.service            │
│ - speech_node                       │
│   Provides:                         │
│   - /r2d2/speech/start_session      │
│   - /r2d2/speech/stop_session       │
└─────────────────────────────────────┘
```

### Data Flow for Gesture Recognition

```
1. Camera captures frame
   ↓ /oak/rgb/image_raw
2. image_listener processes frame
   ↓ detects hand + predicts gesture
   ↓ /r2d2/perception/gesture_event: "index_finger_up"
3. gesture_intent_node receives event
   ↓ checks person_status == "red"?
   ↓ YES: proceed | NO: ignore gesture
4. gesture_intent_node calls service
   ↓ /r2d2/speech/start_session
5. speech_node activates
   ↓ publishes /r2d2/speech/session_status
6. gesture_intent_node sees status change
   ↓ plays audio beep (ffplay)
   ↓ Voicy_R2-D2 - 16.mp3
```

### Critical Blocking Conditions

**Gestures will be IGNORED if:**
1. `person_status != "red"` (person not recognized)
2. Cooldown period hasn't elapsed (2s for start, 1s for stop)
3. Session already in desired state (can't start if already started)

---

## Critical Files Reference

### Systemd Service Files

**Location:** `/etc/systemd/system/`

1. `r2d2-camera-perception.service` (INSTALLED)
   - Starts camera and perception nodes
   - Modified to include gesture parameters
   - Key parameters:
     - `target_person_name:=severin`
     - `target_person_gesture_name:=severin`
     - `gesture_model_path:=.../severin_gesture_classifier.pkl`

2. `r2d2-gesture-intent.service` (INSTALLED)
   - Starts gesture intent node
   - Depends on camera-perception
   - Key parameters:
     - `auto_shutdown_timeout_seconds:=35.0`
     - `audio_feedback_enabled:=true`

3. `r2d2-speech-node.service` (TO BE INSTALLED)
   - Provides speech services
   - Source: `/home/severin/dev/r2d2/r2d2-speech-node.service`
   - Startup script: `/home/severin/dev/r2d2/start_speech_node.sh`

### ROS2 Package Files

**Location:** `/home/severin/dev/r2d2/ros2_ws/src/`

1. `r2d2_perception/r2d2_perception/image_listener.py`
   - Processes camera frames
   - Detects faces and gestures
   - Publishes gesture_event topic

2. `r2d2_gesture/r2d2_gesture/gesture_intent_node.py`
   - Translates gestures to actions
   - Has person_status gate (line 206-211)
   - Manages audio feedback

3. `r2d2_speech/r2d2_speech/speech_node.py`
   - Manages OpenAI connection
   - Provides start/stop services

### Launch Files (Modified)

1. `r2d2_perception/launch/perception.launch.py`
   - Added: `target_person_gesture_name` parameter
   - Fixed: parameter name `gesture_model_path`

2. `r2d2_bringup/launch/r2d2_camera_perception.launch.py`
   - Added: `target_person_gesture_name` parameter
   - Fixed: parameter name `gesture_model_path`

3. `r2d2_gesture/launch/gesture_intent.launch.py`
   - Fixed: timeout comment (35s not 300s)

### Data Files

1. Gesture Model:
   - `/home/severin/dev/r2d2/data/gesture_recognition/models/severin_gesture_classifier.pkl`
   - Contains: SVM classifier + gesture labels

2. Face Model:
   - `/home/severin/dev/r2d2/data/face_recognition/models/severin_lbph.xml`
   - Contains: LBPH face recognition model

3. Audio Files:
   - Start beep: `/home/severin/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/Voicy_R2-D2 - 16.mp3`
   - Stop beep: `/home/severin/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/Voicy_R2-D2 - 20.mp3`

### Helper Scripts

**Location:** `/home/severin/dev/r2d2/`

All executable, run with: `bash SCRIPT_NAME.sh`

- `INSTALL_SPEECH_NODE_SERVICE.sh` - Install speech_node service
- `VERIFY_AUTO_START.sh` - Check service status
- `WATCH_GESTURES.sh` - Monitor gesture events
- `TEST_AUDIO_BEEP.sh` - Test audio playback
- `TEST_SPEECH_SERVICE.sh` - Test speech services manually
- `TEST_GESTURE_WORKFLOW.sh` - Complete workflow test guide
- `DIAGNOSE_GESTURE_SYSTEM.sh` - System diagnostics
- `CHECK_GESTURE_INTENT.sh` - Check gesture intent status

---

## Command Reference

### Service Management

```bash
# Check status
systemctl status r2d2-camera-perception
systemctl status r2d2-gesture-intent
systemctl status r2d2-speech-node

# Start/stop manually
sudo systemctl start SERVICE_NAME
sudo systemctl stop SERVICE_NAME
sudo systemctl restart SERVICE_NAME

# Enable/disable auto-start
sudo systemctl enable SERVICE_NAME
sudo systemctl disable SERVICE_NAME

# View logs
sudo journalctl -u SERVICE_NAME -f          # Follow logs
sudo journalctl -u SERVICE_NAME -n 50       # Last 50 lines
sudo journalctl -u SERVICE_NAME --since "5 minutes ago"
```

### ROS2 Debugging

```bash
# Setup environment (run in every new terminal)
cd /home/severin/dev/r2d2/ros2_ws
source install/setup.bash

# List nodes
ros2 node list

# List topics
ros2 topic list

# Monitor topic (real-time)
ros2 topic echo /TOPIC_NAME

# Check topic publishing rate
ros2 topic hz /TOPIC_NAME

# List services
ros2 service list

# Call service
ros2 service call /SERVICE_NAME SERVICE_TYPE

# Get parameter
ros2 param get /NODE_NAME PARAM_NAME

# Set parameter (temporary)
ros2 param set /NODE_NAME PARAM_NAME VALUE

# List all parameters for a node
ros2 param list /NODE_NAME
```

### Build Commands

```bash
cd /home/severin/dev/r2d2/ros2_ws
source /opt/ros/humble/setup.bash

# Build single package
colcon build --packages-select PACKAGE_NAME --symlink-install

# Build multiple packages
colcon build --packages-select r2d2_perception r2d2_gesture --symlink-install

# Build all packages
colcon build --symlink-install

# After building, source workspace
source install/setup.bash
```

### Useful Diagnostic Commands

```bash
# Check if camera is streaming
ros2 topic hz /oak/rgb/image_raw
# Should show ~12 Hz

# Monitor gesture events
ros2 topic echo /r2d2/perception/gesture_event

# Monitor person status (CRITICAL!)
ros2 topic echo /r2d2/audio/person_status

# Check all R2D2 topics
ros2 topic list | grep r2d2

# Check all R2D2 services
ros2 service list | grep r2d2

# Check system resources
htop
# Look for high CPU/memory usage
```

---

## Success Criteria

You have successfully completed the implementation when:

1. **All services auto-start on boot**
   - r2d2-camera-perception ✅
   - r2d2-gesture-intent ✅
   - r2d2-speech-node ✅

2. **All ROS2 nodes are running**
   - /oak_d_camera ✅
   - /image_listener ✅
   - /gesture_intent_node ✅
   - /speech_node ✅
   - /audio_notification_node ✅

3. **Person detection works**
   - LED shows RED when standing in camera view ✅
   - person_status topic publishes "red" ✅

4. **Gesture workflow complete**
   - Index finger up → START beep + LED green + speech active ✅
   - Fist → STOP beep + LED red + speech inactive ✅
   - Auto-shutdown after 35s inactivity ✅

5. **System survives reboot**
   - All services auto-start ✅
   - Gesture workflow works immediately ✅
   - No manual intervention needed ✅

---

## Next Steps After Completion

Once everything is working:

1. **Document final configuration**
   - Update `300_GESTURE_SYSTEM_OVERVIEW.md` with final status
   - Note any configuration changes made
   - Document any issues encountered and solutions

2. **Optimize if needed**
   - Adjust audio volume if too loud
   - Tune gesture recognition confidence thresholds
   - Adjust watchdog timeout if needed

3. **Consider enhancements**
   - Add more gestures
   - Implement auto-restart on person return
   - Add visual feedback beyond LED colors
   - Improve error handling

4. **Update documentation**
   - Create user guide for gesture training
   - Document common issues and solutions
   - Add screenshots or videos of working system

---

## Important Notes

### Person Status Gate
**This is the #1 cause of "gestures not working"!**

The gesture_intent_node has a hard-coded gate at line 206-211:
```python
if self.person_status != "red":
    return  # Gesture ignored!
```

**Always verify person_status first** when debugging gesture issues!

### Service Start Order Matters
1. camera-perception must start first (publishes topics)
2. gesture-intent depends on topics from camera-perception
3. speech-node can start anytime (provides services)

### Cooldown Periods Prevent Rapid Triggering
- After triggering start: 2 second cooldown
- After triggering stop: 1 second cooldown
- Wait between gestures to avoid "ignored" messages

### Audio Feedback Timing
- Beeps play when **session status changes**, not when gesture detected
- Workflow: Gesture → Service call → Status change → Beep
- If service fails, no status change, no beep!

---

**END OF GUIDE**

Created: December 17, 2025  
Last Updated: December 17, 2025  
Status: Ready for implementation  

For questions or issues, refer to:
- `300_GESTURE_SYSTEM_OVERVIEW.md` - System architecture
- `200_SPEECH_SYSTEM_REFERENCE.md` - Speech system details  
- `001_ARCHITECTURE_OVERVIEW.md` - Complete system overview

