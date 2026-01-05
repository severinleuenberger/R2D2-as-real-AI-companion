# Session Lessons Learned - January 2, 2026

**Session Context:** Attempted to fix stop beep timing issue in R2D2 speech system

---

## 🎯 Good Ideas & Concepts

### Two-Stage Fist Confirmation
- **Concept:** Warning beep on Stage 1 (1.5s sustained fist), then stop beep on Stage 2 (another 1.5s hold)
- **Benefits:** Prevents accidental conversation stops, gives user chance to cancel
- **Implementation:** Already existed in golden commit - was working correctly
- **Sounds:** `Voicy_R2-D2 - 7.mp3` (warning), `Voicy_R2-D2 - 20.mp3` (stop)

### Rolling Window Detection with Threshold
- **Mechanism:** Track gesture detections over time window, require threshold to trigger
- **Parameters:** 
  - Detection rate: ~10Hz from MediaPipe Hands
  - Window: 1.5 seconds = 15 max possible detections
  - Threshold: 10 detections = ~67% hit rate (allows frame drops)
- **Benefit:** Robust against flicker, requires sustained gesture

### Idle Timeout for WebSocket
- **Problem:** OpenAI Realtime API has 60-minute session limit, after which it keeps billing
- **Solution:** Auto-disconnect WebSocket after 2 minutes of idle time (no streaming)
- **Implementation:** Timer started after `_stop_streaming()`, cancelled on `_start_streaming()`
- **Status:** Good idea, but never tested in isolation

### Force Disconnect Service
- **Purpose:** Immediate WebSocket close on explicit user termination (confirmed fist gesture)
- **Difference from stop_session:** 
  - `stop_session`: Stop streaming, keep WebSocket open (user might return)
  - `force_disconnect`: Close WebSocket immediately (user is done)
- **Status:** Good concept, but added alongside too many other changes

### Grace Period After Session Start
- **Duration:** 5 seconds after starting conversation
- **Purpose:** Ignore fist gestures to prevent accidental stop during greeting
- **Status:** Already implemented in golden commit, working correctly

---

## ❌ What Went Wrong

### 1. Changed Too Many Things at Once
**What happened:**
- Lowered fist threshold (1.5s/10 → 1.0s/6)
- Added force_disconnect service
- Modified service call flow in gesture node
- Added idle timeout to speech node
- Changed logging verbosity

**Impact:** Impossible to isolate which change caused the system to break

**Should have done:** One change, rebuild, test, verify, commit, then next change

### 2. Didn't Verify Force Disconnect Service
**What happened:**
- Added `force_disconnect_srv` to speech_node.py
- Added `force_disconnect_client` to gesture_intent_node.py
- Changed fist Stage 2 to call force_disconnect instead of stop_session
- Never tested if the service actually worked before lowering thresholds

**Impact:** When system broke, couldn't tell if it was threshold issue or service issue

### 3. Lowered Fist Threshold Too Aggressively
**Original:** 1.5s window, 10 detections (67% hit rate)
**Changed to:** 1.0s window, 6 detections (60% hit rate)

**Why:** User reported fist gesture "vanishing too quickly"
**Problem:** This might have been a camera hardware issue, not a threshold issue
**Result:** Made detection potentially too sensitive/flickery

### 4. Forgot to Rebuild ROS2 Packages
**Critical mistake:** Modified Python source files but forgot:
```bash
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_speech r2d2_gesture
sudo systemctl restart r2d2-speech-node.service r2d2-gesture-intent.service
```

**Impact:** Old code kept running, new changes never applied, led to confusion

### 5. Chased Phantom Problems
**Time wasted on:**
- Volume issues (master volume vs local volume calculation)
- Camera crashes (hardware X_LINK_ERROR, not software bug)
- Bluetooth audio routing (worked fine, was red herring)
- Service timing (was actually old code running)

**Should have done:** Verify basic functionality first, check if new code is actually running

### 6. Misdiagnosed the Original Problem
**User's complaint:** "Stop beep timing not correct"
**My assumption:** Race condition with audio stream closing
**Reality:** Looking at the original code, the stop beep was ALREADY played before calling `_exit_speaking_state()`, so timing was likely correct

**Possible real issue:** User might have wanted different timing/behavior, not a bug fix

---

## 📚 Lessons Learned

### Development Process

1. **ONE CHANGE AT A TIME**
   - Make single, isolated change
   - Rebuild packages
   - Restart services
   - Test thoroughly
   - Commit if successful
   - Then move to next change

2. **Always Rebuild After Code Changes**
   ```bash
   cd ~/dev/r2d2/ros2_ws
   colcon build --packages-select <package_name>
   sudo systemctl restart <service_name>
   ```

3. **Verify Changes Are Active**
   - Check logs for expected log messages
   - Use `journalctl -u <service> -f` to watch real-time
   - Look for parameter values in startup logs
   - Verify services exist: `ros2 service list`

4. **Keep Golden Commit as Baseline**
   - Document golden commit hash
   - When things break: `git checkout -- .` to revert
   - Debug from working state, not broken state

5. **When System Breaks, Revert First**
   - Don't try to fix a broken state
   - Revert to working code
   - Then debug in clean environment

### Technical Understanding

1. **ROS2 Build System**
   - Python changes require `colcon build` (even with `--symlink-install`)
   - Services must be restarted to load new code
   - Old processes keep running until explicitly stopped

2. **Gesture Detection Parameters**
   - MediaPipe Hands: ~10Hz detection rate
   - Rolling window: maintain list of recent detection timestamps
   - Threshold: balance between responsiveness and robustness
   - Working values: 1.5s window, 10 detections (67% hit rate)

3. **Audio Feedback Chain**
   - `gesture_intent_node.py`: Triggers audio playback via ffplay
   - Effective volume: `master_volume * audio_volume`
   - Master volume from hardware knob (0.35 = 35%)
   - Local volume from config (0.02 = 2%)
   - Effective: 0.35 * 0.02 = 0.007 (0.7%) - very quiet!

4. **WebSocket Connection Management**
   - OpenAI Realtime API: 60-minute session limit
   - After limit: keeps billing for idle connection
   - Need explicit disconnect to stop billing
   - Idle timeout: good safety measure

5. **Hardware Issues vs Software Issues**
   - Camera crashes (X_LINK_ERROR): hardware/USB issue
   - Blue status: camera not detected
   - Red status: camera detected, waiting for person
   - These are NOT software bugs

---

## 🔧 Technical Notes

### Fist Gesture Detection Flow

```
1. MediaPipe detects "fist" gesture → ~10Hz
2. gesture_callback() receives gesture event
3. Check grace period (5s after session start)
4. Check if session active
5. Add timestamp to fist_detection_buffer
6. Remove old timestamps (outside 1.5s window)
7. Count detections in window
8. If count >= 10:
   - Stage 1: Play warning beep, reset buffer
   - Stage 2: Play stop beep, call _exit_speaking_state()
9. If no fist detected for 0.5s: reset to idle stage
```

### Original Code Analysis

**Stop beep was ALREADY played before service call:**
```python
# In gesture_intent_node.py, fist Stage 2:
self._play_audio_feedback(self.stop_beep_sound)  # ← Beep plays first
self.stop_beep_already_played = True
if self.speaking_state == "speaking":
    self._exit_speaking_state(reason="user_fist_gesture_confirmed")  # ← Then exit
```

**This means:** The "beep timing issue" may not have existed, or was misunderstood.

### Service Call Changes (What I Changed)

**Original:**
```python
_exit_speaking_state() → _stop_session() → speech_node.stop_session service
```

**My changes:**
```python
_exit_speaking_state(force_disconnect=True) → _force_disconnect() → speech_node.force_disconnect service
```

**Difference:**
- `stop_session`: Stops streaming, keeps WebSocket open
- `force_disconnect`: Stops streaming AND closes WebSocket

---

## 🎬 What Should Have Been Done

### Minimal Fix Approach

1. **First, verify the problem actually exists**
   - Test current golden commit behavior
   - Document exact issue with timestamps
   - Confirm it's reproducible

2. **If beep timing is truly broken:**
   - Option A: Add small delay before service call
   - Option B: Wait for audio playback to finish
   - Option C: Don't change anything (might be working)

3. **For billing safety:**
   - Add idle timeout in ISOLATION
   - Test for several days
   - Verify it doesn't disconnect during active use
   - Commit when proven stable

4. **For force disconnect:**
   - Add service in ISOLATION
   - Test it works via `ros2 service call`
   - Then modify gesture node to use it
   - Test end-to-end
   - Commit when proven working

### Proper Testing Protocol

1. Start from clean golden commit
2. Make ONE change
3. Rebuild: `colcon build --packages-select <pkg>`
4. Restart: `sudo systemctl restart <service>`
5. Monitor logs: `journalctl -u <service> -f`
6. Test manually with actual gestures
7. Verify expected behavior
8. If good: commit with descriptive message
9. If bad: `git checkout -- .` and try different approach

---

## 📊 Summary Statistics

**Time spent:** ~2 hours of back-and-forth debugging
**Changes made:** 6+ modifications across 2 files
**Rebuilds:** 3-4 times
**Service restarts:** 5+ times
**Camera restarts:** 3 times (hardware issue)
**Result:** Full revert to golden commit

**Efficiency if done right:**
- Make one change: 5 minutes
- Test: 5 minutes
- Verify: 5 minutes
- Total: 15 minutes per change
- 3 changes = 45 minutes

**Actual time:** 120 minutes (2.67x longer due to wrong approach)

---

## 🎯 Action Items for Future

1. ✅ Document golden commit hash in project README
2. ✅ Create checklist for making ROS2 package changes
3. ✅ Add "verify rebuild" step to all code change workflows
4. ✅ Separate hardware issues from software issues
5. ✅ Test changes in isolation before combining
6. ✅ Keep this lessons learned doc as reference

---

**Date:** January 2, 2026  
**System:** R2D2 Robot (ROS2 Humble, Jetson Orin Nano)  
**Outcome:** Reverted to golden commit, system restored to working state  
**Lesson:** Measure twice, cut once. Test small, test often.

