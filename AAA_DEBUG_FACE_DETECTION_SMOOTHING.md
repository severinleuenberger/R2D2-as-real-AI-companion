# DEBUG: Face Detection Smoothing Issue

**Date:** December 18, 2025  
**Status:** Active Issue - Needs Fix  
**Priority:** Medium (system works, but recognition beeps unreliable)  
**For:** Next agent working on this issue

---

## Executive Summary

**What Works:** ✅
- Gesture-controlled speech-to-speech (fully operational)
- Index finger gesture starts conversation
- Fist gesture stops conversation
- "Hello!" beep on recognition
- "Start" and "Stop" beeps on gestures
- SPEAKING state protection (35s consecutive)
- Simplified RED status (15s timer)

**What Needs Fix:** ❌
- "Lost you!" beep unreliable (doesn't play when walking away)
- RED timer keeps resetting due to false face detections
- Face recognition too "nervous" (flickers between detected/not detected)

---

## Root Cause Analysis

### The Problem

**Face detection (`/r2d2/perception/face_count`) is unstable:**
```
Observed pattern when user walks away:
data: 1 (face)
data: 1
data: 0 (no face)
data: 1 (false detection!)
data: 0
data: 0
data: 1 (false detection!)
data: 1
data: 0
...continues flickering for 60+ seconds
```

**Impact on person_id:**
- When face_count > 0, face recognition runs
- False detections sometimes match as "severin" (low confidence matches)
- This publishes person_id = "severin"
- audio_notification_node receives "severin" → Resets RED timer
- RED timer never reaches 15s → No transition to BLUE → No "Lost you!" beep

**Timeline of actual behavior:**
```
t=0s:   User makes fist, stops speech
t=1s:   User walks away
t=2-60s: Face count flickers (1,0,1,0...)
        Occasional false recognitions reset RED timer
        Timer never reaches 15s
t=90s:  Finally no faces detected for 15s consecutive
        RED timer expires
        "Lost you!" beep plays (but user is far away, doesn't hear)
```

---

## What Was Implemented (December 18, 2025)

### Fix #1: auto_start=false ✅ COMPLETE
**Files Changed:**
- `ros2_ws/src/r2d2_speech/config/speech_params.yaml` line 18
- `ros2_ws/src/r2d2_speech/r2d2_speech_ros/speech_node.py` line 53
- `start_speech_node.sh` line 14
- `ros2_ws/src/r2d2_speech/launch/speech_node.launch.py` line 30

**Result:** Speech node no longer auto-starts session on boot, waits for gesture trigger

### Fix #2: Simplified RED Status ✅ COMPLETE
**Files Changed:**
- `ros2_ws/src/r2d2_audio/r2d2_audio/audio_notification_node.py`
  - Removed: jitter_tolerance (5s) + loss_confirmation (15s) complex logic
  - Added: Simple 15s reset timer
  - Reduced code: 110 lines → 40 lines (63% reduction)

**Result:** RED status timer resets every time target_person recognized, expires after 15s without recognition

### Fix #3: SPEAKING State ✅ COMPLETE
**Files Changed:**
- `ros2_ws/src/r2d2_gesture/r2d2_gesture/gesture_intent_node.py`
  - Added: SPEAKING state machine
  - Added: 35s consecutive non-RED protection
  - Added: _enter_speaking_state(), _exit_speaking_state(), _update_speaking_protection()

**Result:** Conversations protected from brief face recognition failures

### Fix #4: session_active Detection ✅ COMPLETE
**Files Changed:**
- `ros2_ws/src/r2d2_gesture/r2d2_gesture/gesture_intent_node.py` line 174

**Changed:**
```python
# Before:
self.session_active = (status_str in ['active', 'connected'])

# After:
self.session_active = (status_str == 'connected')
```

**Result:** Only treats "connected" as conversation active, ignores lifecycle "active" state

### Fix #5: Audio Playback ✅ COMPLETE
**Files Changed:**
- `ros2_ws/src/r2d2_audio/r2d2_audio/audio_notification_node.py` _play_audio_file method
- `ros2_ws/src/r2d2_audio/r2d2_audio/audio_player.py` removed -ao option

**Result:** ffplay called directly without nested subprocess, audio plays correctly

---

## Remaining Issue: Face Detection Smoothing

### Proposed Solution

**Add temporal smoothing to face detection before running recognition:**

```python
# In image_listener.py:

# New state variables:
self.face_detected_start_time = None  # When face first detected
self.face_stability_timeout = 5.0  # Seconds face must be stable

# In image_callback():
if face_count > 0:
    # Face detected
    if self.face_detected_start_time is None:
        # First detection - start timer
        self.face_detected_start_time = current_time
    
    # Check if stable for required duration
    time_stable = current_time - self.face_detected_start_time
    
    if time_stable >= self.face_stability_timeout:
        # Face stable for 5s - safe to run recognition
        if self.recognition_enabled and self.recognition_frame_counter >= self.recognition_frame_skip:
            # Run face recognition
            # ...existing code...
else:
    # No face detected - reset stability timer
    self.face_detected_start_time = None
```

**Effect:**
- Face must be detected continuously for 5 seconds before recognition runs
- Filters out brief false detections
- RED timer won't reset from flickering detections
- "Lost you!" beep will play after 15s actual absence

### Alternative: Smooth face_count Publishing

```python
# Publish face_count only if stable
self.face_count_history = []  # Last N face counts

def get_stable_face_count():
    # Keep last 10 face counts (1 second at 10 Hz)
    self.face_count_history.append(face_count)
    if len(self.face_count_history) > 10:
        self.face_count_history.pop(0)
    
    # If majority (>60%) say faces present, return 1, else 0
    faces_detected = sum(1 for c in self.face_count_history if c > 0)
    return 1 if faces_detected > 6 else 0
```

---

## Testing After Fix

**Test Scenario: Walk Away After Conversation**
1. Start conversation with index finger
2. Speak briefly
3. Stop with fist gesture
4. **Immediately walk completely out of frame**
5. **Stay away for 20 seconds in empty room**
6. EXPECT: "Lost you!" beep at ~15 second mark
7. Return to camera
8. EXPECT: "Hello!" beep

**Success Criteria:**
- ✅ "Lost you!" beep plays when actually gone
- ✅ No false recognitions reset timer
- ✅ Stable RED/BLUE transitions

---

## Current System State (December 18, 2025)

### What's Working
✅ All 4 services auto-start on boot  
✅ Face recognition detects and identifies person  
✅ Gesture recognition (index finger, fist)  
✅ Speech-to-speech with OpenAI Realtime API  
✅ SPEAKING state protection (35s consecutive)  
✅ Simplified RED status (15s timer concept)  
✅ "Hello!" beep on recognition  
✅ "Start" beep on gesture  
✅ "Stop" beep on gesture  
✅ LED status indicator  

### Known Issues
❌ "Lost you!" beep unreliable (face detection flickering)  
❌ RED timer resets from false face detections  
❌ Takes 60-90 seconds for loss detection instead of 15s  

### Parameters in Use
```
audio_notification_node:
  red_status_timeout_seconds: 15.0
  audio_volume: 0.3 (30%, or 0.5/50% from startup script)
  cooldown_seconds: 2.0

gesture_intent_node:
  speaking_protection_seconds: 35.0
  cooldown_start_seconds: 5.0
  cooldown_stop_seconds: 3.0

speech_node:
  auto_start: false
  realtime_voice: sage
```

---

## Files Modified (Git Commits)

**Session commits (December 18, 2025):**
1. `b7d3288e` - Fix audio playback (remove -ao option)
2. `5772e2f2` - Call ffplay directly (no nested subprocess)
3. `f4242a83` - Increase volume to 50%
4. `5b37fd19` - Fix session_active detection
5. `608e1f19` - Add RESTART_ALL_SERVICES.sh script
6. `1133c7dd` - Reorganize documentation (007-009)
7. `9333d89b` - Add SPEAKING state methods
8. `723dab55` - Fix missing SPEAKING state methods
9. `030fda44` - Implement all critical fixes
10. `0d2a2303` - Add RED status and SPEAKING analysis
11. Multiple earlier commits for architecture analysis

**Total changes:** ~2000+ lines across analysis, fixes, and documentation

---

## Code Locations for Next Agent

### Face Detection (Where to Add Smoothing)
**File:** `ros2_ws/src/r2d2_perception/r2d2_perception/image_listener.py`

**Lines 288-308:** Face detection with Haar Cascade
**Lines 310-352:** Face recognition (runs when face_count > 0)

**Add smoothing BEFORE line 310** to filter unstable detections

### RED Status Logic
**File:** `ros2_ws/src/r2d2_audio/r2d2_audio/audio_notification_node.py`

**Lines 238-284:** person_callback (simplified)
**Lines 340-377:** check_loss_state (15s timer)

**This is already simplified and works correctly IF person_id stops publishing**

### SPEAKING State Protection
**File:** `ros2_ws/src/r2d2_gesture/r2d2_gesture/gesture_intent_node.py`

**Lines 76-82:** SPEAKING state variables
**Lines 291-307:** _enter_speaking_state
**Lines 309-316:** _exit_speaking_state
**Lines 318-373:** _update_speaking_protection

**This works perfectly - no changes needed**

---

## Recommended Next Steps

### Immediate (High Priority)
1. **Implement face detection smoothing** in image_listener.py
   - Add 5-second stability requirement before recognition
   - Filter brief false detections
   - Test "Lost you!" beep reliability

### Future (Medium Priority)
2. **Tune face detection parameters**
   - Adjust Haar Cascade sensitivity
   - May reduce false positives
   - See `recognition_confidence_threshold` parameter

3. **Add face detection quality metrics**
   - Publish face detection confidence
   - Log false positive rate
   - Monitor detection stability

---

## Testing Commands for Next Agent

```bash
# Monitor face detection stability
ros2 topic echo /r2d2/perception/face_count | ts '[%H:%M:%S]'
# Watch for flickering when user walks away

# Monitor person_id publishing
ros2 topic echo /r2d2/perception/person_id | ts '[%H:%M:%S]'
# Should stop or change to "unknown" when user leaves

# Monitor RED timer resets
sudo journalctl -u r2d2-audio-notification -f | grep "Timer reset"
# Should stop resetting when user actually gone

# Test complete cycle
# 1. Start conversation, speak, stop with fist
# 2. Walk away immediately (empty room, no objects)
# 3. Start timer - should hear "Lost you!" at ~15 seconds
# 4. If no beep or delayed >30s, face detection still unstable
```

---

## References

**System Integration:**
- 007_SYSTEM_INTEGRATION_REFERENCE.md - Complete architecture
- 008_SYSTEM_INTEGRATION_QUICK_START.md - Quick commands
- 009_SYSTEM_INTEGRATION_TROUBLESHOOTING.md - Debug procedures

**Analysis Documents (archived):**
- _ANALYSIS_AND_DOCUMENTATION/huge_overall_system_map_goal.md
- _ANALYSIS_AND_DOCUMENTATION/huge_overall_system_map_as_built_now.md
- _ANALYSIS_AND_DOCUMENTATION/system_architecture_gap_analysis.md
- _ANALYSIS_AND_DOCUMENTATION/COMPREHENSIVE_FIX_PLAN.md

**Implementation:**
- 100_PERSON_RECOGNITION_REFERENCE.md - Face recognition system
- 200_SPEECH_SYSTEM_REFERENCE.md - Speech system
- 300_GESTURE_SYSTEM_OVERVIEW.md - Gesture system

---

## Quick Context for Next Agent

**User's Goal:** Gesture-controlled speech-to-speech that "just works"

**What Was Fixed Today:**
1. ✅ "Works once then stops" → Fixed auto_start issue
2. ✅ Conversation stability → Added SPEAKING state protection
3. ✅ RED status simplification → 15s reset timer
4. ✅ Audio playback → Direct ffplay call
5. ✅ Beeps audible → 3 out of 4 working

**What Remains:**
1. ❌ "Lost you!" beep unreliable due to face detection flickering
2. Solution identified: Add temporal smoothing to face detection
3. Implementation: 5-second stability requirement before recognition

**Expected Time to Fix:** 30-60 minutes
- Add smoothing logic to image_listener.py
- Test with user walking away scenarios
- Verify "Lost you!" beep plays reliably at 15s mark

---

**Document Version:** 1.0  
**Date:** December 18, 2025  
**Status:** Ready for next implementation session  
**Estimated Effort:** 1 hour (implementation + testing)

