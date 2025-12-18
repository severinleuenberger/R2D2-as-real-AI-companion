# Face Detection Hysteresis Implementation Summary

**Date:** December 18, 2025  
**Status:** ✅ COMPLETE - Ready for Testing  
**Implementation:** Temporal smoothing with hysteresis state machine

---

## Problem Solved

**Original Issue:** Face detection flickering (1,0,1,0...) caused false recognitions that reset the 15-second "Lost you!" timer, preventing the beep from playing reliably.

**Root Cause:** Raw Haar Cascade detector is noisy. The system was reacting to every instantaneous detection, causing "nervous" behavior.

**Solution:** Implemented hysteresis (temporal smoothing) that requires sustained signals before changing state, filtering out noise at the sensor level.

---

## Implementation Details

### Hysteresis State Machine

```
Raw Detection (Flickering)
         ↓
    [Hysteresis Filter]
         ↓
    Stable State (Smooth)
```

**States:**
- `face_stable_state = False`: "Stable Absence" (no person detected)
- `face_stable_state = True`: "Stable Presence" (person detected)

**Transitions:**
- **Absence → Presence:** Requires 2.0 seconds of continuous detection
- **Presence → Absence:** Requires 5.0 seconds of continuous non-detection

### Parameters

Added to `image_listener.py`:

```python
face_presence_threshold: 2.0   # Seconds to confirm presence
face_absence_threshold: 5.0    # Seconds to confirm absence
```

### Key Changes

**File:** `ros2_ws/src/r2d2_perception/r2d2_perception/image_listener.py`

1. **Replaced simple timer with state machine** (lines 195-200)
   - `face_stable_state`: Current stable state (Boolean)
   - `face_transition_start_time`: Tracks when transition started
   - `last_raw_face_count`: Stores raw detection for comparison

2. **Implemented hysteresis logic** (lines 314-345)
   - Tracks raw detection changes
   - Applies time-based thresholds
   - Logs state transitions for debugging

3. **Published stable face_count** (lines 347-350)
   - Only publishes 0 or 1 (stable state)
   - No more flickering in published data

4. **Gated recognition on stable state** (line 353)
   - Face recognition only runs when `face_stable_state = True`
   - person_id, confidence, and gestures all gated automatically

---

## Expected Behavior

### Timeline: User Walks Away

```
t=0s:   User stops conversation (fist gesture)
t=0-5s: Face detector may flicker, but stable state remains TRUE
        (Hysteresis window - ignoring brief dropouts)
t=5s:   5 seconds of continuous absence confirmed
        → face_stable_state = False
        → face_count published as 0
        → person_id stops publishing
t=5-20s: Audio node RED timer counting (no resets)
t=20s:  RED timer expires (15s after stable absence)
        → "Lost you!" beep plays
```

### Timeline: User Returns

```
t=0s:   User enters camera frame (raw detection starts)
t=0-2s: Raw detections may be intermittent
        (Hysteresis window - waiting for stability)
t=2s:   2 seconds of continuous presence confirmed
        → face_stable_state = True
        → face_count published as 1
        → Face recognition starts running
t=2-3s: Face recognition identifies user
        → person_id = "severin"
        → "Hello!" beep plays
```

---

## Testing Instructions

### Quick Test

```bash
cd ~/dev/r2d2
./TEST_FACE_DETECTION_SMOOTHING.sh
```

### Manual Test Steps

1. **Build and restart:**
   ```bash
   cd ~/dev/r2d2/ros2_ws
   colcon build --packages-select r2d2_perception --symlink-install
   sudo systemctl restart r2d2-camera-perception.service
   ```

2. **Monitor (3 terminals):**
   - T1: `ros2 topic echo /r2d2/perception/face_count`
   - T2: `ros2 topic echo /r2d2/perception/person_id`
   - T3: `sudo journalctl -u r2d2-audio-notification -f | grep -E '(Lost you|RED|BLUE)'`

3. **Test scenario:**
   - Stand in front of camera → wait 2s → hear "Hello!"
   - Start conversation (index finger) → speak → stop (fist)
   - Walk away immediately → stay away 25 seconds
   - **EXPECT: "Lost you!" beep at ~20 second mark**
   - Return to camera → wait 2s → hear "Hello!"

### Success Criteria

✅ face_count is stable (0 or 1, no flickering)  
✅ person_id stops publishing after 5s absence  
✅ "Lost you!" beep plays at ~20s (5s + 15s)  
✅ No timer resets during absence window  
✅ "Hello!" beep plays after 2s on return  

---

## Parameters Reference

To adjust timing, modify parameters:

```bash
# Check current values
ros2 param get /image_listener face_presence_threshold
ros2 param get /image_listener face_absence_threshold

# Set new values (runtime)
ros2 param set /image_listener face_presence_threshold 3.0
ros2 param set /image_listener face_absence_threshold 7.0
```

**Tuning Guide:**
- **face_presence_threshold:** Lower = faster "Hello!" but may catch false positives
- **face_absence_threshold:** Higher = longer delay before "Lost you!" but more stable

**Recommended ranges:**
- Presence: 1.0 - 3.0 seconds
- Absence: 3.0 - 7.0 seconds

---

## Technical Architecture

### Data Flow

```
Camera (30fps)
    ↓
Haar Cascade (Raw, Noisy)
    ↓
[Hysteresis Filter]  ← NEW
    ↓
Stable face_count (0 or 1)
    ↓
Face Recognition (gated)
    ↓
person_id → Audio Node
    ↓
15s Timer → "Lost you!" beep
```

### Before vs After

**Before (Simple Timer):**
- Recognition started after 5s detection
- But continued running if ANY face detected
- False detections reset audio timer
- "Lost you!" beep unreliable

**After (Hysteresis):**
- Stable state confirmed after 2s/5s
- Published face_count is stable (no flicker)
- False detections filtered before recognition
- "Lost you!" beep reliable and predictable

---

## Files Modified

1. `ros2_ws/src/r2d2_perception/r2d2_perception/image_listener.py`
   - Added hysteresis parameters (2 new)
   - Replaced simple timer with state machine
   - Implemented transition logic with time thresholds
   - Gated all publishing on stable state

2. `TEST_FACE_DETECTION_SMOOTHING.sh`
   - Updated timing expectations (2s/5s/20s)
   - Updated monitoring instructions
   - Clarified expected behavior

---

## Debugging

### Check if hysteresis is working

```bash
# Monitor stable state transitions (look for "PRESENCE confirmed" / "ABSENCE confirmed")
sudo journalctl -u r2d2-camera-perception.service -f | grep -E "(PRESENCE|ABSENCE)"

# Watch face_count - should be stable (0 or 1 only)
ros2 topic echo /r2d2/perception/face_count
```

### Common issues

**"Lost you!" still doesn't play:**
- Check if person_id is still publishing: `ros2 topic echo /r2d2/perception/person_id`
- If yes, increase `face_absence_threshold` to 7.0 seconds
- Check audio node logs for timer resets

**"Hello!" beep too slow:**
- Reduce `face_presence_threshold` to 1.5 seconds
- Trade-off: may get false positives if too low

**System too sensitive:**
- Increase both thresholds
- Recommendation: 3.0s presence, 7.0s absence

---

## Next Steps (Future Improvements)

1. **Adaptive thresholds:** Adjust based on detection confidence
2. **Multi-face handling:** Track multiple people independently
3. **Confidence-weighted smoothing:** Use detection quality in hysteresis
4. **Metrics:** Log false positive rate and transition frequency

---

## Verification Status

✅ Code implemented and reviewed  
✅ No linting errors  
✅ Parameters loaded correctly (2.0s / 5.0s)  
✅ Service restarted and active  
✅ Node running (/image_listener)  
✅ Test script updated  
⏳ User testing pending  

---

**Ready for testing!** Run `./TEST_FACE_DETECTION_SMOOTHING.sh` to verify the fix.

