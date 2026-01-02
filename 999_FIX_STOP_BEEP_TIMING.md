# Fix: Stop Beep Timing Issue

**Status:** Known Issue - Minor  
**Severity:** Low (cosmetic)  
**Date Identified:** January 2, 2026  
**Affects:** Bluetooth audio output (FreeBuds 4i)

---

## Problem Description

### Symptoms

When stopping a speech session via fist gesture:
- ✅ Fist gesture is **recognized** correctly
- ✅ Session **stops** successfully (status changes to "disconnected")
- ✅ Code **triggers** stop beep (`_play_audio_feedback(self.stop_beep_sound)`)
- ❌ User does **not hear** the stop beep during live flow
- ✅ Stop beep **plays correctly** when tested manually with `ffplay`

### Evidence from Logs

```bash
# Logs show the code path executes correctly:
Jan 02 10:10:12 R2D2 [gesture_intent_node]: ✊ Fist detected → Stopping Fast Mode conversation
Jan 02 10:10:12 R2D2 [gesture_intent_node]: 🔇 Exited SPEAKING state (reason: user_fist_gesture)
Jan 02 10:10:14 R2D2 [gesture_intent_node]: 📡 Session status received: disconnected (active=False, was=True)
Jan 02 10:10:14 R2D2 [gesture_intent_node]: 🔊 Session stopped (status=disconnected)
```

The log line `🔊 Session stopped` confirms that line 256 in `gesture_intent_node.py` executed, which calls:

```python
self._play_audio_feedback(self.stop_beep_sound)  # Voicy_R2-D2 - 20.mp3
```

### Manual Test Confirms Audio File Works

```bash
ffplay -autoexit -nodisp -af 'pan=stereo|c0=c0|c1=c0' \
  ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/Voicy_R2-D2\ -\ 20.mp3
```

✅ User **hears** the stop beep when played manually.

---

## Root Cause Analysis

### Likely Cause: Race Condition

The stop beep `ffplay` process starts, but gets interrupted before audio plays due to:

1. **TTS stream still active:** Speech node is disconnecting, TTS audio stream may still be closing
2. **PulseAudio state transition:** Bluetooth sink transitioning states during disconnect
3. **Process cleanup:** Subprocess.Popen is fire-and-forget; if parent process cleans up quickly, child may be killed

### Timing Details

From session status callback (`gesture_intent_node.py` lines 247-256):

```python
if old_active != self.session_active:
    if self.session_active:
        # Session started
        self._play_audio_feedback(self.start_beep_sound)  # ✅ Works
    else:
        # Session stopped
        self._play_audio_feedback(self.stop_beep_sound)   # ❌ Timing issue
```

**Why start beep works but stop beep doesn't:**
- **Start beep:** Audio system is idle, PulseAudio ready, Bluetooth stable
- **Stop beep:** Audio system is busy closing TTS stream, PulseAudio transitioning, race condition

---

## Solution Options

### Option 1: Add Small Delay Before Stop Beep (Recommended)

**Rationale:** Give audio system time to release TTS stream before playing beep.

**Implementation:**

```python
# In gesture_intent_node.py, session_status_callback (line 253-256)
else:
    # Session stopped (active → inactive or connected → disconnected)
    self.get_logger().info(f'🔊 Session stopped (status={status_str})')
    
    # Add small delay to let audio system stabilize
    import time
    time.sleep(0.3)  # 300ms delay
    
    self._play_audio_feedback(self.stop_beep_sound)
```

**Pros:**
- Simple, minimal code change
- Gives PulseAudio time to transition
- Should resolve race condition

**Cons:**
- Blocks ROS callback for 300ms (acceptable for infrequent event)
- Hardcoded delay value

---

### Option 2: Use Synchronous Audio Playback for Stop Beep

**Rationale:** Ensure stop beep process completes before callback returns.

**Implementation:**

```python
# In gesture_intent_node.py, _play_audio_feedback method
def _play_audio_feedback(self, audio_file: Path, wait: bool = False):
    if not self.audio_feedback_enabled:
        return
    
    if not audio_file.exists():
        self.get_logger().warn(f'Audio file not found: {audio_file}')
        return
    
    try:
        effective_volume = self.master_volume * self.audio_volume
        
        cmd = ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'error',
               '-af', f'pan=stereo|c0=c0|c1=c0,volume={effective_volume}', 
               str(audio_file)]
        
        if wait:
            # Synchronous: wait for completion
            subprocess.run(cmd, stdout=subprocess.DEVNULL, 
                          stderr=subprocess.DEVNULL, timeout=5.0)
        else:
            # Asynchronous: fire and forget
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, 
                           stderr=subprocess.DEVNULL)
    except Exception as e:
        self.get_logger().warn(f'Audio playback error: {e}')
```

**Usage:**

```python
# Start beep (asynchronous - no wait needed)
self._play_audio_feedback(self.start_beep_sound, wait=False)

# Stop beep (synchronous - wait for completion)
self._play_audio_feedback(self.stop_beep_sound, wait=True)
```

**Pros:**
- More robust, ensures beep completes
- No arbitrary delay value
- Process completion guaranteed

**Cons:**
- More complex code change
- Blocks callback longer (duration of beep ~3 seconds)

---

### Option 3: Queue Stop Beep in Separate Thread

**Rationale:** Play stop beep asynchronously without blocking callback.

**Implementation:**

```python
# In gesture_intent_node.py, add threading import
import threading

# Modify session_status_callback
else:
    # Session stopped
    self.get_logger().info(f'🔊 Session stopped (status={status_str})')
    
    # Play stop beep in separate thread after delay
    def delayed_stop_beep():
        time.sleep(0.3)  # Wait for audio to stabilize
        self._play_audio_feedback(self.stop_beep_sound)
    
    threading.Thread(target=delayed_stop_beep, daemon=True).start()
```

**Pros:**
- Non-blocking (callback returns immediately)
- Delay ensures audio system ready
- Clean separation of concerns

**Cons:**
- Introduces threading complexity
- Thread overhead (minimal)

---

## Recommended Fix

**Use Option 1 (Simple Delay) for immediate fix:**

```bash
# Edit gesture_intent_node.py
cd ~/dev/r2d2/ros2_ws/src/r2d2_gesture/r2d2_gesture/

# Add delay before stop beep (around line 256)
# Change from:
#     self._play_audio_feedback(self.stop_beep_sound)
# To:
#     import time
#     time.sleep(0.3)
#     self._play_audio_feedback(self.stop_beep_sound)
```

**Rebuild and test:**

```bash
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_gesture --symlink-install
sudo systemctl restart r2d2-gesture-intent.service

# Test: fist gesture should now play stop beep
```

---

## Alternative Workaround (No Code Changes)

If you don't want to modify code, the stop beep can be **inferred** from:
- Visual feedback: Speech mode status changes to "🔇 OFF" in minimal_monitor
- Session disconnection is still working correctly
- Only the audio confirmation is missing (timing issue)

---

## Testing Procedure

After implementing fix:

1. **Start conversation:** Index finger gesture
   - ✅ Hear acknowledgment beep
   - ✅ Hear start beep

2. **Speak to R2D2:** Say something
   - ✅ Hear TTS response

3. **Stop conversation:** Fist gesture
   - ✅ Hear stop beep ← **This should now work**
   - ✅ Session stops (minimal_monitor shows OFF)

4. **Repeat test 3-5 times** to ensure consistency

---

## Related Files

- `ros2_ws/src/r2d2_gesture/r2d2_gesture/gesture_intent_node.py` (line 256)
- `ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/Voicy_R2-D2 - 20.mp3` (stop beep file)

---

## Impact Assessment

**Severity:** Low
- Core functionality works (session stops correctly)
- Only audio confirmation is missing
- User has visual feedback (monitor shows OFF status)
- Not blocking any workflows

**User Impact:** Minimal
- 1 of 3 beeps missing (start beep works, ack beep works)
- Easy workaround (visual feedback available)

**Recommended Priority:** Medium
- Fix when convenient
- Not urgent for production use
- Simple 5-minute fix with Option 1

---

**Document Created:** January 2, 2026  
**System:** R2D2 Bluetooth Audio (FreeBuds 4i, mono-to-stereo routing)  
**Status:** Documented - Fix ready to implement when desired

