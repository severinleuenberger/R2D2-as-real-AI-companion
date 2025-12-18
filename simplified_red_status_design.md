# Simplified RED Status Design

**Date:** December 18, 2025  
**User Request:** Simpler RED status logic - 15 second timeout with continuous reset  
**Status:** Design ready for implementation  
**Simplification:** Removes complex jitter + confirmation windows

---

## New Simple Rule

**Single Rule:** RED status has a 15-second countdown timer that resets every time target person is recognized.

```
IF target_person recognized:
    last_recognition_time = now()
    status = "red"

IF (now() - last_recognition_time) > 15 seconds:
    status = "blue"
```

**That's it.** No jitter tolerance, no loss confirmation windows, no complex state tracking.

---

## Simplified State Machine

```
BLUE (idle)
  ↓ target_person recognized
RED (recognized)
  - Timer: 15 seconds
  - Reset: Every time target_person seen
  - Timeout: If not seen for 15s → BLUE
  ↓ 15 seconds without recognition
BLUE (lost)
```

---

## Comparison: Old vs New

| Aspect | Current (Complex) | Proposed (Simple) | Benefit |
|--------|------------------|-------------------|---------|
| **Recognition Logic** | Jitter tolerance (5s) + Loss confirmation (15s) = 20s total | Single 15s timeout | Simpler, faster |
| **Timer Mechanism** | Two timers: jitter_exceeded_time + loss_time | One timer: last_recognition_time | Less state tracking |
| **Reset Condition** | Only during jitter window | Every recognition | More responsive |
| **Transition Time** | 20 seconds (5s + 15s) | 15 seconds | Faster feedback |
| **State Variables** | 4 variables (is_recognized, last_time, jitter_time, loss_time) | 1 variable (last_recognition_time) | Much simpler |
| **Code Complexity** | ~80 lines | ~15 lines | Easier to debug |

---

## Simplified Code Implementation

### Complete Replacement for audio_notification_node

**File:** `ros2_ws/src/r2d2_audio/r2d2_audio/audio_notification_node.py`

**Remove these parameters (lines 89-90):**
```python
# DELETE:
self.declare_parameter('jitter_tolerance_seconds', 5.0)
self.declare_parameter('loss_confirmation_seconds', 15.0)
```

**Add this parameter:**
```python
# Line 89 - ADD:
self.declare_parameter('red_status_timeout_seconds', 15.0)  # Simple timeout
```

**Remove these state variables (lines 110-115):**
```python
# DELETE all of these:
self.is_currently_recognized = False
self.loss_jitter_exceeded_time = None
self.last_loss_beep_time = None
self.loss_alert_time = None
# Keep only:
self.last_recognition_time = None  # ← KEEP THIS ONE
```

**Add simple state variables:**
```python
# Lines 110-115 - REPLACE WITH:
self.last_recognition_time = None  # Last time target person seen
self.red_status_timeout = self.get_parameter('red_status_timeout_seconds').value
self.last_recognition_beep_time = None  # Cooldown for recognition beep
self.last_loss_beep_time = None  # Cooldown for loss beep
```

**Simplify person_callback (lines 238-342):**
```python
def person_callback(self, msg: String):
    """
    Handle person_id messages from face recognition.
    Simple 15-second timeout logic.
    """
    person_id = msg.data
    current_time = time.time()
    
    if not self.enabled:
        return
    
    if person_id == self.target_person:
        # Target person recognized - reset timer
        was_red = (self.current_status == "red")
        self.last_recognition_time = current_time
        
        if not was_red:
            # Transition: BLUE/GREEN → RED
            old_status = self.current_status
            self.current_status = "red"
            self.current_person = self.target_person
            self.status_changed_time = current_time
            self.unknown_person_detected = False
            
            # Publish status FIRST
            self._publish_status("red", self.target_person, confidence=0.95)
            
            # Play "Hello!" beep (with cooldown)
            if self.last_recognition_beep_time is None or \
               (current_time - self.last_recognition_beep_time) >= self.cooldown_seconds:
                self._play_audio_alert(self.recognition_audio)
                self.last_recognition_beep_time = current_time
                self._publish_event(f"🎉 Recognized {self.target_person}!")
            
            self.get_logger().info(f"✓ {self.target_person} recognized ({old_status} → RED)")
        else:
            # Already RED - just update timestamp
            # Publish to update duration
            self._publish_status("red", self.target_person, confidence=0.95)
    
    elif person_id == "unknown":
        # Unknown person detected
        if self.current_status != "green":
            self.current_status = "green"
            self.current_person = "unknown"
            self.status_changed_time = current_time
            self.unknown_person_detected = True
            self._publish_status("green", "unknown", confidence=0.70)
            self.get_logger().info("🟢 Unknown person detected")
        else:
            self._publish_status("green", "unknown", confidence=0.70)

def check_loss_state(self):
    """
    Timer callback: Simple 15-second timeout check.
    If no target_person seen for 15 seconds, transition to BLUE.
    """
    if not self.enabled:
        return
    
    if self.last_recognition_time is None:
        return  # Never recognized yet
    
    if self.current_status != "red":
        return  # Not in RED state, nothing to check
    
    current_time = time.time()
    time_since_recognition = current_time - self.last_recognition_time
    
    if time_since_recognition > self.red_status_timeout:
        # 15 seconds elapsed without seeing target person
        # Transition to BLUE
        self.current_status = "blue"
        self.current_person = "no_person"
        self.status_changed_time = current_time
        self.unknown_person_detected = False
        
        # Publish status FIRST
        self._publish_status("blue", "no_person", confidence=0.0)
        
        # Play "Lost you!" beep (with cooldown)
        if self.last_loss_beep_time is None or \
           (current_time - self.last_loss_beep_time) >= self.cooldown_seconds:
            self._play_audio_alert(self.loss_audio)
            self.last_loss_beep_time = current_time
            self._publish_event(f"❌ {self.target_person} lost (after {time_since_recognition:.1f}s)")
        
        self.get_logger().info(
            f"✗ {self.target_person} lost (no recognition for {time_since_recognition:.1f}s)"
        )
```

**That's the complete simplified implementation!**

---

## How It Works

### Visual Timeline

```
t=0s:   target_person recognized
        → last_recognition_time = 0s
        → status = "red"
        
t=3s:   target_person recognized again
        → last_recognition_time = 3s  (RESET)
        → status = "red"
        
t=8s:   target_person recognized again
        → last_recognition_time = 8s  (RESET)
        → status = "red"
        
t=12s:  target_person recognized again
        → last_recognition_time = 12s  (RESET)
        → status = "red"
        
t=20s:  No recognition since t=12s
        → time_since = 8s (< 15s)
        → status = "red"  (still valid)
        
t=27s:  Still no recognition
        → time_since = 15s (TIMEOUT!)
        → status = "blue"
        → Play "Lost you!" beep
```

### Key Behavior

**Continuous Recognition:**
- User in front of camera
- Face recognized at 6.5 Hz (every ~154ms)
- last_recognition_time resets continuously
- RED status stays active indefinitely

**Brief Absence (< 15s):**
- User turns head for 10 seconds
- No recognition during turn
- Timer at 10s (< 15s threshold)
- User returns, face recognized → Timer resets
- RED status never lost

**Actual Departure (> 15s):**
- User walks away
- No recognition for 15+ seconds
- Timer reaches 15s threshold
- Status transitions RED → BLUE
- "Lost you!" beep plays

---

## Benefits of Simplified Design

1. **Easier to understand** - One timer, one rule
2. **Faster feedback** - 15s vs 20s for loss detection
3. **More predictable** - Behavior is obvious from code
4. **Less state** - Fewer variables to track
5. **Easier debugging** - Simple timer to monitor
6. **Still protective** - 15s is enough for brief face loss

---

## Integration with SPEAKING State

**Perfect combination:**

**RED Status (audio_notification_node):**
- Simple 15s timeout
- Handles face recognition stability
- Publishes person_status

**SPEAKING State (gesture_intent_node):**
- Separate 35s consecutive non-RED protection
- Handles conversation continuation
- Independent of RED status timer

**Together:**
- RED can change after 15s (face not seen)
- But SPEAKING protects conversation for 35s consecutive non-RED
- Result: Best of both worlds

**Example:**
```
t=0s:   User starts conversation (SPEAKING state)
        status = "red"
        
t=10s:  User turns away, no face recognition
        status still "red" (within 15s timeout)
        SPEAKING: consecutive_nonred not started (still RED)
        
t=16s:  Still no recognition for 16s total
        status → "blue" (15s timeout expired)
        SPEAKING: consecutive_nonred starts counting
        
t=20s:  User returns, face recognized
        status → "red" (immediate)
        SPEAKING: consecutive_nonred resets (only was 4s)
        Conversation continues ✓
```

---

## Code Changes Summary

### File: audio_notification_node.py

**Lines to REMOVE:**
- Line 89: `jitter_tolerance_seconds` parameter
- Line 90: `loss_confirmation_seconds` parameter
- Lines 110-115: Complex state variables
- Lines 397-479: Old `check_loss_state` method

**Lines to ADD:**
- Line 89: `red_status_timeout_seconds` parameter (15.0)
- Lines 110-112: Simple state variables
- Lines 238-280: Simplified `person_callback`
- Lines 397-430: Simplified `check_loss_state`

**Net change:** -70 lines (simpler code!)

---

## Testing the Simplified Logic

### Test 1: Continuous Recognition

```bash
# Monitor person_status while standing in front of camera
ros2 topic echo /r2d2/audio/person_status | grep status

# EXPECT: Continuous "red" status
# EXPECT: Never switches to "blue" while present
```

### Test 2: Brief Absence

```bash
# 1. Stand in camera until RED
# 2. Turn away for 10 seconds
# 3. Turn back

# EXPECT: Status stays RED (10s < 15s timeout)
# EXPECT: No "Lost you!" beep
```

### Test 3: Actual Departure

```bash
# 1. Stand in camera until RED
# 2. Walk away completely
# 3. Monitor logs

# EXPECT: After 15 seconds, status → BLUE
# EXPECT: "Lost you!" beep at 15s mark
# EXPECT: Log: "lost (no recognition for 15.Xs)"
```

---

## Final Simplified Architecture

**RED Status Timer:**
- Simple 15-second countdown
- Resets on every target_person recognition
- Switches to BLUE after 15s without recognition

**SPEAKING State Protection:**
- Separate 35-second consecutive non-RED protection
- Tracks consecutive time (resets when RED seen)
- Protects conversations from brief RED loss

**Combined Effect:**
- RED changes after 15s no recognition
- But SPEAKING continues for 35s consecutive non-RED
- User has up to 35s to return to camera during conversation
- Maximum protection: 15s + 35s = 50s total (if RED lost exactly when conversation started)

---

## Implementation Priority

**Step 1:** Implement simplified RED status (this design)
**Step 2:** Implement SPEAKING state (from speaking_state_design.md)
**Step 3:** Test both together

**Or implement both simultaneously** since they're independent:
- RED status: audio_notification_node
- SPEAKING state: gesture_intent_node
- No conflicts, complementary systems

---

## Ready-to-Apply Code

### Complete Simplified person_callback

```python
def person_callback(self, msg: String):
    """
    Handle person_id messages - SIMPLIFIED 15-second timeout logic.
    
    Rule: If target_person recognized, reset 15s timer. If timer expires, switch to BLUE.
    """
    person_id = msg.data
    current_time = time.time()
    
    if not self.enabled:
        return
    
    if person_id == self.target_person:
        # Target person recognized - reset 15s countdown
        was_red = (self.current_status == "red")
        self.last_recognition_time = current_time  # ← RESET TIMER
        
        if not was_red:
            # Transition to RED
            old_status = self.current_status
            self.current_status = "red"
            self.current_person = self.target_person
            self.status_changed_time = current_time
            
            # Publish RED status
            self._publish_status("red", self.target_person, confidence=0.95)
            
            # Play "Hello!" beep (with cooldown)
            current_time_beep = time.time()
            if self.last_recognition_beep_time is None or \
               (current_time_beep - self.last_recognition_beep_time) >= self.cooldown_seconds:
                self._play_audio_alert(self.recognition_audio)
                self.last_recognition_beep_time = current_time_beep
                self._publish_event(f"🎉 Recognized {self.target_person}!")
            
            self.get_logger().info(f"✓ {self.target_person} recognized ({old_status} → RED)")
        else:
            # Already RED - just reset timer and update duration
            self._publish_status("red", self.target_person, confidence=0.95)
            self.get_logger().debug(f"RED: Timer reset (target_person seen)")
    
    elif person_id == "unknown":
        # Unknown person detected - switch to GREEN
        if self.current_status != "green":
            self.current_status = "green"
            self.current_person = "unknown"
            self.status_changed_time = current_time
            self._publish_status("green", "unknown", confidence=0.70)
            self.get_logger().info("🟢 Unknown person detected")
        else:
            self._publish_status("green", "unknown", confidence=0.70)

def check_loss_state(self):
    """
    Timer callback: SIMPLIFIED 15-second timeout check.
    
    If in RED state and no target_person recognized for 15 seconds, switch to BLUE.
    Runs every 500ms to check timeout.
    """
    if not self.enabled:
        return
    
    if self.last_recognition_time is None:
        return  # Never recognized yet, nothing to check
    
    if self.current_status != "red":
        return  # Not in RED state, nothing to timeout
    
    current_time = time.time()
    time_since_recognition = current_time - self.last_recognition_time
    
    if time_since_recognition > self.red_status_timeout:  # 15 seconds
        # Timer expired - switch to BLUE
        self.current_status = "blue"
        self.current_person = "no_person"
        self.status_changed_time = current_time
        
        # Publish BLUE status
        self._publish_status("blue", "no_person", confidence=0.0)
        
        # Play "Lost you!" beep (with cooldown)
        if self.last_loss_beep_time is None or \
           (current_time - self.last_loss_beep_time) >= self.cooldown_seconds:
            self._play_audio_alert(self.loss_audio)
            self.last_loss_beep_time = current_time
            self._publish_event(f"❌ {self.target_person} lost (no recognition for {time_since_recognition:.1f}s)")
        
        self.get_logger().info(
            f"✗ {self.target_person} lost (timer expired: {time_since_recognition:.1f}s > {self.red_status_timeout}s)"
        )
```

---

## Changed Parameters

**REMOVED:**
- `jitter_tolerance_seconds` (no longer needed)
- `loss_confirmation_seconds` (no longer needed)
- `recognition_cooldown_after_loss_seconds` (can keep for "Hello!" beep cooldown after loss)

**ADDED:**
- `red_status_timeout_seconds` (default: 15.0)

**KEPT:**
- `cooldown_seconds` (for preventing beep spam)
- `target_person`
- `audio_volume`
- All audio file parameters

---

## Logging Output (Simplified)

**When recognized:**
```
[INFO] ✓ severin recognized (blue → RED)
[DEBUG] RED: Timer reset (target_person seen)
[DEBUG] RED: Timer reset (target_person seen)
```

**When lost:**
```
[INFO] ✗ severin lost (timer expired: 15.2s > 15.0s)
```

**Much cleaner!**

---

## Testing Strategy

### Monitor Timer Resets

```bash
# Watch logs with timing
sudo journalctl -u r2d2-audio-notification -f | grep "Timer reset\|lost\|recognized"
```

**Expected output during continuous presence:**
```
[DEBUG] RED: Timer reset (target_person seen)
[DEBUG] RED: Timer reset (target_person seen)
[DEBUG] RED: Timer reset (target_person seen)
# Repeats every ~154ms (6.5 Hz recognition rate)
```

### Verify 15-Second Timeout

```bash
# 1. Stand in camera until RED
# 2. Walk away at t=0s
# 3. Watch clock

# At t=15s: EXPECT status → BLUE and "Lost you!" beep
```

### Verify Timer Resets

```bash
# 1. Stand in camera (RED)
# 2. Turn away for 10s (t=0 to t=10)
# 3. Turn back at t=10s
# 4. Repeat several times

# EXPECT: Status stays RED throughout (timer resets each time)
```

---

## Backward Compatibility

**Launch files that override parameters:**
- Remove `jitter_tolerance_seconds:=X`
- Remove `loss_confirmation_seconds:=X`
- Add `red_status_timeout_seconds:=15.0` (or omit for default)

**Service files:**
- No changes needed (will use new default)

**Code using old parameters:**
- Will get parameter not found error
- But gracefully handled (uses defaults)

---

## Comparison with SPEAKING State

**These are two separate, complementary protections:**

| System | Component | Timeout | Resets On | Purpose |
|--------|-----------|---------|-----------|---------|
| **RED Status** | audio_notification_node | 15s simple timeout | target_person recognized | Face recognition stability |
| **SPEAKING Protection** | gesture_intent_node | 35s consecutive non-RED | person_status = "red" | Conversation continuity |

**Why both are needed:**

**Scenario: User speaking while looking away**
```
t=0s:   Conversation active, RED status
t=5s:   User looks at notes (face not visible)
        → No recognition from t=5s onward
        
t=20s:  (15s without recognition)
        → RED status → BLUE (15s timeout)
        → BUT SPEAKING state protects conversation
        → consecutive_nonred_start_time = t=20s
        
t=30s:  User still looking at notes
        → BLUE status continues
        → SPEAKING: 10s non-RED (still protected, < 35s)
        
t=55s:  User still away
        → BLUE status
        → SPEAKING: 35s non-RED (TIMEOUT!)
        → Conversation auto-stops
```

**Result:** User has 35 seconds to return during conversation, even though RED changes after 15s

---

## Summary of Simplified Changes

**What's Simpler:**
- One timer instead of two (jitter + confirmation)
- One timeout value (15s) instead of two (5s + 15s)
- Fewer state variables (1 vs 4)
- Cleaner code (~40 lines vs ~110 lines)
- Faster transition (15s vs 20s)

**What's the Same:**
- RED/BLUE/GREEN state machine
- "Hello!" and "Lost you!" beeps
- Cooldown for preventing beep spam
- 10 Hz status publishing
- LED synchronization

**What's Better:**
- More responsive (15s vs 20s)
- More predictable (simple rule)
- Easier to tune (one parameter)
- Timer resets continuously (not just in jitter window)

---

## Implementation Order

**Recommended:**

1. **First:** Implement simplified RED status (this design)
   - Cleaner, simpler foundation
   - Test independently
   
2. **Second:** Implement SPEAKING state (previous design)
   - Builds on simplified RED status
   - Provides conversation protection

3. **Third:** Implement auto_start=false
   - Fixes boot issue
   - Completes gesture-triggered operation

4. **Test:** Complete integration testing
   - All three working together
   - Verify UX goals met

---

**Document Version:** 1.0  
**Date:** December 18, 2025  
**Status:** Simplified design ready  
**Code:** Complete replacement provided above  
**Complexity:** Much simpler than original  
**Ready:** For immediate implementation

