# RED Status Stability Analysis

**Date:** December 18, 2025  
**Issue:** RED status may not be stable enough during conversations  
**User Concern:** "minimum 15 second" stability required, current 5s jitter may be too short  
**Impact:** RED status flickering could interrupt speech-to-speech conversations

---

## Executive Summary

**CRITICAL ISSUE FOUND:** The current RED status stability mechanism has a fundamental flaw that can interrupt active conversations:

1. **Current jitter tolerance: 5 seconds** - Too short for conversation protection
2. **No distinction between idle and speaking states** - Treats active conversations same as idle waiting
3. **Watchdog triggers immediately on non-RED** - Doesn't account for conversation continuity
4. **Face recognition frame skipping** - Can cause multi-second gaps in person_id publishing

**Result:** If face recognition hiccups during conversation (head turn, lighting change, occlusion), RED status may transition to BLUE/GREEN, which could interrupt the speech session.

---

## Current RED Status Stability Mechanism

### How RED Status Works (Current Implementation)

```
Face Recognition publishes person_id at 6.5 Hz (every ~154ms)
  ↓
audio_notification_node receives person_id
  ↓
IF person_id == target_person:
  last_recognition_time = now()
  current_status stays "red"
  
ELSE (person_id == "unknown" or no message):
  # Timer check_loss_state runs every 500ms
  time_since_last = now() - last_recognition_time
  
  IF time_since_last > 5.0 seconds:
    # Jitter tolerance exceeded
    IF time_since_last > 20.0 seconds (5s + 15s):
      # Loss confirmed
      current_status = "blue"
      Trigger "Lost you!" beep
```

**Key Behavior:**
- **RED is maintained** as long as `last_recognition_time` was updated within last 5 seconds
- **RED transitions to BLUE** if no person_id received for 20 seconds total (5s jitter + 15s confirmation)
- **Timer checks every 500ms** for loss detection

### Face Recognition Publishing Rate

**From code analysis:**

```python
# image_listener.py lines 311-352
if self.recognition_enabled and face_count > 0:
    self.recognition_frame_counter += 1
    if self.recognition_frame_counter >= self.recognition_frame_skip:  # Default: 2
        self.recognition_frame_counter = 0
        # Process face recognition
        # Publish person_id
```

**Actual publishing:**
- Camera: 30 FPS
- Face recognition: Every 2nd frame = 15 FPS processing attempts
- Actual person_id publishes: Only when face detected AND recognized = ~6.5 Hz (observed)

**Gap Detection:**
- If 2 consecutive frames fail to detect face: ~133ms gap (2 frames at 15 FPS)
- If 10 consecutive frames fail: ~667ms gap
- If 75 consecutive frames fail: ~5 seconds → Jitter tolerance exceeded → Start loss countdown

**Implications:**
- Brief head turns (0.5-1s) covered by jitter tolerance ✓
- Medium occlusions (2-4s) covered by jitter tolerance ✓
- Long occlusions (>5s) trigger loss detection ⚠️

---

## Problem 1: Face Recognition Hiccups During Conversation

### Scenario: User Turns Head While Speaking

```
t=0s:   User speaking, face forward, RED status
t=1s:   User turns head 45° to gesture, face not detected for 1 second
        → person_id not published for ~15 frames
        → last_recognition_time not updated
        → RED status MAINTAINED (within 5s jitter)
t=2s:   Face detected again, person_id = target_person
        → last_recognition_time updated
        → RED status continues ✓

Result: ✅ Brief head turn covered by jitter tolerance
```

### Scenario: User Looks Down While Thinking

```
t=0s:   User speaking, RED status
t=1s:   User looks down, face not visible for 3 seconds
        → person_id not published
        → last_recognition_time not updated
        → RED status MAINTAINED (within 5s jitter)
t=4s:   Face detected again
        → last_recognition_time updated
        → RED status continues ✓

Result: ✅ Brief look-away covered by jitter tolerance
```

### Scenario: User Walks Away Briefly Then Returns

```
t=0s:   User speaking, RED status
t=1s:   User walks away (out of frame) for 7 seconds
        → person_id stops publishing
        → last_recognition_time not updated
t=6s:   Jitter tolerance exceeded (5s)
        → loss_jitter_exceeded_time = t=6s
        → Still in RED (waiting for 15s confirmation)
t=8s:   User returns, face detected
        → person_id = target_person
        → last_recognition_time updated
        → loss_jitter_exceeded_time reset
        → RED status continues ✓

Result: ✅ Brief absence (< 20s total) doesn't trigger loss
```

### Scenario: Lighting Change Causes Recognition Failure

```
t=0s:   User speaking, RED status, conversation active
t=1s:   Bright sunlight through window, face detected but not recognized
        → person_id = "unknown" for 6 seconds
        → last_recognition_time NOT updated
        → current_status transitions to GREEN immediately ⚠️
t=6s:   Jitter tolerance exceeded
        → loss_jitter_exceeded_time = t=6s
        → Status is GREEN (unknown person)
t=7s:   Face recognized again (lighting adjusted)
        → person_id = target_person
        → last_recognition_time updated
        → Status transitions back to RED ✓

Result: ⚠️ Status flickered RED → GREEN → RED
        ⚠️ Does this affect watchdog?
```

---

## Problem 2: Current Watchdog Doesn't Protect Conversations

### Current Watchdog Logic

```python
# gesture_intent_node.py lines 295-342
def watchdog_callback(self):  # Every 10 seconds
    if self.person_status != "red":
        # Person absent - start timer
        if self.last_red_status_time is None:
            self.last_red_status_time = now()
        
        time_since_red = now() - self.last_red_status_time
        
        if time_since_red > self.auto_shutdown_timeout:  # 35 seconds
            if self.session_active:
                self._stop_session()  # Auto-stop
```

**Critical Issue:** Watchdog checks `person_status != "red"` without considering:
1. Whether conversation is active
2. How long conversation has been active
3. Whether person_status briefly flickered

### Scenario: Status Flicker During Conversation

```
t=0s:   User speaking, RED status, session active
t=10s:  Face recognition hiccup (lighting/angle)
        → person_status briefly goes to GREEN for 2 seconds
        → Watchdog sees person_status != "red"
        → last_red_status_time = t=10s (starts counting)
t=12s:  Face recognized again
        → person_status back to RED
        → Watchdog resets: last_red_status_time = None
        
Result: ✅ Brief flicker doesn't trigger shutdown (watchdog reset < 35s)

BUT: If flickering happens repeatedly, could accumulate timing issues
```

### Scenario: User Speaks While Facing Away

```
t=0s:   User starts conversation, RED status
t=5s:   User turns to look at something while speaking
        → Face not visible for 8 seconds
        → person_status = BLUE (no face) or GREEN (side view unrecognized)
t=6s:   Watchdog check: person_status != "red"
        → last_red_status_time = t=6s
t=13s:  User turns back, face recognized
        → person_status = RED
        → Watchdog reset
        
Result: ✅ Conversation continues (returned before 35s)

BUT: Conversation is at risk if user doesn't look back quickly
```

---

## Gap Analysis: RED Stability Requirements

### User's Requirements (Interpreted)

**"Minimum 15 second delay" could mean:**

**Interpretation A:** RED should stay RED for minimum 15 seconds after first detection
- Once RED, stay RED for at least 15s even if face lost
- Prevents rapid RED→BLUE transitions
- Protects brief absences

**Interpretation B:** Jitter tolerance should be 15 seconds instead of 5 seconds
- Allow 15s of face loss before considering person absent
- Current 5s is too short for conversation protection
- Increases from 5s to 15s

**Interpretation C:** During SPEAKING, RED should be "locked" for stability
- When conversation active, don't transition out of RED easily
- Maintain RED status during active conversations
- Separate rules for idle vs speaking

**My Analysis:** User likely means **B + C combined**:
- Increase jitter tolerance to 15s (more stability)
- Add SPEAKING state protection (don't transition during conversation)

### Current Implementation vs Requirements

| Requirement | Current | Gap | Impact |
|-------------|---------|-----|--------|
| **Stable RED during conversation** | RED can flicker if face lost >5s | ❌ Too sensitive | May interrupt conversation |
| **Minimum RED duration** | No minimum, can change immediately | ❌ Missing | Rapid state changes |
| **Jitter tolerance** | 5 seconds | ⚠️ May be too short | Brief absences trigger loss countdown |
| **Conversation protection** | None - watchdog uses person_status directly | ❌ Missing | Active conversations not protected |
| **SPEAKING state** | No separate state, only boolean | ❌ Missing | No conversation-specific rules |

---

## Gap Analysis: SPEAKING State Requirements

### User's SPEAKING State Concept

**Rules (From User Description):**

1. **Entry Condition:**
   - index_finger_up gesture detected
   - AND person_status = "red"
   - → Enter SPEAKING state

2. **SPEAKING State Properties:**
   - Minimum duration: 35 seconds (protected)
   - RED status monitoring: Continue even if RED briefly lost
   - Independent state from person_status

3. **Exit Conditions:**
   - **Immediate:** Fist gesture (user explicitly stops)
   - **Timeout:** Non-RED status for 35 consecutive seconds
   - **Explicit:** Service call to stop_session

**Current Implementation:**
- ✅ Entry: index_finger_up → start_session
- ❌ No SPEAKING state - only `session_active` boolean
- ❌ Watchdog uses person_status directly, not conversation duration
- ✅ Exit: Fist gesture → stop_session (works)
- ⚠️ Exit: Watchdog after 35s non-RED (but doesn't protect conversation)

### State Machine Comparison

**CURRENT (Boolean only):**
```
session_active = false  (Idle)
  ↓ index_finger_up
session_active = true  (Active)
  ↓ fist OR watchdog triggers
session_active = false  (Idle)
```

**PROPOSED (SPEAKING State):**
```
IDLE
  ↓ index_finger_up + person_status="red"
SPEAKING (Protected)
  - Timer: Started_at timestamp
  - Protection: Ignore brief non-RED
  - Duration: At least 35s before auto-stop eligible
  ↓ fist (immediate) OR non-RED for 35s consecutive
IDLE
```

---

## Task 1: RED Status Stability Analysis

### Finding 1: Jitter Tolerance is Too Short for Conversations

**Current:** 5 seconds jitter tolerance
- Adequate for: Brief head turns, quick glances away
- Inadequate for: Active conversations where user may look away while thinking/gesturing

**Recommendation:** Increase to 15 seconds
- Better conversation protection
- Allows user to look away briefly while speaking
- Still detects actual departures (15s + 15s = 30s total before BLUE)

### Finding 2: No Conversation-Aware State Transitions

**Current Logic:**
```python
# audio_notification_node treats all RED→BLUE transitions the same
# Doesn't know if conversation is active
# Doesn't protect active conversations differently
```

**Problem:** During active conversation, RED status should be "stickier"
- User is speaking → Should tolerate longer face loss
- User is idle → Current behavior OK

**Recommendation:** Audio notification node should be aware of conversation state
- Subscribe to `/r2d2/speech/session_status`
- Apply different jitter tolerance when session active
- Example: 5s jitter when idle, 15s jitter when speaking

### Finding 3: GREEN State Handling During Conversations

**Current Logic:**
```python
elif person_id == "unknown":
    current_status = "green"
    # Does NOT reset is_currently_recognized
    # Loss timer continues running
```

**Problem:** If user's face not recognized (lighting, angle) while speaking:
- Status transitions to GREEN
- `is_currently_recognized` stays true (good)
- But status is no longer RED (may affect gesture gating)

**Question:** Should GREEN be treated same as BLUE for watchdog purposes?
- Current: Watchdog triggers on `person_status != "red"` (includes GREEN)
- Proposed: During SPEAKING, treat GREEN as "person still present, just not recognized"

---

## Task 2: Watchdog Behavior During Conversations

### Current Watchdog Logic Analysis

```python
def watchdog_callback(self):  # Every 10 seconds
    if self.person_status != "red":
        # ANY non-RED status triggers timer
        if self.last_red_status_time is None:
            self.last_red_status_time = now()
        
        time_since_red = now() - self.last_red_status_time
        
        if time_since_red > 35.0:  # After 35 seconds
            if self.session_active:  # If conversation active
                self._stop_session()  # Stop it
```

### Problem: No Conversation Protection

**Current Behavior:**
1. User speaking, face not detected for ANY reason (turn head, look away, lighting)
2. person_status != "red" (even if just briefly)
3. Watchdog timer starts immediately
4. If person_status stays non-RED for 35 consecutive seconds, conversation stops

**Issue:** No protection for active conversations
- Brief face loss (5-10s) should not start shutdown timer if speaking
- Conversation should continue if user is present but face temporarily not recognized
- Only stop if user ACTUALLY left (not just face lost)

### Desired Watchdog Behavior During SPEAKING

**Proposed Logic:**
```python
if self.session_active:  # Conversation is active
    # Use SPEAKING-specific protection
    if self.person_status != "red":
        # Start conversation protection timer
        if self.conversation_protection_start_time is None:
            self.conversation_protection_start_time = now()
        
        time_in_non_red = now() - self.conversation_protection_start_time
        
        # Only stop if non-RED for 35 CONSECUTIVE seconds
        if time_in_non_red > 35.0:
            self._stop_session()
    else:
        # Back to RED - reset protection timer
        self.conversation_protection_start_time = None

else:  # Not speaking (idle)
    # Current behavior OK
    # No need to track - no conversation to protect
```

---

## Task 3: SPEAKING State Design

### Proposed SPEAKING State Machine

```
STATE: IDLE
  - Monitoring: person_status only
  - Gestures: Enabled when RED
  - Protection: None
  
  TRANSITION: index_finger_up gesture + person_status="red"
    → CALL start_session service
    → ENTER SPEAKING state
  
STATE: SPEAKING
  - Monitoring: person_status + conversation duration
  - Gestures: Stop gesture enabled (fist)
  - Protection: 35-second consecutive non-RED required for auto-stop
  - Minimum Duration: Can end anytime BUT watchdog protection is 35s
  
  Properties:
    - speaking_start_time: Timestamp when entered SPEAKING
    - last_red_in_speaking_time: Last time saw RED during SPEAKING
    - consecutive_nonred_start_time: When non-RED period started
  
  TRANSITIONS:
    → IMMEDIATE: Fist gesture (user explicit stop)
    → TIMEOUT: Non-RED for 35 consecutive seconds
    → EXPLICIT: stop_session service call
    → ERROR: Speech service failure
  
  Special Rules:
    - Brief RED loss (< 35s): Continue SPEAKING, don't stop
    - RED returns: Reset consecutive_nonred timer
    - During SPEAKING: More tolerant of face recognition failures
```

### State Variables Needed

```python
# In gesture_intent_node.py:
self.speaking_state = "idle"  # "idle" or "speaking"
self.speaking_start_time = None  # When SPEAKING state entered
self.consecutive_nonred_start_time = None  # When current non-RED period started
self.last_red_in_speaking_time = None  # Last time saw RED during SPEAKING
```

### State Transition Logic

```python
# When index_finger_up detected and gates pass:
def _enter_speaking_state(self):
    self.speaking_state = "speaking"
    self.speaking_start_time = now()
    self.last_red_in_speaking_time = now()
    self.consecutive_nonred_start_time = None
    self._start_session()

# When person_status updates during SPEAKING:
def _update_speaking_state(self, person_status):
    if person_status == "red":
        # RED during SPEAKING - reset non-RED timer
        self.last_red_in_speaking_time = now()
        self.consecutive_nonred_start_time = None
    else:
        # Non-RED during SPEAKING
        if self.consecutive_nonred_start_time is None:
            # Start counting consecutive non-RED time
            self.consecutive_nonred_start_time = now()
        
        # Check if exceeded protection threshold
        consecutive_time = now() - self.consecutive_nonred_start_time
        if consecutive_time > 35.0:
            # Been non-RED for 35 consecutive seconds
            self._exit_speaking_state("timeout")

# When fist gesture detected:
def _exit_speaking_state(self, reason):
    self.speaking_state = "idle"
    self.speaking_start_time = None
    self.consecutive_nonred_start_time = None
    self._stop_session()
```

---

## Task 4: Implementation Gaps Identified

### Gap 1: Jitter Tolerance Too Short

**Current:** 5 seconds
**Required:** 15 seconds minimum for conversation stability
**Fix:** Change parameter default

### Gap 2: No SPEAKING State

**Current:** Only `session_active` boolean
**Required:** Full SPEAKING state with protection rules
**Fix:** Add state machine to gesture_intent_node

### Gap 3: Watchdog Not Conversation-Aware

**Current:** Triggers immediately on any non-RED
**Required:** 35s consecutive non-RED during SPEAKING before stopping
**Fix:** Modify watchdog to use SPEAKING state

### Gap 4: Audio Notification Not Session-Aware

**Current:** audio_notification_node doesn't know if conversation active
**Required:** Should apply different rules during active conversations
**Fix:** Subscribe to `/r2d2/speech/session_status` in audio_notification_node

### Gap 5: GREEN Status During Conversations

**Current:** person_status = "green" triggers watchdog timer (same as BLUE)
**Required:** During SPEAKING, GREEN should be treated as "person present but temporarily unrecognized"
**Fix:** Distinguish between BLUE (nobody) and GREEN (somebody but unknown) in watchdog

---

## Task 5: Prepared Fix Code (NOT IMPLEMENTED YET)

### Fix A: Increase Jitter Tolerance to 15 Seconds

**File:** `ros2_ws/src/r2d2_audio/r2d2_audio/audio_notification_node.py`

```python
# Line 89 - Change default:
self.declare_parameter('jitter_tolerance_seconds', 15.0)  # Was: 5.0

# Rationale: 15 seconds allows:
# - User to look away while thinking
# - Brief occlusions (hand in front of face)
# - Lighting adjustments
# - Head turns during conversation
# Still detects actual departures (15s + 15s = 30s total)
```

**Also update:**
- Service file if it overrides: `r2d2-audio-notification.service`
- Launch file defaults
- Documentation

### Fix B: Add SPEAKING State to gesture_intent_node

**File:** `ros2_ws/src/r2d2_gesture/r2d2_gesture/gesture_intent_node.py`

**Add state variables (after line 81):**
```python
# SPEAKING state tracking (NEW)
self.speaking_state = "idle"  # "idle" or "speaking"
self.speaking_start_time = None  # When SPEAKING started
self.consecutive_nonred_start_time = None  # When current non-RED period started
self.last_red_in_speaking_time = None  # Last RED seen during SPEAKING
```

**Modify gesture_callback (around line 229):**
```python
# After all gates pass for index_finger_up:
self.get_logger().info('🤚 Index finger up detected → Starting conversation')
self._enter_speaking_state()  # NEW: Enter SPEAKING state
self.last_trigger_time = current_time
```

**Modify gesture_callback for fist (around line 252):**
```python
# After all gates pass for fist:
self.get_logger().info('✊ Fist detected → Stopping conversation')
self._exit_speaking_state(reason="user_gesture")  # NEW: Exit SPEAKING state
self.last_trigger_time = current_time
```

**Add new methods:**
```python
def _enter_speaking_state(self):
    """Enter SPEAKING state when conversation starts."""
    self.speaking_state = "speaking"
    self.speaking_start_time = self.get_clock().now()
    self.last_red_in_speaking_time = self.get_clock().now()
    self.consecutive_nonred_start_time = None
    self._start_session()
    self.get_logger().info('🗣️  Entered SPEAKING state')

def _exit_speaking_state(self, reason: str):
    """Exit SPEAKING state when conversation ends."""
    self.speaking_state = "idle"
    self.speaking_start_time = None
    self.consecutive_nonred_start_time = None
    self.last_red_in_speaking_time = None
    self._stop_session()
    self.get_logger().info(f'🔇 Exited SPEAKING state (reason: {reason})')

def _update_speaking_state_on_person_status(self, person_status: str):
    """Update SPEAKING state based on person_status changes."""
    if self.speaking_state != "speaking":
        return  # Not in SPEAKING state, nothing to do
    
    current_time = self.get_clock().now()
    
    if person_status == "red":
        # Person recognized during SPEAKING - reset non-RED timer
        self.last_red_in_speaking_time = current_time
        self.consecutive_nonred_start_time = None
        self.get_logger().debug('SPEAKING: Person RED, protection timer reset')
    else:
        # Non-RED during SPEAKING (BLUE or GREEN)
        if self.consecutive_nonred_start_time is None:
            # Start tracking consecutive non-RED period
            self.consecutive_nonred_start_time = current_time
            self.get_logger().info(f'SPEAKING: Person non-RED ({person_status}), starting 35s protection timer')
        else:
            # Already tracking - check if protection expired
            time_nonred = (current_time - self.consecutive_nonred_start_time).nanoseconds / 1e9
            
            if time_nonred > 35.0:
                # Been non-RED for 35 consecutive seconds during SPEAKING
                self.get_logger().warn(
                    f'SPEAKING: Person absent for {time_nonred:.0f}s (35s threshold). Auto-stopping conversation.'
                )
                self._exit_speaking_state(reason="absence_timeout")
```

**Modify person_status_callback (after line 144):**
```python
# After updating self.person_status:
if old_status != self.person_status:
    self.get_logger().info(f'👤 Person status: {old_status} → {self.person_status}')
    
    # Update SPEAKING state if active (NEW)
    self._update_speaking_state_on_person_status(self.person_status)
```

### Fix C: Modify Watchdog to Use SPEAKING State

**File:** `ros2_ws/src/r2d2_gesture/r2d2_gesture/gesture_intent_node.py`

**Modify watchdog_callback (lines 295-342):**
```python
def watchdog_callback(self):
    """
    Watchdog timer callback with SPEAKING state protection.
    
    Rules:
    - IDLE state: Use old behavior (35s non-RED → no action, just tracking)
    - SPEAKING state: Protected by _update_speaking_state_on_person_status()
      (handled in person_status_callback, not here)
    """
    if not self.auto_shutdown_enabled:
        return
    
    # NEW: If in SPEAKING state, protection is handled by _update_speaking_state
    if self.speaking_state == "speaking":
        # Protection logic is in _update_speaking_state_on_person_status
        # This watchdog only monitors idle state
        return
    
    # OLD LOGIC: Only applies when IDLE (not speaking)
    if self.person_status != "red":
        # Person not present in IDLE state
        if self.last_red_status_time is None:
            self.last_red_status_time = self.get_clock().now()
            self.get_logger().info(
                f'⏰ Watchdog (IDLE): Person absent (status={self.person_status}), tracking'
            )
        
        time_since_red = (self.get_clock().now() - self.last_red_status_time).nanoseconds / 1e9
        
        # In IDLE state, we don't auto-stop anything (no session running)
        # Just track for monitoring purposes
        
    else:
        # Back to RED
        if self.last_red_status_time is not None:
            self.get_logger().info('⏰ Watchdog (IDLE): Person returned (RED)')
        self.last_red_status_time = None
        self.auto_shutdown_triggered = False
```

**Key Change:** Watchdog defers to SPEAKING state protection when conversation active

---

## Complete Solution: Combined Fixes

### Summary of All Fixes

| Fix | File | Change | Priority | Impact |
|-----|------|--------|----------|--------|
| **A. Increase Jitter** | audio_notification_node.py | 5s → 15s | 🔴 High | More stable RED |
| **B. Add SPEAKING State** | gesture_intent_node.py | New state machine | 🔴 Critical | Conversation protection |
| **C. Modify Watchdog** | gesture_intent_node.py | Use SPEAKING state | 🔴 Critical | Protect active conversations |
| **D. Session-Aware Audio** | audio_notification_node.py | Subscribe to session_status | 🟡 Medium | Different rules when speaking |

### Implementation Order

1. **First:** Fix B + C (SPEAKING state + watchdog modification)
   - Provides immediate conversation protection
   - Solves interruption issue

2. **Second:** Fix A (increase jitter to 15s)
   - Improves RED stability overall
   - Reduces false loss detections

3. **Third:** Fix D (session-aware audio notification)
   - Further refinement
   - Different jitter tolerance for speaking vs idle

### Testing After Fixes

**Test Scenario 1: Face Lost During Conversation**
```
1. Start conversation with index finger
2. Turn head away for 10 seconds while speaking
3. EXPECT: Conversation continues (protected by SPEAKING state)
4. EXPECT: RED status maintained or quickly restored
5. EXPECT: No interruption in speech-to-speech
```

**Test Scenario 2: User Leaves During Conversation**
```
1. Start conversation with index finger
2. Walk away completely (out of frame)
3. Stay away for 40 seconds
4. EXPECT: After 35 consecutive seconds non-RED, conversation auto-stops
5. EXPECT: "Stop" beep heard
6. EXPECT: Clean stop, no errors
```

**Test Scenario 3: Brief GREEN During Conversation**
```
1. Start conversation
2. Lighting changes, face detected but not recognized (GREEN)
3. GREEN status for 5 seconds
4. Face recognized again (RED)
5. EXPECT: Conversation continues without interruption
6. EXPECT: GREEN treated as "person present" during SPEAKING
```

---

## Proposed Parameter Changes

### audio_notification_node Parameters

```python
# CURRENT:
self.declare_parameter('jitter_tolerance_seconds', 5.0)

# PROPOSED:
self.declare_parameter('jitter_tolerance_seconds', 15.0)
self.declare_parameter('jitter_tolerance_speaking', 20.0)  # NEW: Longer during conversations
```

### gesture_intent_node Parameters

```python
# NEW PARAMETERS:
self.declare_parameter('speaking_protection_seconds', 35.0)  # Consecutive non-RED before stop
self.declare_parameter('treat_green_as_present', True)  # GREEN = person present during SPEAKING
```

---

## State Diagram: Proposed SPEAKING State

```
User Raises Finger (person_status="red")
  ↓
[ENTER SPEAKING STATE]
  - Start conversation timer
  - Call start_session service
  - Play "Start" beep
  - Begin monitoring consecutive non-RED time
  ↓
SPEAKING STATE ACTIVE
  ├─ Monitor person_status every update
  ├─ If RED: Reset consecutive_nonred timer
  ├─ If non-RED: Count consecutive time
  ├─ If consecutive_nonred > 35s: AUTO-STOP
  └─ If fist gesture: IMMEDIATE STOP
  ↓
[EXIT SPEAKING STATE]
  - Call stop_session service  
  - Play "Stop" beep
  - Clear all timers
  - Return to IDLE
```

---

## Conclusion: Why Current Implementation May Interrupt Conversations

### Root Cause

**Current watchdog logic:**
```python
if self.person_status != "red":
    # Immediately starts 35s timer
    # Doesn't matter if conversation is 5s old or 5 minutes old
    # Doesn't distinguish between idle and speaking
```

**Problem:** If face lost during active conversation:
1. person_status → non-RED (BLUE or GREEN)
2. Watchdog starts 35s timer immediately
3. If user doesn't return face to camera within 35s, conversation stops
4. No protection for active conversation continuation

**User's Expectation (Correct):**
1. During SPEAKING, brief face loss should be tolerated
2. Only stop if person ACTUALLY gone (35s consecutive absence)
3. Active conversations should be "protected" from brief recognition failures

**Solution:** Implement SPEAKING state with 35-second consecutive non-RED protection

---

## Next Steps

**Awaiting User Confirmation:**
1. ✅ Increase jitter tolerance to 15 seconds?
2. ✅ Implement SPEAKING state with 35s consecutive non-RED protection?
3. ✅ Modify watchdog to use SPEAKING state?

**Ready to Implement:** All code prepared, waiting for approval to apply fixes.

---

**Document Version:** 1.0  
**Date:** December 18, 2025  
**Status:** Analysis complete, fixes prepared, awaiting user confirmation  
**Impact:** CRITICAL - May explain conversation interruptions

