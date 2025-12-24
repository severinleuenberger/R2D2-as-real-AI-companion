# SPEAKING State Design Document

**Date:** December 18, 2025  
**Purpose:** Design comprehensive SPEAKING state for conversation protection  
**Status:** Design complete, code prepared, NOT implemented yet  
**Priority:** CRITICAL for conversation stability

---

## Executive Summary

The current system lacks a dedicated SPEAKING state, which causes conversations to be vulnerable to interruption. This document designs a robust SPEAKING state that:

1. **Protects active conversations** from brief face recognition failures
2. **Implements 35-second consecutive non-RED rule** before auto-stopping
3. **Distinguishes speaking from idle** for different behavior rules
4. **Allows immediate stop** via fist gesture regardless of protection
5. **Handles GREEN status intelligently** during conversations

---

## State Machine Design

### High-Level States

```
IDLE → SPEAKING → IDLE
```

**IDLE State:**
- No conversation active
- Monitoring person_status for gesture enabling (RED required)
- Watchdog not protecting anything
- Low-sensitivity to status changes

**SPEAKING State:**
- Conversation active
- Protected from brief status changes
- Consecutive non-RED timer for auto-stop
- High-tolerance for recognition hiccups

### Detailed State Diagram

```
                    IDLE
                     |
                     | Event: index_finger_up gesture
                     | Condition: person_status = "red"
                     | Action: Call start_session service
                     |
                     ↓
    ┌────────────────────────────────────┐
    │         SPEAKING                   │
    │                                    │
    │  Entry Actions:                   │
    │  - Set speaking_start_time        │
    │  - Reset consecutive_nonred timer │
    │  - Call start_session service     │
    │  - Play "Start" beep              │
    │                                    │
    │  During SPEAKING:                 │
    │  - Monitor person_status          │
    │  - Track consecutive non-RED time │
    │  - Protect from brief RED loss    │
    │                                    │
    │  Exit Triggers:                   │
    │  1. Fist gesture (immediate)      │
    │  2. Non-RED for 35s consecutive   │
    │  3. Explicit stop_session call    │
    │                                    │
    └────────────────────────────────────┘
                     |
                     | Event: Exit triggered
                     | Action: Call stop_session service
                     |
                     ↓
                    IDLE
```

### State Variables

**New variables in gesture_intent_node:**

```python
# SPEAKING state tracking
self.speaking_state = "idle"  # "idle" or "speaking"
self.speaking_start_time = None  # ROS Time when SPEAKING entered
self.consecutive_nonred_start_time = None  # When current non-RED period began
self.last_red_in_speaking_time = None  # Last time saw RED during SPEAKING
self.speaking_protection_timeout = 35.0  # Consecutive non-RED threshold
```

---

## State Transition Rules

### Rule 1: Entering SPEAKING State

**Trigger:** index_finger_up gesture detected

**Preconditions:**
- person_status = "red"
- session_active = false (from current logic)
- Cooldown elapsed

**Actions:**
1. Set `speaking_state = "speaking"`
2. Record `speaking_start_time = now()`
3. Record `last_red_in_speaking_time = now()`
4. Clear `consecutive_nonred_start_time = None`
5. Call `start_session` service
6. When session_status updates → Play "Start" beep

**Effect:** System enters protected conversation mode

### Rule 2: Maintaining SPEAKING State

**During SPEAKING, on every person_status update:**

```python
IF person_status == "red":
    # Person recognized - reset protection
    last_red_in_speaking_time = now()
    consecutive_nonred_start_time = None
    LOG: "SPEAKING protected: Person RED, timer reset"

ELSE:  # person_status is "blue" or "green"
    # Person not RED - track consecutive time
    IF consecutive_nonred_start_time is None:
        # Start tracking consecutive non-RED period
        consecutive_nonred_start_time = now()
        LOG: "SPEAKING: Person non-RED, starting protection timer (35s)"
    
    ELSE:
        # Already tracking - check if protection expired
        time_nonred = now() - consecutive_nonred_start_time
        
        IF time_nonred > speaking_protection_timeout:  # 35 seconds
            # Person absent for 35 CONSECUTIVE seconds
            LOG: "SPEAKING: 35s consecutive absence, auto-stopping"
            EXIT SPEAKING STATE (reason: "absence_timeout")
```

**Key Point:** Timer resets EVERY time RED is seen. Must be 35 CONSECUTIVE seconds without RED.

### Rule 3: Exiting SPEAKING State

**Trigger 1: Fist Gesture (Immediate)**
- Preconditions: speaking_state = "speaking"
- Actions:
  1. Call `stop_session` service
  2. When session_status updates → Play "Stop" beep
  3. Set `speaking_state = "idle"`
  4. Clear all SPEAKING timers
- Priority: HIGHEST (user explicit command)

**Trigger 2: Timeout (35s Consecutive Non-RED)**
- Preconditions: consecutive_nonred time > 35s
- Actions:
  1. Call `stop_session` service
  2. Set `speaking_state = "idle"`
  3. Clear all SPEAKING timers
  4. LOG: Auto-stopped due to absence
- Priority: MEDIUM (automatic safety)

**Trigger 3: Explicit Service Call**
- Preconditions: External stop_session call
- Actions: Same as Trigger 1
- Priority: HIGH (system command)

**Trigger 4: Session Failure**
- Preconditions: session_status = "disconnected" unexpectedly
- Actions:
  1. Set `speaking_state = "idle"`
  2. Clear all SPEAKING timers
  3. LOG: Session failed
- Priority: HIGH (error handling)

---

## Behavior Differences: IDLE vs SPEAKING

| Behavior | IDLE State | SPEAKING State | Rationale |
|----------|------------|----------------|-----------|
| **Gesture Enablement** | person_status = "red" | person_status = "red" | Same (still need recognition) |
| **Start Gesture** | Enabled if RED | Blocked (already speaking) | Prevent duplicate starts |
| **Stop Gesture** | Blocked (nothing to stop) | Enabled always | Allow immediate stop |
| **Watchdog Trigger** | Tracks non-RED time (idle) | Tracks CONSECUTIVE non-RED | Protect conversations |
| **RED Flickering** | Not critical | CRITICAL - resets protection | Conversation must continue |
| **GREEN Status** | Treated as "not RED" | Treated as "person present" | Someone there, just not recognized |
| **Auto-Stop Threshold** | N/A (nothing running) | 35s consecutive non-RED | Configured protection |

---

## GREEN Status Handling During SPEAKING

### Current Problem

**Current logic treats GREEN same as BLUE:**
```python
if self.person_status != "red":
    # Triggers for both BLUE (nobody) and GREEN (somebody unknown)
```

**Issue During Conversations:**
- Lighting changes → Face detected but not recognized → GREEN
- GREEN is someone present, just not identified
- Should GREEN trigger watchdog same as BLUE (nobody)?

### Proposed Solution

**Differentiate BLUE vs GREEN during SPEAKING:**

```python
# During SPEAKING state:
if self.person_status == "blue":
    # Nobody detected - definitely start absence timer
    consecutive_nonred_start_time = now()

elif self.person_status == "green":
    # Someone present but unknown
    # Options:
    # A. Treat as RED (assume it's still the user, just not recognized)
    # B. Treat as BLUE (someone else, stop conversation)
    # C. Give extra grace period (10s) before starting timer
    
    # RECOMMENDED: Option C
    if consecutive_nonred_start_time is None:
        consecutive_nonred_start_time = now() - 10.0  # Back-date by 10s
        # This gives 10s grace for re-recognition
        # But still counts toward 35s if unknown persists
```

**Rationale:** GREEN = someone is there, might still be the user. Give extra time before stopping conversation.

---

## Timing Rules

### Protection Timers

| Timer | Duration | Purpose | Reset Condition |
|-------|----------|---------|-----------------|
| **Jitter Tolerance** | 15s (proposed) | Maintain RED despite brief face loss | person_id = target_person |
| **Loss Confirmation** | 15s | Confirm actual departure before BLUE | person_id = target_person |
| **Speaking Protection** | 35s consecutive | Protect conversation from brief absence | person_status = "red" |
| **Cooldown (Start)** | 5s | Prevent rapid gesture re-triggering | On gesture trigger |
| **Cooldown (Stop)** | 3s | Prevent rapid gesture re-triggering | On gesture trigger |

### Cumulative Protection

**Idle State (No Conversation):**
- Brief face loss (< 15s): RED maintained by jitter tolerance
- Medium absence (15-30s): RED → BLUE transition begins
- Long absence (> 30s): BLUE state, "Lost you!" beep

**Speaking State (Conversation Active):**
- Brief face loss (< 35s): SPEAKING continues, ignores status changes
- RED returns: Protection timer resets, conversation continues indefinitely
- Long absence (> 35s consecutive): SPEAKING → IDLE, conversation stops

**Maximum Protection:** 15s (jitter) + 35s (speaking protection) = 50 seconds total before conversation stops if user walks away during conversation

---

## Code Integration Points

### Integration Point 1: gesture_intent_node Initialization

**Location:** `__init__` method after line 81

**Add:**
```python
# SPEAKING state tracking
self.speaking_state = "idle"  # "idle" or "speaking"
self.speaking_start_time = None
self.consecutive_nonred_start_time = None
self.last_red_in_speaking_time = None
self.speaking_protection_timeout = 35.0  # Parameter
```

### Integration Point 2: person_status_callback

**Location:** After line 144 (status change log)

**Add:**
```python
# Update SPEAKING state protection if conversation active
if self.speaking_state == "speaking":
    self._update_speaking_protection(self.person_status)
```

### Integration Point 3: gesture_callback (index_finger_up)

**Location:** Replace line 236 `self._start_session()`

**Replace with:**
```python
self._enter_speaking_state()
```

### Integration Point 4: gesture_callback (fist)

**Location:** Replace line 253 `self._stop_session()`

**Replace with:**
```python
self._exit_speaking_state(reason="user_fist_gesture")
```

### Integration Point 5: watchdog_callback

**Location:** Replace entire method (lines 295-342)

**Replace with:**
```python
def watchdog_callback(self):
    # SPEAKING state protection handled in person_status_callback
    # This watchdog only monitors IDLE state
    if self.speaking_state == "speaking":
        return  # Protection active, defer to _update_speaking_protection
    
    # IDLE state monitoring (unchanged from current)
    # ... existing code ...
```

---

## Implementation Checklist

**Phase A: Add State Variables** ✅ Prepared
- [ ] Add speaking_state, timers to __init__
- [ ] Add parameter for speaking_protection_timeout
- [ ] Log initialization

**Phase B: Implement State Transitions** ✅ Prepared
- [ ] Create _enter_speaking_state() method
- [ ] Create _exit_speaking_state(reason) method
- [ ] Create _update_speaking_protection(person_status) method

**Phase C: Integrate with Callbacks** ✅ Prepared
- [ ] Modify person_status_callback to update SPEAKING protection
- [ ] Modify gesture_callback (index_finger_up) to use _enter_speaking_state
- [ ] Modify gesture_callback (fist) to use _exit_speaking_state

**Phase D: Update Watchdog** ✅ Prepared
- [ ] Skip watchdog when in SPEAKING state
- [ ] Protection handled by _update_speaking_protection instead

**Phase E: Testing** ⏳ After implementation
- [ ] Test brief face loss during conversation
- [ ] Test 35s consecutive absence
- [ ] Test GREEN status during conversation
- [ ] Test fist gesture immediate stop

---

## Expected Behavior After Implementation

### Scenario 1: Brief Face Loss During Conversation

```
t=0s:   User raises finger, enters SPEAKING
        speaking_state = "speaking"
        last_red_in_speaking_time = t=0s
        
t=5s:   User turns head, face lost for 3 seconds
        person_status = "blue"
        consecutive_nonred_start_time = t=5s
        
t=8s:   Face detected again
        person_status = "red"  
        last_red_in_speaking_time = t=8s
        consecutive_nonred_start_time = None  (RESET)
        
Result: ✅ Conversation continues uninterrupted
```

### Scenario 2: User Leaves During Conversation

```
t=0s:   User raises finger, enters SPEAKING
        
t=10s:  User walks away
        person_status = "blue"
        consecutive_nonred_start_time = t=10s
        
t=20s:  Still away (10s non-RED so far)
        
t=30s:  Still away (20s non-RED so far)
        
t=40s:  Still away (30s non-RED so far)
        
t=45s:  Still away (35s non-RED REACHED)
        → Auto-stop triggered
        → Exit SPEAKING state
        → Call stop_session
        
Result: ✅ Conversation protected for 35s, then auto-stopped
```

### Scenario 3: GREEN Status During Conversation

```
t=0s:   User raises finger, enters SPEAKING, person_status="red"
        
t=10s:  Lighting changes, face detected but not recognized
        person_status = "green"
        consecutive_nonred_start_time = t=10s
        
t=15s:  Still GREEN (5s non-RED so far)
        
t=20s:  Face recognized again
        person_status = "red"
        consecutive_nonred_start_time = None  (RESET)
        last_red_in_speaking_time = t=20s
        
Result: ✅ Conversation continues (GREEN is someone present)
```

### Scenario 4: Fist Gesture Immediate Stop

```
t=0s:   User raises finger, enters SPEAKING
        
t=5s:   User makes fist gesture
        → Gesture detected
        → _exit_speaking_state(reason="user_fist_gesture")
        → Call stop_session IMMEDIATELY
        → Play "Stop" beep
        
Result: ✅ Immediate stop regardless of protection timers
```

---

## Integration with Existing Components

### Modified Component: gesture_intent_node

**Responsibilities:**
- Manage SPEAKING state transitions
- Track consecutive non-RED time during SPEAKING
- Enforce 35s protection rule
- Handle immediate fist gesture stop
- Coordinate with watchdog

**New Methods:**
- `_enter_speaking_state()` - Entry actions
- `_exit_speaking_state(reason)` - Exit actions
- `_update_speaking_protection(person_status)` - Protection logic

### Unchanged Components

**audio_notification_node:**
- Still manages RED/BLUE/GREEN state machine
- Still publishes person_status at 10 Hz
- Optionally: Could subscribe to session_status for conversation-aware jitter

**speech_node:**
- Still provides start/stop services
- Still publishes session_status
- No changes needed

**image_listener:**
- Still publishes person_id
- Still publishes gesture_event
- No changes needed

---

## Parameters and Configuration

### New Parameters

```python
# In gesture_intent_node:
self.declare_parameter('speaking_protection_seconds', 35.0)
    # Consecutive non-RED time before auto-stop during SPEAKING
    
self.declare_parameter('treat_green_as_present_during_speaking', True)
    # If True, GREEN status during SPEAKING doesn't immediately start timer
    # Gives extra grace for re-recognition

self.declare_parameter('speaking_min_duration_seconds', 5.0)
    # Minimum time in SPEAKING before auto-stop can trigger
    # Prevents immediate stop if enter SPEAKING then immediately lose RED
```

### Existing Parameters (Keep)

```python
self.declare_parameter('cooldown_start_seconds', 5.0)
self.declare_parameter('cooldown_stop_seconds', 3.0)
self.declare_parameter('auto_shutdown_enabled', True)
self.declare_parameter('audio_feedback_enabled', True)
```

### Deprecated Parameters

```python
# No longer used in SPEAKING mode:
self.declare_parameter('auto_shutdown_timeout_seconds', 35.0)
    # Replaced by speaking_protection_seconds
    # Keep for backward compatibility but use speaking_protection instead
```

---

## Logging and Monitoring

### Log Messages

**Entering SPEAKING:**
```
[INFO] 🗣️  Entered SPEAKING state (person_status=red, session starting)
```

**During SPEAKING - RED seen:**
```
[DEBUG] SPEAKING: Person RED (consecutive_nonred timer reset)
```

**During SPEAKING - Non-RED starts:**
```
[INFO] SPEAKING: Person non-RED (status=blue), starting 35s protection timer
```

**During SPEAKING - Non-RED continues:**
```
[DEBUG] SPEAKING: Non-RED for 10s / 35s (still protected)
[DEBUG] SPEAKING: Non-RED for 20s / 35s (still protected)
[WARN]  SPEAKING: Non-RED for 30s / 35s (approaching timeout)
```

**Exiting SPEAKING - Timeout:**
```
[WARN] 🔇 Exited SPEAKING state: 35s consecutive absence (timeout)
```

**Exiting SPEAKING - Fist:**
```
[INFO] 🔇 Exited SPEAKING state: User fist gesture (immediate stop)
```

### Monitoring Commands

```bash
# Monitor person_status for RED stability
ros2 topic echo /r2d2/audio/person_status | grep status

# Monitor gesture_intent logs for SPEAKING state
sudo journalctl -u r2d2-gesture-intent -f | grep "SPEAKING\|Entered\|Exited"

# Monitor session_status
ros2 topic echo /r2d2/speech/session_status
```

---

## Comparison: Current vs Proposed

| Aspect | Current Implementation | Proposed SPEAKING State | Improvement |
|--------|----------------------|------------------------|-------------|
| **Conversation Protection** | None - uses person_status directly | 35s consecutive non-RED protection | ✅ Protects from hiccups |
| **State Tracking** | Boolean session_active | Enum speaking_state + timers | ✅ Explicit state |
| **Stop Conditions** | Fist OR any 35s non-RED | Fist (immediate) OR 35s CONSECUTIVE non-RED | ✅ Smarter logic |
| **RED Flickering** | May trigger watchdog | Resets protection timer | ✅ Stable conversations |
| **GREEN Handling** | Same as BLUE | Optional grace period | ✅ Better recognition tolerance |
| **Timer Reset** | Simple timer | Resets on every RED | ✅ Truly consecutive |
| **Minimum Duration** | None | Optional parameter | ✅ Prevents instant stops |

---

## Edge Cases Handled

### Edge Case 1: Rapid RED/Non-RED Flickering

**Scenario:** Face recognition unstable, alternates RED/GREEN every second

**Current Behavior:**
- Watchdog timer starts/resets every second
- Never accumulates to 35s
- Conversation continues indefinitely

**Proposed Behavior:**
- Same - protection timer resets on every RED
- Conversation protected as long as any RED seen within 35s window
- ✅ Correct behavior

### Edge Case 2: Enter SPEAKING Then Immediate Loss

**Scenario:** User raises finger at edge of camera frame, then walks away

**Current Behavior:**
- Gesture triggers start_session
- Immediately loses RED
- Watchdog starts 35s timer
- Stops after 35s

**Proposed Behavior:**
- Enter SPEAKING state
- Immediate non-RED starts protection timer
- If `speaking_min_duration` parameter set: Must be in SPEAKING for minimum time
- Otherwise: Stops after 35s (same as current)
- ✅ Behavior OK, optional min_duration adds safety

### Edge Case 3: Multiple People During Conversation

**Scenario:** User speaking, another person walks in front of camera

**Current Behavior:**
- person_id = "unknown" (not target person)
- person_status = "green"
- Watchdog triggers (non-RED)

**Proposed Behavior:**
- person_status = "green"
- If `treat_green_as_present`: Give grace period
- If target person returns to view within grace: Continue
- If unknown person stays >35s: Stop conversation
- ✅ Handles multi-person scenarios better

---

## Visual State Diagram: IDLE ↔ SPEAKING Transitions

```
┌─────────────────────────────────────────────────────────┐
│                        IDLE                             │
│                                                         │
│  • No conversation                                      │
│  • Monitoring: person_status for gesture enabling      │
│  • Watchdog: Inactive (nothing to protect)             │
│  • Gestures: Start enabled (if RED), Stop disabled     │
│                                                         │
│  Waiting for: index_finger_up gesture                  │
└─────────────────────────────────────────────────────────┘
                          │
                          │ Trigger: index_finger_up
                          │ Gate: person_status = "red"
                          │       session_active = false
                          │       cooldown elapsed
                          │ Action: start_session service
                          ↓
┌─────────────────────────────────────────────────────────┐
│                     SPEAKING                            │
│                  (Protected State)                      │
│                                                         │
│  Entry:                                                 │
│  • speaking_start_time = now()                         │
│  • last_red_in_speaking_time = now()                   │
│  • consecutive_nonred_start_time = None                │
│  • Call start_session service                          │
│  • Play "Start" beep (on session_status update)        │
│                                                         │
│  During:                                                │
│  • Monitor person_status continuously                   │
│  • If RED: Reset consecutive_nonred timer              │
│  • If non-RED: Count consecutive time                  │
│  • Protection: < 35s consecutive non-RED               │
│                                                         │
│  Monitoring:                                            │
│  ┌─────────────────────────────────────┐              │
│  │ person_status = "red"               │              │
│  │ → Reset consecutive_nonred timer    │              │
│  │ → Continue SPEAKING                 │              │
│  └─────────────────────────────────────┘              │
│                                                         │
│  ┌─────────────────────────────────────┐              │
│  │ person_status != "red" (BLUE/GREEN) │              │
│  │ → Start/continue consecutive timer  │              │
│  │ → If < 35s: Continue SPEAKING       │              │
│  │ → If >= 35s: Exit SPEAKING          │              │
│  └─────────────────────────────────────┘              │
│                                                         │
│  Exit Triggers:                                         │
│  1. Fist gesture (immediate, any time)                 │
│  2. Non-RED for 35s consecutive                        │
│  3. Session failure (error)                            │
│                                                         │
└─────────────────────────────────────────────────────────┘
                          │
                          │ Trigger: Exit condition met
                          │ Action: stop_session service
                          │         Play "Stop" beep
                          │         Clear timers
                          ↓
┌─────────────────────────────────────────────────────────┐
│                        IDLE                             │
│                   (Back to waiting)                     │
└─────────────────────────────────────────────────────────┘
```

---

## Testing Strategy

### Test 1: Stable Conversation

**Procedure:**
1. Start conversation with index finger
2. Speak for 2 minutes
3. Occasionally look away (< 5s each time)
4. Monitor person_status for flickering
5. Verify conversation never interrupts

**Expected:**
- SPEAKING state remains active
- Brief non-RED periods reset when RED returns
- No auto-stops
- No interruptions

### Test 2: 35 Second Consecutive Absence

**Procedure:**
1. Start conversation
2. Walk away completely
3. Stay away for exactly 40 seconds
4. Return to camera

**Expected:**
- At t=35s: Conversation auto-stops
- "Stop" beep plays
- speaking_state = "idle"
- When return: Can start new conversation

### Test 3: Intermittent Absence

**Procedure:**
1. Start conversation
2. Walk away for 20 seconds
3. Return for 2 seconds (RED)
4. Walk away for 20 seconds again
5. Return

**Expected:**
- First 20s absence: Protected, timer at 20s
- Return: Timer resets to 0s
- Second 20s absence: Protected, timer at 20s
- Conversation continues (never reached 35s consecutive)

### Test 4: Fist Gesture Override

**Procedure:**
1. Start conversation
2. Wait only 5 seconds
3. Make fist gesture

**Expected:**
- Immediate stop (overrides 35s protection)
- "Stop" beep plays
- speaking_state = "idle"

---

## Backward Compatibility

### Maintaining Current Behavior When SPEAKING Not Used

**If user doesn't use gestures:**
- Can still call start/stop services directly
- SPEAKING state entered automatically on start_session
- SPEAKING state exited automatically on stop_session
- No UX change for non-gesture users

**Migration Path:**
- Phase 1: Add SPEAKING state (default behavior same)
- Phase 2: Enable speaking_protection_seconds
- Phase 3: Users can tune parameters

---

## Success Criteria

**SPEAKING State is working correctly when:**

✅ Conversations can continue during brief face recognition failures  
✅ 35-second consecutive non-RED rule enforced accurately  
✅ Fist gesture stops immediately regardless of timers  
✅ GREEN status doesn't interrupt conversations unnecessarily  
✅ State transitions are logged clearly  
✅ Watchdog defers to SPEAKING protection  
✅ User can have 2+ minute conversations without interruption  
✅ Auto-stop only triggers when user actually gone (35s consecutive)  

---

**Document Version:** 1.0  
**Date:** December 18, 2025  
**Status:** Design complete, ready for implementation  
**Code:** Prepared in red_status_stability_analysis.md Task 5 section  
**Next Step:** User approval, then implementation

