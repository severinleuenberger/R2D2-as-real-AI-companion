# Watchdog Auto-Shutdown Fix

## Problem Identified

The auto-shutdown watchdog in `gesture_intent_node` was not working due to **two bugs**:

### Bug 1: Schema Mismatch
- **gesture_intent_node expected:** `{"active": true/false, ...}`  
- **speech_node published:** `{"status": "connected"|"active"|"inactive"|"disconnected", ...}`

The gesture_intent_node was checking for a boolean `"active"` field that didn't exist, so `self.session_active` was always `False`, preventing the watchdog from triggering.

### Bug 2: No Periodic Status Updates
The speech_node only published status when events occurred (connect, disconnect, activate, deactivate). If `gesture_intent_node` started **after** the speech node was already connected, it would never receive status updates.

---

## Fixes Applied

### Fix 1: Updated gesture_intent_node.py
**File:** `/home/severin/dev/r2d2/ros2_ws/src/r2d2_gesture/r2d2_gesture/gesture_intent_node.py`

**Change:** Modified `session_status_callback()` to parse the `"status"` string field:

```python
# OLD CODE (broken):
self.session_active = status_data.get('active', False)

# NEW CODE (fixed):
status_str = status_data.get('status', '')
self.session_active = (status_str in ['active', 'connected'])
```

Now correctly interprets:
- `"active"` or `"connected"` → `session_active = True`
- `"inactive"` or `"disconnected"` → `session_active = False`

### Fix 2: Added Periodic Status Publishing to speech_node.py
**File:** `/home/severin/dev/r2d2/ros2_ws/src/r2d2_speech/r2d2_speech_ros/speech_node.py`

**Changes:**
1. Added `self.current_status` to track current state
2. Added `self.status_timer` for periodic publishing
3. Added `_publish_and_update_status()` helper method
4. Added `_periodic_status_callback()` that publishes every 5 seconds
5. Start timer in `on_activate()`, stop in `on_deactivate()`

**Result:** The speech node now publishes its status every 5 seconds, ensuring late-joining nodes (like gesture_intent_node) always receive current status within 5 seconds of starting.

---

## Testing Instructions

### Step 1: Restart Speech Node (Terminal 17)

In the terminal running the speech launch script:

1. **Press Ctrl+C** to stop the current speech node
2. **Wait for clean shutdown**
3. **Restart:**
   ```bash
   cd ~/dev/r2d2
   bash launch_ros2_speech.sh
   ```
4. **Wait for initialization** (10-15 seconds)

### Step 2: Verify Periodic Status Publishing

In a new terminal:

```bash
ros2 topic echo /r2d2/speech/session_status
```

**Expected:** You should see messages every 5 seconds:
```
data: '{"status": "active", "timestamp": ..., "session_id": "..."}'
---
data: '{"status": "connected", "timestamp": ..., "session_id": "..."}'
---
```

If you see messages repeating every ~5 seconds, **Fix 2 is working!** ✅

### Step 3: Restart gesture_intent_node (Terminal 15)

In the terminal running gesture_intent_node:

1. **Press Ctrl+C** to stop
2. **Restart with DEBUG logging to see watchdog activity:**
   ```bash
   cd ~/dev/r2d2/ros2_ws
   source install/setup.bash
   ros2 launch r2d2_gesture gesture_intent.launch.py \
       auto_shutdown_enabled:=true \
       auto_shutdown_timeout_seconds:=60.0 \
       auto_restart_on_return:=false \
       --ros-args --log-level gesture_intent_node:=DEBUG
   ```

### Step 4: Run Test 2 Again

Now repeat Test 2 with proper visibility:

1. **Stand in front of camera** → should see RED status in Terminal 13
2. **Watch Terminal 15** - should see:
   ```
   [INFO] Session active changed: False → True (status=connected)
   ```
3. **Step away from camera** → BLUE status in Terminal 13
4. **Watch Terminal 15** - should see:
   ```
   [DEBUG] Watchdog: Person absent (status=blue), starting timer
   ```
5. **Wait 60 seconds** ⏱️
6. **After 60 seconds, Terminal 15 should show:**
   ```
   [WARN] ⏰ No person presence for 60s (timeout: 60s). Auto-stopping speech service to save API costs.
   [INFO] ✅ stop_session: Session stopped
   ```

---

## Verification Checklist

After completing the steps above, verify:

- ✅ **Periodic status publishing:** `ros2 topic echo /r2d2/speech/session_status` shows messages every 5s
- ✅ **gesture_intent_node receives status:** See `[INFO] Session active changed: False → True`
- ✅ **Watchdog timer starts:** See `[DEBUG] Watchdog: Person absent...` when you step away
- ✅ **Auto-shutdown triggers:** See `[WARN] ⏰ No person presence for 60s...` after timeout
- ✅ **Speech service stops:** Verify with `ros2 service call /r2d2/speech/start_session` (should respond quickly if stopped)

---

## Technical Details

### Status String Mapping

| Speech Node Status | gesture_intent_node Interpretation |
|--------------------|-----------------------------------|
| `"active"` | `session_active = True` |
| `"connected"` | `session_active = True` |
| `"inactive"` | `session_active = False` |
| `"disconnected"` | `session_active = False` |

### Watchdog Logic Flow

1. **Every 10 seconds:** `watchdog_callback()` runs
2. **If person_status != "red":**
   - Start tracking absence time (`last_red_status_time`)
   - Calculate `time_since_red`
3. **If time_since_red > timeout AND session_active:**
   - Trigger auto-shutdown
   - Call `_stop_session()` service
   - Set `auto_shutdown_triggered = True`
4. **If person returns (status = "red"):**
   - Reset timer
   - Optionally restart session (if `auto_restart_on_return = true`)

---

## What You Should See Now

With both fixes applied, the complete flow is:

```
1. gesture_intent_node starts
   ↓
2. Within 5 seconds, receives session_status from speech_node
   ↓
3. Logs: [INFO] Session active changed: False → True (status=connected)
   ↓
4. Person steps away (RED → BLUE)
   ↓
5. Logs: [DEBUG] Watchdog: Person absent (status=blue), starting timer
   ↓
6. Wait 60 seconds...
   ↓
7. Logs: [WARN] ⏰ No person presence for 60s...
   ↓
8. Calls /r2d2/speech/stop_session
   ↓
9. Logs: [INFO] ✅ stop_session: Session stopped
   ↓
10. SUCCESS! 🎉
```

---

## Build Status

Both packages have been rebuilt successfully:

```bash
✅ r2d2_gesture rebuilt (exit code 0)
✅ r2d2_speech rebuilt (exit code 0)
✅ No linter errors
```

**Ready for testing!** 🚀

