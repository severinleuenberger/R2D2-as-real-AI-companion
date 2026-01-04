# Safe Diagnostics Dashboard - Comprehensive Test Protocol
**Date:** January 4, 2026  
**Version:** 1.0  
**Feature:** Web-based diagnostics and monitoring page  
**Platform:** NVIDIA Jetson AGX Orin 64GB + ROS 2 Humble  
**Status:** Ready for Testing

---

## Test Overview

**Purpose:** Verify the Safe Diagnostics Dashboard operates correctly, displays accurate data, enforces safety protections, and doesn't interfere with core R2D2 UX functionality.

**Total Test Cases:** 52  
**Estimated Duration:** 45-60 minutes  
**Critical Tests:** 8 (marked with 🔴)  
**Prerequisites:** R2D2 system running, web dashboard accessible

---

## Pre-Test Setup

### System State Verification

```bash
# 1. Verify core services are running
systemctl is-active r2d2-camera-perception    # Should be: active
systemctl is-active r2d2-audio-notification   # Should be: active
systemctl is-active r2d2-gesture-intent       # Should be: active
systemctl is-active r2d2-speech-node          # Should be: active

# 2. Start rosbridge for live monitoring
sudo systemctl start r2d2-rosbridge
systemctl is-active r2d2-rosbridge            # Should be: active

# 3. Start web dashboard if not running
bash ~/dev/r2d2/web_dashboard/scripts/start_web_dashboard.sh &
sleep 5

# 4. Verify web dashboard is accessible
curl -s http://localhost:8080 | grep "R2D2" && echo "✓ Dashboard accessible"

# 5. Verify you're in front of camera
ros2 topic echo /r2d2/audio/person_status --once
```

**Required State Before Testing:**
- [ ] All core services running
- [ ] rosbridge running
- [ ] Web dashboard accessible
- [ ] You can be recognized by camera (for live testing)

---

## Test Suite 1: Page Access & Navigation

### TC-1.1: Main Dashboard Access
**Priority:** High  
**Objective:** Verify diagnostics link appears on main dashboard

**Steps:**
1. Navigate to `http://100.x.x.x:8080`
2. Locate "🔧 Diagnostics" link in header (right side, before "Exit Service Mode")

**Expected Result:**
- ✅ Link visible with wrench emoji
- ✅ Link styled consistently with header
- ✅ Hover shows blue highlight

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-1.2: Diagnostics Page Load
**Priority:** High  
**Objective:** Verify diagnostics page loads without errors

**Steps:**
1. Click "🔧 Diagnostics" link
2. Wait for page to load
3. Open browser console (F12)

**Expected Result:**
- ✅ Page loads at `/diagnostics` URL
- ✅ Page title: "R2D2 Diagnostics"
- ✅ No JavaScript errors in console
- ✅ No network errors (except expected rosbridge if not running)

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-1.3: Back Navigation
**Priority:** Medium  
**Objective:** Verify return to main dashboard works

**Steps:**
1. From diagnostics page, click "← Back to Dashboard"

**Expected Result:**
- ✅ Returns to main dashboard (/)
- ✅ Main dashboard loads correctly
- ✅ All panels display

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-1.4: Page Layout & Compactness
**Priority:** Medium  
**Objective:** Verify compact design principles

**Steps:**
1. View diagnostics page on 1920×1200 resolution
2. Check panel spacing and sizing

**Expected Result:**
- ✅ All content fits on screen without scrolling
- ✅ No excessive white space between panels
- ✅ Panels use fit-content (height adapts to content)
- ✅ Grid layouts properly aligned

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

## Test Suite 2: Status Indicators (Live Monitoring)

### TC-2.1: rosbridge Connection
**Priority:** High  
**Objective:** Verify rosbridge connects successfully

**Steps:**
1. Ensure rosbridge is running: `systemctl is-active r2d2-rosbridge`
2. Load diagnostics page
3. Check browser console for connection messages

**Expected Result:**
- ✅ Console shows: "✓ Connected to rosbridge"
- ✅ No "rosbridge not connected" alert
- ✅ WebSocket connection to `ws://100.x.x.x:9090` established

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-2.2: Status Indicator - Status Badge
**Priority:** High  
**Objective:** Verify RED/GREEN/BLUE status displays correctly

**Steps:**
1. Stand in front of camera (get recognized)
2. Observe "Status" indicator

**Expected Result:**
- ✅ Status shows correct state (RED/GREEN/BLUE)
- ✅ Badge has colored background (red/green/blue)
- ✅ Updates in real-time when status changes

**Test Cases:**
- RED status: Badge shows "RED" with red background
- GREEN status: Badge shows "GREEN" with green background  
- BLUE status: Badge shows "BLUE" with blue background

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-2.3: Status Indicator - Person Name
**Priority:** High  
**Objective:** Verify person identity displays

**Steps:**
1. While recognized (RED status), check "Person" indicator

**Expected Result:**
- ✅ Shows your name (e.g., "severin")
- ✅ Updates when status changes
- ✅ Shows "no_person" when in BLUE status
- ✅ Shows "unknown" when in GREEN status

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-2.4: Status Indicator - Confidence Bar
**Priority:** Medium  
**Objective:** Verify confidence displays with color coding

**Steps:**
1. While recognized, observe confidence indicator
2. Note the percentage and bar color

**Expected Result:**
- ✅ Shows percentage (0-100%)
- ✅ Bar fills proportionally
- ✅ Color coding:
  - Green: >80%
  - Yellow: 50-80%
  - Red: <50%

**Result:** PASS / FAIL  
**Observed Confidence:** _____% Color: _______  
**Notes:** ___________________________________________

---

### TC-2.5: Status Indicator - LED State
**Priority:** High  
**Objective:** Verify LED status matches hardware

**Steps:**
1. While in RED status, check LED indicator AND physical LED
2. Walk away (BLUE status), check both again

**Expected Result:**
- ✅ RED status: Indicator shows "💡 ON", physical LED is ON
- ✅ BLUE status: Indicator shows "○ OFF", physical LED is OFF
- ✅ Updates every 2 seconds
- ✅ Reading GPIO doesn't affect LED output

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-2.6: Status Indicator - Face Count
**Priority:** Medium  
**Objective:** Verify face count updates

**Steps:**
1. Stand alone in view → check Faces indicator
2. Have another person join → check again

**Expected Result:**
- ✅ Shows 1 when you're alone
- ✅ Shows 2 when two people visible
- ✅ Shows 0 when no one visible
- ✅ Updates in real-time

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-2.7: Status Indicator - Gesture Display
**Priority:** Medium  
**Objective:** Verify gestures display briefly

**Steps:**
1. Make index finger gesture (☝️)
2. Watch Gesture indicator
3. Wait 3 seconds

**Expected Result:**
- ✅ Shows ☝️ immediately when gesture detected
- ✅ Clears to "--" after 2 seconds
- ✅ Works for all gestures (fist ✊, open hand 🖐️)

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-2.8: Status Indicator - Phase Calculation
**Priority:** Medium  
**Objective:** Verify phase computed correctly

**Test Scenarios:**

| Your State | Expected Phase | Result |
|------------|----------------|--------|
| No one visible (BLUE) | Phase 1: Waiting | ☐ |
| Unknown person (GREEN) | Phase 3: Unknown | ☐ |
| Recognized, no speech (RED) | Phase 4: Ready | ☐ |
| Recognized + Fast Mode active | Phase 5-7: Active | ☐ |
| Recognized + R2-D2 Mode active | Phase 5-7: Active | ☐ |

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-2.9: Status Indicator - Speech Mode
**Priority:** Medium  
**Objective:** Verify speech mode displays

**Steps:**
1. Note initial speech mode (should be 🔇 OFF)
2. Start conversation with index finger
3. Observe speech mode change
4. Stop conversation
5. Observe return to OFF

**Expected Result:**
- ✅ Initial: "🔇 OFF"
- ✅ After index finger: "🎙️ Fast Mode"
- ✅ After open hand: "🖐️ R2-D2 Mode" (if supported)
- ✅ After stop: Returns to "🔇 OFF"

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-2.10: Status Indicator - VAD State
**Priority:** Medium  
**Objective:** Verify voice activity detection displays

**Steps:**
1. Start conversation (index finger)
2. Speak into microphone → observe VAD
3. Stop speaking → observe VAD

**Expected Result:**
- ✅ While speaking: "Speaking"
- ✅ When silent: "Silent"
- ✅ Updates in real-time

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-2.11: Status Indicator - Silence Timer
**Priority:** Medium  
**Objective:** Verify silence timer counts correctly

**Steps:**
1. During conversation, stop speaking
2. Watch silence timer
3. Speak again

**Expected Result:**
- ✅ Counts up when silent: "5/60s", "10/60s"
- ✅ Resets when you speak
- ✅ Shows "--/60s" when not in conversation

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-2.12: Status Indicator - Fist Stop Progress
**Priority:** High  
**Objective:** Verify two-stage fist stop tracking

**Steps:**
1. While in RED status, make fist gesture
2. Hold for 0.5s → observe
3. Hold for 1.5s → observe
4. Hold for 3.0s → observe
5. Release fist → observe

**Expected Result:**
- ✅ 0-1.5s: Shows countdown "0.5s...", "1.0s..."
- ✅ 1.5-3.0s: Shows "Stage 1 (Warning)"
- ✅ 3.0s+: Shows "Stage 2 (STOP)"
- ✅ Release: Resets to "--"

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-2.13: Status Indicator - Cooldowns & Watchdog
**Priority:** Low  
**Objective:** Verify placeholder indicators display

**Expected Result:**
- ✅ Cooldowns shows: "Start ✓ Stop ✓" (placeholder)
- ✅ Watchdog shows: "--/35s" (placeholder)
- ✅ No errors or crashes

**Note:** Full cooldown/watchdog implementation requires backend data

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

## Test Suite 3: Service Status Grid

### TC-3.1: Service Count & Display
**Priority:** High  
**Objective:** Verify all 12 services displayed

**Steps:**
1. Count services in grid
2. Verify each service name

**Expected Services:**
1. camera-perception (🛡️)
2. audio-notification (🛡️)
3. gesture-intent (🛡️)
4. volume-control (⚡)
5. speech-node (⚡)
6. rest-speech-node (⚡)
7. heartbeat
8. powerbutton
9. wake-api
10. rosbridge
11. camera-stream
12. web-dashboard

**Result:** PASS / FAIL  
**Count:** _____ / 12  
**Notes:** ___________________________________________

---

### TC-3.2: Service Status Accuracy
**Priority:** High  
**Objective:** Verify status badges match actual systemd state

**Steps:**
1. For each service, compare grid status with systemd

**Verification:**
```bash
for svc in camera-perception audio-notification gesture-intent volume-control speech-node rest-speech-node heartbeat powerbutton wake-api rosbridge camera-stream web-dashboard; do
  actual=$(systemctl is-active r2d2-$svc 2>/dev/null || echo "inactive")
  echo "$svc: $actual"
done
```

**Expected Result:**
- ✅ [●] Running badge for active services
- ✅ [○] Stopped badge for inactive services
- ✅ Status matches systemd state exactly

**Result:** PASS / FAIL  
**Mismatches:** ___________________________________________

---

### TC-3.3: 🔴 Read-Only Mode Default (CRITICAL SAFETY)
**Priority:** CRITICAL  
**Objective:** Verify page starts in safe read-only mode

**Steps:**
1. Fresh page load (hard refresh: Shift+F5)
2. Check mode indicator
3. Look for service control buttons

**Expected Result:**
- ✅ Mode indicator: "🔒 READ-ONLY MODE" with green background
- ✅ NO Start/Stop/Restart buttons visible anywhere
- ✅ Toggle button shows: "🔓 Enable Control"

**CRITICAL:** Must start in read-only mode to prevent accidents

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-3.4: Enable Control Mode
**Priority:** High  
**Objective:** Verify control activation works

**Steps:**
1. Click "🔓 Enable Control" button
2. Read confirmation dialog
3. Click "OK"

**Expected Result:**
- ✅ Confirmation dialog appears with warning text
- ✅ After OK: Mode indicator changes to "🔓 CONTROL MODE" (yellow)
- ✅ Service control buttons appear (Start/Stop/Restart)
- ✅ Button text changes to "🔒 Lock Controls"

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-3.5: 🔴 Critical Service Protection - camera-perception (CRITICAL)
**Priority:** CRITICAL  
**Objective:** Verify critical service has double confirmation

**Steps:**
1. Enable control mode
2. Find camera-perception (has 🛡️ red shield icon)
3. Click "Stop" button
4. Read first dialog
5. Click "Cancel"

**Expected Result:**
- ✅ First dialog shows: "⚠️ CRITICAL SERVICE WARNING"
- ✅ Warning explains: "Face recognition, gesture detection, and LED feedback will stop!"
- ✅ Click Cancel → NO second dialog
- ✅ Service still running

**Now test acceptance flow:**
6. Click "Stop" again
7. Click "OK" on first dialog
8. Read second dialog
9. Click "Cancel"

**Expected Result:**
- ✅ Second dialog shows: "Final confirmation"
- ✅ Click Cancel → Service still running
- ✅ Verify: `systemctl is-active r2d2-camera-perception` returns "active"

**CRITICAL:** Service must NOT stop unless BOTH confirmations accepted

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-3.6: 🔴 Critical Service Protection - audio-notification (CRITICAL)
**Priority:** CRITICAL  
**Objective:** Verify second critical service protection

**Steps:**
1. Same as TC-3.5 but for audio-notification (🛡️)

**Expected Result:**
- ✅ Double confirmation required
- ✅ Warning: "LED control and audio alerts will stop!"
- ✅ Service only stops if both confirmations accepted

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-3.7: 🔴 Critical Service Protection - gesture-intent (CRITICAL)
**Priority:** CRITICAL  
**Objective:** Verify third critical service protection

**Steps:**
1. Same as TC-3.5 but for gesture-intent (🛡️)

**Expected Result:**
- ✅ Double confirmation required
- ✅ Warning: "Gesture-to-speech control will stop!"
- ✅ Service only stops if both confirmations accepted

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-3.8: High Protection Service - speech-node
**Priority:** High  
**Objective:** Verify high-priority services have single confirmation

**Steps:**
1. Enable control mode
2. Find speech-node (has ⚡ yellow bolt icon)
3. Click "Stop" button
4. Read confirmation dialog
5. Click "Cancel"

**Expected Result:**
- ✅ Single dialog shows: "⚡ Warning"
- ✅ Warning: "Fast Mode conversations will be disabled!"
- ✅ NO second confirmation
- ✅ Click Cancel → Service still running

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-3.9: Low Protection Service - rosbridge
**Priority:** Medium  
**Objective:** Verify low-priority services have simple confirmation

**Steps:**
1. Enable control mode
2. Find rosbridge (no icon, currently stopped)
3. Click "Start" button

**Expected Result:**
- ✅ Simple confirmation: "start r2d2-rosbridge?"
- ✅ NO warning text
- ✅ Single confirmation only

**Note:** Don't actually start it if it needs to stay stopped

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-3.10: Return to Read-Only Mode
**Priority:** Medium  
**Objective:** Verify locking controls works

**Steps:**
1. While in control mode, click "🔒 Lock Controls"

**Expected Result:**
- ✅ Mode indicator returns to "🔒 READ-ONLY MODE" (green)
- ✅ All control buttons disappear
- ✅ Button text returns to "🔓 Enable Control"

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-3.11: Protection Level Visual Indicators
**Priority:** Low  
**Objective:** Verify protection icons display correctly

**Expected Icons:**
- ✅ camera-perception: 🛡️ (red border-left)
- ✅ audio-notification: 🛡️ (red border-left)
- ✅ gesture-intent: 🛡️ (red border-left)
- ✅ volume-control: ⚡ (yellow border-left)
- ✅ speech-node: ⚡ (yellow border-left)
- ✅ rest-speech-node: ⚡ (yellow border-left)
- ✅ Other services: no icon (gray border-left)

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

## Test Suite 4: Topic Monitoring

### TC-4.1: Start Topic Monitor - person_id
**Priority:** High  
**Objective:** Verify topic subscription works

**Steps:**
1. Click "person_id" button under Perception category
2. Wait 5 seconds

**Expected Result:**
- ✅ Topic output container appears
- ✅ Header shows: "/r2d2/perception/person_id"
- ✅ Messages stream in (every ~150ms at 6.5 Hz)
- ✅ Shows person name: "severin" or "unknown"
- ✅ Auto-scrolls to bottom

**Result:** PASS / FAIL  
**Messages Received:** _____ in 5 seconds  
**Notes:** ___________________________________________

---

### TC-4.2: Topic Monitor - face_confidence
**Priority:** Medium  
**Objective:** Test Float32 topic type

**Steps:**
1. Click "face_confidence" button
2. Observe output

**Expected Result:**
- ✅ Shows floating point numbers
- ✅ Updates at ~6.5 Hz
- ✅ Values in reasonable range (0-200)

**Result:** PASS / FAIL  
**Sample Values:** ___________________________________________

---

### TC-4.3: Topic Monitor - gesture_event
**Priority:** Medium  
**Objective:** Test event-based topic

**Steps:**
1. Click "gesture_event" button
2. Make index finger gesture
3. Make fist gesture

**Expected Result:**
- ✅ Shows "index_finger_up" when gesture detected
- ✅ Shows "fist" when fist detected
- ✅ Only publishes when gesture active

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-4.4: Topic Monitor - person_status (JSON)
**Priority:** High  
**Objective:** Test JSON String topic type

**Steps:**
1. Click "person_status" button
2. Observe output format

**Expected Result:**
- ✅ Shows JSON data
- ✅ Readable format
- ✅ Contains: status, person_identity, confidence, duration
- ✅ Updates at 10 Hz

**Result:** PASS / FAIL  
**Sample JSON:** ___________________________________________

---

### TC-4.5: Topic Monitor - session_status
**Priority:** Medium  
**Objective:** Test speech session monitoring

**Steps:**
1. Click "session_status" button
2. Start conversation (index finger)
3. Stop conversation (fist)

**Expected Result:**
- ✅ Shows status changes
- ✅ "connected" when session active
- ✅ "disconnected" when stopped

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-4.6: Topic Monitor - voice_activity
**Priority:** Medium  
**Objective:** Test VAD monitoring

**Steps:**
1. Click "voice_activity" button
2. Start conversation
3. Speak, then stop speaking

**Expected Result:**
- ✅ Shows "speaking" when you talk
- ✅ Shows "silent" when quiet
- ✅ Real-time updates

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-4.7: Topic Monitor - Transcripts
**Priority:** Medium  
**Objective:** Test transcript monitoring

**Steps:**
1. Click "user_transcript" button
2. Start conversation
3. Say something
4. Observe transcript appears

**Expected Result:**
- ✅ User transcript shows what you said
- ✅ Updates when you speak
- ✅ Assistant transcript can be monitored separately

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-4.8: Stop Topic Monitor
**Priority:** High  
**Objective:** Verify unsubscribe works correctly

**Steps:**
1. While monitoring any topic, click "Stop Monitor" button

**Expected Result:**
- ✅ Topic output container disappears
- ✅ Subscription cancelled (check console logs)
- ✅ Can immediately start monitoring different topic

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-4.9: Switch Between Topics
**Priority:** Medium  
**Objective:** Verify switching topics works

**Steps:**
1. Start monitoring face_count
2. Without stopping, click person_id

**Expected Result:**
- ✅ Old subscription auto-stops
- ✅ New topic starts monitoring
- ✅ Header updates to new topic name
- ✅ Output clears and shows new data

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-4.10: Multiple Subscribers (Safety Check)
**Priority:** High  
**Objective:** Verify multiple subscribers don't interfere

**Steps:**
1. Start monitoring /r2d2/audio/person_status on diagnostics page
2. Keep main dashboard open (also subscribes to same topic)
3. In terminal, run: `python3 ~/dev/r2d2/tools/minimal_monitor.py`

**Expected Result:**
- ✅ All 3 subscribers receive same data
- ✅ No topic rate degradation
- ✅ No errors in any subscriber
- ✅ Main dashboard still updates
- ✅ minimal_monitor.py works correctly

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

## Test Suite 5: Diagnostic Tests

### TC-5.1: Test - PulseAudio
**Priority:** High  
**Objective:** Verify PulseAudio diagnostic works

**Steps:**
1. Click "PulseAudio" button
2. Read output

**Expected Output:**
```
✓ PASS: PulseAudio daemon is running
✓ PASS: Default sink: [sink name]
```

**Result:** PASS / FAIL  
**Actual Output:** ___________________________________________

---

### TC-5.2: Test - Bluetooth
**Priority:** Medium  
**Objective:** Verify Bluetooth diagnostic

**Steps:**
1. Click "Bluetooth" button

**Expected Output:**
```
✓ PASS: Bluetooth service active
✓ PASS: Bluetooth adapter powered on
```

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-5.3: Test - Audio Playback
**Priority:** Medium  
**Objective:** Verify audio playback test with overlap warning

**Steps:**
1. Note the warning: "* Audio Playback may briefly overlap with R2D2 beeps"
2. Click "Audio Playback*" button
3. Listen for beep

**Expected Result:**
- ✅ Warning visible before clicking
- ✅ Hear short beep (0.3s, 800Hz tone)
- ✅ Output shows: "✓ PASS: Test tone played"
- ✅ Note about potential overlap displayed

**Did you hear the beep?** YES / NO

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-5.4: Test - Volume
**Priority:** Medium  
**Objective:** Verify volume control check

**Steps:**
1. Click "Volume" button

**Expected Output:**
```
✓ PASS: volume_control_node is running
✓ PASS: Master volume topic publishing: [value]
```

**Result:** PASS / FAIL  
**Volume Value:** ___________________________________________

---

### TC-5.5: Test - Speech Status
**Priority:** Medium  
**Objective:** Verify speech service check

**Steps:**
1. Click "Speech Status" button

**Expected Output:**
```
✓ PASS: r2d2-speech-node service is active
✓ INFO: Speech node lifecycle state: [state]
```

**Result:** PASS / FAIL  
**Lifecycle State:** ___________________________________________

---

### TC-5.6: Test - Quick Status
**Priority:** High  
**Objective:** Verify comprehensive status overview

**Steps:**
1. Click "Quick Status" button

**Expected Output:**
```
=== R2D2 Quick Status ===

Camera/Perception: ✅ Running
Audio Notification: ✅ Running
Gesture Intent: ✅ Running
Speech Node: ✅ Running
```

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-5.7: Test - Topic Hz
**Priority:** Medium  
**Objective:** Verify topic rate measurements

**Steps:**
1. Click "Topic Hz" button
2. Wait for completion (~15 seconds)

**Expected Output:**
```
=== Topic Publishing Rates ===

/oak/rgb/image_raw: ✓ ~30.0 Hz
/r2d2/perception/face_count: ✓ ~13.0 Hz
/r2d2/perception/person_id: ✓ ~6.5 Hz
/r2d2/audio/person_status: ✓ ~10.0 Hz
/r2d2/heartbeat: ✓ ~1.0 Hz
```

**Result:** PASS / FAIL  
**Measured Rates:** ___________________________________________

---

### TC-5.8: Test - ROS Nodes
**Priority:** Medium  
**Objective:** Verify ROS node listing

**Steps:**
1. Click "ROS Nodes" button

**Expected Output:**
```
=== Active ROS 2 Nodes ===

Total nodes: [number]

  /camera_node
  /image_listener
  /audio_notification_node
  /status_led_node
  /database_logger_node
  /gesture_intent_node
  /speech_node
  ...
```

**Result:** PASS / FAIL  
**Node Count:** ___________________________________________

---

### TC-5.9: Test - Recognition Log
**Priority:** Medium  
**Objective:** Verify log retrieval from journalctl

**Steps:**
1. Stand in front of camera (get recognized)
2. Wait 5 seconds
3. Click "Recognition Log" button

**Expected Output:**
```
=== Recent Recognition Events (last 5 minutes) ===

RED-FIRST: severin recognized (blue -> RED)
[other events]
```

**Result:** PASS / FAIL  
**Events Found:** _____  
**Notes:** ___________________________________________

---

### TC-5.10: Test - Gesture Log
**Priority:** Medium  
**Objective:** Verify gesture event logging

**Steps:**
1. Make index finger gesture (☝️)
2. Wait 2 seconds
3. Click "Gesture Log" button

**Expected Output:**
```
=== Recent Gesture Events (last 5 minutes) ===

Gesture detected: index_finger_up
[other events]
```

**Result:** PASS / FAIL  
**Events Found:** _____  
**Notes:** ___________________________________________

---

### TC-5.11: Test Output Display
**Priority:** Low  
**Objective:** Verify test output formatting

**Steps:**
1. Run any test
2. Check output styling

**Expected Result:**
- ✅ PASS lines are green
- ✅ FAIL lines are red
- ✅ WARN lines are yellow
- ✅ Monospace font (Courier New)
- ✅ Readable line spacing

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

## Test Suite 6: Safety & Parallel Operation

### TC-6.1: 🔴 Core UX Unaffected (CRITICAL)
**Priority:** CRITICAL  
**Objective:** Verify diagnostics doesn't interfere with R2D2 UX

**Steps:**
1. Leave diagnostics page open
2. Perform complete UX workflow:
   - Stand in front of camera
   - Verify LED turns on (RED status)
   - Make index finger gesture
   - Start conversation
   - Speak and get response
   - Make fist gesture to stop
   - Verify LED turns off

**Expected Result:**
- ✅ All UX functionality works normally
- ✅ LED responds correctly (ON when recognized)
- ✅ Audio beeps play at correct times
- ✅ Speech conversations work perfectly
- ✅ Gestures trigger correctly
- ✅ No delays or lag
- ✅ No interference or glitches

**CRITICAL:** Core UX must work perfectly with diagnostics page open

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-6.2: 🔴 Multiple Topic Subscribers Safe (CRITICAL)
**Priority:** CRITICAL  
**Objective:** Verify ROS 2 multiple subscriber support

**Steps:**
1. Main dashboard open (subscribes to person_status)
2. Diagnostics page open, monitoring person_status
3. Terminal running: `python3 ~/dev/r2d2/tools/minimal_monitor.py`
4. All watching same topic simultaneously

**Expected Result:**
- ✅ All 3 subscribers receive identical data
- ✅ Topic rate unchanged (10 Hz)
- ✅ No subscriber errors
- ✅ No performance degradation

**Verification:**
```bash
# While all 3 subscribers active
ros2 topic hz /r2d2/audio/person_status
# Should still show ~10 Hz
```

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-6.3: 🔴 GPIO Read Safety (CRITICAL)
**Priority:** CRITICAL  
**Objective:** Verify GPIO reads don't affect outputs

**Steps:**
1. Diagnostics page reading GPIO 17 every 2s
2. Trigger LED changes (walk in/out of view)
3. Watch for any LED flicker or glitches

**Expected Result:**
- ✅ LED turns on/off correctly
- ✅ No flicker or glitches
- ✅ GPIO read doesn't interfere with write
- ✅ LED timing unchanged

**Verification:**
```bash
# Watch GPIO while page is open
watch -n 0.5 'cat /sys/class/gpio/gpio17/value'
```

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-6.4: 🔴 Service Control Blocked in Read-Only (CRITICAL)
**Priority:** CRITICAL  
**Objective:** Ensure read-only mode prevents accidental service control

**Steps:**
1. Ensure page is in read-only mode
2. Open browser console (F12)
3. Try to call: `controlService('r2d2-camera-perception', 'stop')`

**Expected Result:**
- ✅ Alert appears: "Please enable Control Mode first"
- ✅ NO confirmation dialogs appear
- ✅ Service does NOT stop
- ✅ Error message in console

**Verification:**
```bash
systemctl is-active r2d2-camera-perception
# Should still be: active
```

**CRITICAL:** Service must remain running

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-6.5: rosbridge Connection Loss Handling
**Priority:** Medium  
**Objective:** Verify graceful degradation without rosbridge

**Steps:**
1. While page is open, stop rosbridge:
```bash
sudo systemctl stop r2d2-rosbridge
```
2. Wait 10 seconds
3. Try to start a topic monitor

**Expected Result:**
- ✅ Status indicators show "--" (no data)
- ✅ Alert/notification: "rosbridge not connected"
- ✅ Topic monitoring doesn't start
- ✅ Page doesn't crash
- ✅ Service grid still works
- ✅ Diagnostic tests still work

**Restore:**
```bash
sudo systemctl start r2d2-rosbridge
# Refresh page
```

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

## Test Suite 7: Performance & Resource Impact

### TC-7.1: CPU Impact
**Priority:** High  
**Objective:** Verify minimal CPU overhead

**Steps:**
1. Check baseline CPU:
```bash
top -bn1 | grep "Cpu(s)"
```
2. Open diagnostics page with all indicators active
3. Monitor person_status topic (10 Hz)
4. Check CPU again

**Expected Result:**
- ✅ CPU increase <2% total
- ✅ No individual process spikes >5%
- ✅ System remains responsive

**Baseline CPU:** _____%  
**With Diagnostics:** _____%  
**Increase:** _____%

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-7.2: Memory Impact
**Priority:** Medium  
**Objective:** Verify no memory leaks

**Steps:**
1. Check baseline memory: `free -h`
2. Leave page open 5 minutes with active monitoring
3. Check memory again

**Expected Result:**
- ✅ Memory increase <50 MB
- ✅ No continuous growth
- ✅ Memory stable over time

**Baseline:** _____ MB  
**After 5 min:** _____ MB  
**Increase:** _____ MB

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-7.3: Topic Rate Impact
**Priority:** High  
**Objective:** Verify subscribers don't slow topics

**Steps:**
1. Measure baseline:
```bash
ros2 topic hz /r2d2/audio/person_status
```
2. Start monitoring person_status on diagnostics
3. Measure again

**Expected Result:**
- ✅ Rate unchanged (~10 Hz)
- ✅ No degradation from additional subscriber

**Baseline Rate:** _____ Hz  
**With Subscriber:** _____ Hz

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-7.4: Browser Performance
**Priority:** Low  
**Objective:** Check browser resource usage

**Steps:**
1. Open browser task manager (Shift+Esc in Chrome)
2. Find diagnostics page tab
3. Note CPU and Memory

**Expected Result:**
- ✅ Tab CPU <5%
- ✅ Tab Memory <100 MB
- ✅ No memory leaks over time

**Result:** PASS / FAIL  
**Tab CPU:** _____%  
**Tab Memory:** _____ MB  
**Notes:** ___________________________________________

---

## Test Suite 8: Error Handling & Edge Cases

### TC-8.1: Missing Service Graceful Degradation
**Priority:** Medium  
**Objective:** Page works when non-critical services down

**Steps:**
1. Stop heartbeat: `sudo systemctl stop r2d2-heartbeat`
2. Refresh diagnostics page
3. Check service grid

**Expected Result:**
- ✅ heartbeat shows [○] Stopped
- ✅ No JavaScript errors
- ✅ Other services display correctly
- ✅ Page remains functional

**Restore:** `sudo systemctl start r2d2-heartbeat`

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-8.2: Network Error Handling
**Priority:** Low  
**Objective:** Graceful API failure handling

**Steps:**
1. In browser console, test API:
```javascript
fetch('/api/diagnostics/services').then(r => r.json()).then(d => console.log('API works', d))
```

**Expected Result:**
- ✅ API returns data
- ✅ No 500 errors
- ✅ Proper error handling if API fails

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-8.3: Topic Not Publishing
**Priority:** Low  
**Objective:** Handle non-publishing topics gracefully

**Steps:**
1. Monitor a low-frequency topic
2. Wait 30 seconds

**Expected Result:**
- ✅ No JavaScript errors if no messages
- ✅ Output area remains empty (no crashes)
- ✅ "Stop Monitor" still works

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-8.4: Rapid Button Clicking
**Priority:** Low  
**Objective:** Test UI stability under stress

**Steps:**
1. Rapidly click different topic monitor buttons (10 clicks in 5 seconds)
2. Rapidly click diagnostic test buttons

**Expected Result:**
- ✅ No crashes or errors
- ✅ Only latest topic monitored
- ✅ Test output updates correctly

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

## Test Suite 9: End-to-End Scenarios

### TC-9.1: Full Monitoring Session
**Priority:** High  
**Objective:** Use all features together

**Steps:**
1. Open diagnostics page
2. Observe status indicators for 5 minutes
3. Start monitoring person_status
4. Run 3 different diagnostic tests
5. Enable control mode
6. Check various services

**Expected Result:**
- ✅ All features work simultaneously
- ✅ No conflicts between features
- ✅ Data updates correctly
- ✅ Performance remains good
- ✅ No errors or warnings

**Result:** PASS / FAIL  
**Duration:** _____ minutes  
**Notes:** ___________________________________________

---

### TC-9.2: Troubleshooting Workflow
**Priority:** Medium  
**Objective:** Use page to diagnose real issue

**Scenario:** "Gestures not working"

**Troubleshooting Steps:**
1. Check status indicators → Is status RED?
2. Check service grid → Is gesture-intent running?
3. Run "Recognition Log" → Is person recognized?
4. Run "Gesture Log" → Are gestures detected?
5. Monitor gesture_event → See live gesture events?
6. Monitor person_status → Verify RED status?

**Expected Result:**
- ✅ Can follow logical troubleshooting flow
- ✅ All needed information accessible
- ✅ Issue can be identified from available data

**Result:** PASS / FAIL  
**Issue Identified:** ___________________________________________

---

### TC-9.3: Service Management Workflow
**Priority:** Medium  
**Objective:** Safely restart a service

**Steps:**
1. Enable control mode
2. Select rosbridge (low protection)
3. Stop it (confirm)
4. Wait 2s
5. Start it (confirm)
6. Verify it's running

**Expected Result:**
- ✅ Stop confirmation works
- ✅ Service stops
- ✅ Start confirmation works
- ✅ Service starts
- ✅ Status grid updates

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

## Test Suite 10: Documentation Verification

### TC-10.1: Documentation Updated
**Priority:** High  
**Objective:** Verify documentation is complete

**Files to Check:**
- [ ] `110_WEB_UI_REFERENCE.md` - Has new Diagnostics Page section
- [ ] `000_UX_AND_FUNCTIONS.md` - Section 4 mentions diagnostics
- [ ] Cross-references are correct

**Expected:**
- ✅ Diagnostics page documented
- ✅ Safety features explained
- ✅ Cross-references to service/topic docs

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

### TC-10.2: Security Documentation
**Priority:** High  
**Objective:** Verify no sensitive data in docs

**Check:**
```bash
grep -E "100\.(95|96|97|98|99)\.[0-9]+" ~/dev/r2d2/110_WEB_UI_REFERENCE.md ~/dev/r2d2/000_UX_AND_FUNCTIONS.md
```

**Expected Result:**
- ✅ Returns no matches or only placeholder IPs (100.x.x.x)
- ✅ No real Tailscale IPs exposed

**Result:** PASS / FAIL  
**Notes:** ___________________________________________

---

## Test Suite 11: Git & Deployment Verification

### TC-11.1: Git Commit Verification
**Priority:** High  
**Objective:** Verify commit was created correctly

**Steps:**
```bash
cd ~/dev/r2d2
git log -1 --stat
```

**Expected Result:**
- ✅ Commit message is descriptive
- ✅ All 9 diagnostics files included
- ✅ No unrelated files included
- ✅ Commit on feature/face-tracking-tilt-servo branch

**Result:** PASS / FAIL  
**Commit Hash:** ___________________________________________

---

### TC-11.2: Git Push Verification
**Priority:** High  
**Objective:** Verify push succeeded

**Steps:**
1. Check push status:
```bash
cd ~/dev/r2d2
git status
```
2. Verify on GitHub web interface

**Expected Result:**
- ✅ Working tree clean
- ✅ Branch up to date with origin
- ✅ Commit visible on GitHub

**Result:** PASS / FAIL  
**GitHub URL:** ___________________________________________

---

## Test Suite 12: Long-Running Stability

### TC-12.1: Extended Operation (24 hours)
**Priority:** Medium  
**Objective:** Verify no issues over extended use

**Steps:**
1. Leave diagnostics page open for 24 hours
2. Check periodically for issues

**Expected Result:**
- ✅ Page still responsive after 24h
- ✅ No memory leaks
- ✅ Status indicators still updating
- ✅ No JavaScript errors accumulated

**Result:** PASS / FAIL  
**Duration Tested:** _____ hours  
**Notes:** ___________________________________________

---

## Critical Test Summary

**Critical Tests (Must All Pass):**

| ID | Test | Result |
|----|------|--------|
| TC-3.3 | 🔴 Read-Only Mode Default | ☐ PASS / ☐ FAIL |
| TC-3.5 | 🔴 Critical Service Protection - camera | ☐ PASS / ☐ FAIL |
| TC-3.6 | 🔴 Critical Service Protection - audio | ☐ PASS / ☐ FAIL |
| TC-3.7 | 🔴 Critical Service Protection - gesture | ☐ PASS / ☐ FAIL |
| TC-6.1 | 🔴 Core UX Unaffected | ☐ PASS / ☐ FAIL |
| TC-6.2 | 🔴 Multiple Topic Subscribers Safe | ☐ PASS / ☐ FAIL |
| TC-6.3 | 🔴 GPIO Read Safety | ☐ PASS / ☐ FAIL |
| TC-6.4 | 🔴 Service Control Blocked in Read-Only | ☐ PASS / ☐ FAIL |

**CRITICAL:** All 8 critical tests MUST pass for production approval

---

## Overall Test Results

### Test Statistics

**Total Tests:** 52  
**Tests Passed:** _____ / 52  
**Tests Failed:** _____  
**Tests Skipped:** _____  
**Pass Rate:** _____%

### Critical Tests
**Critical Passed:** _____ / 8  
**Critical Failed:** _____

### Test Categories

| Category | Passed | Total | Pass Rate |
|----------|--------|-------|-----------|
| Page Access & Navigation | ___ | 4 | ___% |
| Status Indicators | ___ | 13 | ___% |
| Service Status Grid | ___ | 11 | ___% |
| Topic Monitoring | ___ | 10 | ___% |
| Diagnostic Tests | ___ | 11 | ___% |
| Safety & Parallel Operation | ___ | 5 | ___% |
| Performance | ___ | 4 | ___% |
| Error Handling | ___ | 4 | ___% |
| End-to-End Scenarios | ___ | 3 | ___% |
| Documentation | ___ | 2 | ___% |
| Git & Deployment | ___ | 2 | ___% |

---

## Issues Found

### High Priority Issues
1. ___________________________________________
2. ___________________________________________
3. ___________________________________________

### Medium Priority Issues
1. ___________________________________________
2. ___________________________________________

### Low Priority Issues
1. ___________________________________________
2. ___________________________________________

---

## Production Readiness Decision

### Minimum Requirements for Production

**Must Have:**
- ✅ All 8 critical tests PASS (100% required)
- ✅ Core UX unaffected (TC-6.1 must PASS)
- ✅ Read-only mode works (TC-3.3 must PASS)
- ✅ Overall pass rate ≥90%

**Should Have:**
- ✅ Performance tests acceptable
- ✅ All topic monitors functional
- ✅ Documentation complete

**Nice to Have:**
- ✅ All tests pass (100%)
- ✅ No issues found
- ✅ Extended stability verified

### Final Decision

**Status:** ☐ APPROVED FOR PRODUCTION / ☐ NEEDS FIXES / ☐ BLOCKED

**Rationale:**
_________________________________________________________________
_________________________________________________________________
_________________________________________________________________

**Approved By:** _______________  
**Date:** _______________  
**Deployment Date:** _______________

---

## Rollback Procedure (If Issues Found)

If critical issues prevent production deployment:

```bash
# 1. Revert the commit
cd ~/dev/r2d2
git revert d6fb856d  # Use actual commit hash
git push origin feature/face-tracking-tilt-servo

# 2. Remove diagnostics link from main dashboard (if needed)
# Edit web_dashboard/app/templates/index.html
# Remove the diagnostics link line

# 3. Restart web dashboard
~/dev/r2d2/scripts/stop_web_dashboard.sh
sleep 2
bash ~/dev/r2d2/web_dashboard/scripts/start_web_dashboard.sh &

# 4. Verify core services still running
systemctl is-active r2d2-camera-perception
systemctl is-active r2d2-audio-notification
systemctl is-active r2d2-gesture-intent

# 5. Test core UX functionality
# Stand in front of camera, verify LED, test gestures, test speech
```

---

## Test Environment Details

**Test Date:** _______________  
**Tested By:** _______________  
**Platform:** NVIDIA Jetson AGX Orin 64GB  
**ROS Version:** Humble  
**Browser:** _______________  
**Browser Version:** _______________  
**Network:** Tailscale VPN / Local  
**Access URL:** http://100.x.x.x:8080/diagnostics

**System State:**
- Camera-perception: ☐ Running / ☐ Stopped
- Audio-notification: ☐ Running / ☐ Stopped
- Gesture-intent: ☐ Running / ☐ Stopped
- Speech-node: ☐ Running / ☐ Stopped
- rosbridge: ☐ Running / ☐ Stopped
- Web-dashboard: ☐ Running / ☐ Stopped

---

## Notes & Observations

### General Observations
_________________________________________________________________
_________________________________________________________________
_________________________________________________________________

### Recommendations
_________________________________________________________________
_________________________________________________________________
_________________________________________________________________

### Future Improvements
_________________________________________________________________
_________________________________________________________________
_________________________________________________________________

---

## Sign-Off

**Test Protocol Version:** 1.0  
**Feature Version:** Initial Release  
**Git Commit:** d6fb856d  
**Branch:** feature/face-tracking-tilt-servo

**Testing Complete:** ☐ YES / ☐ NO  
**Production Ready:** ☐ YES / ☐ NO / ☐ CONDITIONAL

**Tester Signature:** _______________  
**Date:** _______________

---

**Document Status:** Ready for Use  
**Next Action:** Execute this test protocol and complete all test cases  
**Reference:** This protocol follows `000_AGENT_FINALIZATION_GUIDE.md` verification requirements

