# R2D2 Power Button - Test Plan

**Date:** December 9, 2025  
**System:** NVIDIA Jetson AGX Orin  
**Buttons:** 2 (Software + Hardware)  
**Status:** Ready for Testing

---

## Pre-Test Verification

### Hardware Checklist
- [ ] Button 1 soldered correctly (Pin 32 ↔ Pin 39)
- [ ] Button 2 soldered correctly (J42 Pin 2 ↔ J42 Pin 1)
- [ ] All connections are secure
- [ ] No loose wires or short circuits
- [ ] Multimeter confirms continuity on button press

### Software Checklist
- [ ] Service is running: `sudo systemctl status r2d2-powerbutton.service`
- [ ] Code deployed: `/usr/local/bin/r2d2_power_button.py` exists
- [ ] Service enabled for auto-start: `sudo systemctl is-enabled r2d2-powerbutton.service`
- [ ] Logs are being written: `/var/log/r2d2_power_button.log` exists

---

## Test Procedures

### Test 1: Single Press = Low-Power Mode
**Objective:** Verify Button 1 single press enters low-power mode  
**Duration:** ~5 minutes  
**Risk Level:** LOW

**Steps:**
1. System running normally
2. Open two terminals:
   - Terminal A: Run `journalctl -u r2d2-powerbutton.service -f`
   - Terminal B: Monitor system state
3. **Press Button 1 ONCE** (single short tap)
4. Observe logs in Terminal A for:
   - "Single press detected"
   - "Double-press window expired"
   - "ACTION: Entering low-power mode"
   - "nvpmodel -m 2" command execution
   - "✓ Low-power mode activated successfully"

**Expected Outcome:**
- System transitions to low-power mode
- CPU/GPU frequency reduced
- System remains responsive

**Pass/Fail:**
- [ ] PASS - All expected logs appear
- [ ] FAIL - Missing log entries
- [ ] FAIL - System doesn't enter low-power

---

### Test 2: Double Press = Shutdown
**Objective:** Verify Button 1 double press shuts down system  
**Duration:** ~10 minutes  
**Risk Level:** MEDIUM (system will shutdown)

**Steps:**
1. System running normally
2. Open terminal: Run `journalctl -u r2d2-powerbutton.service -f`
3. **Press Button 1 TWICE** (two quick taps, 200-300ms apart)
4. Observe logs for:
   - "Single press detected"
   - "Press count: 2"
   - "Double-press confirmed"
   - "ACTION: Initiating graceful shutdown"
   - "shutdown -h now" command
5. Wait for system to shutdown completely (~10 seconds)

**Expected Outcome:**
- System gracefully shuts down
- All services stop cleanly
- System powers off completely

**Pass/Fail:**
- [ ] PASS - Double-press detected and shutdown executed
- [ ] FAIL - Only single press detected (timing too slow)
- [ ] FAIL - System doesn't shutdown

---

### Test 3: Button 2 Wake from Low-Power
**Objective:** Verify Button 2 wakes from low-power mode  
**Duration:** ~5 minutes  
**Risk Level:** LOW

**Steps:**
1. System in low-power mode (from Test 1)
2. Open terminal: Monitor system state (CPU frequency, services)
3. **Press Button 2 ONCE** (single short tap)
4. Observe system response:
   - System becomes responsive again
   - Services resume normal operation
   - CPU/GPU frequency back to normal

**Expected Outcome:**
- System wakes from low-power mode
- All services resume
- System returns to normal performance

**Pass/Fail:**
- [ ] PASS - System wakes immediately
- [ ] FAIL - No response to Button 2
- [ ] FAIL - System doesn't return to full power

---

### Test 4: Button 2 Boot from Shutdown
**Objective:** Verify Button 2 boots system from shutdown  
**Duration:** ~10 minutes  
**Risk Level:** MEDIUM (system will shutdown then boot)

**Steps:**
1. System running normally (or from end of Test 2)
2. If not already shutdown, press Button 1 twice to shutdown
3. Wait for complete shutdown (monitor lights/fans)
4. **Press Button 2 ONCE** (single short tap)
5. Observe boot sequence:
   - LEDs light up
   - Fans spin up
   - System boots normally
   - Login screen or services start

**Expected Outcome:**
- System boots successfully from shutdown
- Full boot sequence completes (~30-60 seconds)
- System returns to normal operation

**Pass/Fail:**
- [ ] PASS - System boots and runs normally
- [ ] FAIL - No response to Button 2 press
- [ ] FAIL - System starts but hangs

---

### Test 5: Rapid Double-Press Timing Edge Case
**Objective:** Verify system handles rapid button presses  
**Duration:** ~5 minutes  
**Risk Level:** LOW

**Steps:**
1. System running normally
2. Open terminal: `journalctl -u r2d2-powerbutton.service -f`
3. **Press Button 1 TWICE very quickly** (50-100ms apart)
4. Should be detected as double-press (too fast to be two singles)
5. Observe logs and system behavior

**Expected Outcome:**
- Rapid presses detected as double-press
- System shuts down
- No ambiguity between single and double

**Pass/Fail:**
- [ ] PASS - Rapid presses result in shutdown
- [ ] FAIL - Detected as single press instead
- [ ] FAIL - Unexpected behavior

---

### Test 6: Simultaneous Button Presses
**Objective:** Verify both buttons can be pressed without interference  
**Duration:** ~5 minutes  
**Risk Level:** LOW

**Steps:**
1. System in low-power mode
2. **Press both Button 1 AND Button 2 simultaneously**
3. Observe system response

**Expected Outcome:**
- System responds predictably
- One action takes precedence or both execute cleanly
- No system hang or crash

**Pass/Fail:**
- [ ] PASS - Handles simultaneous presses gracefully
- [ ] FAIL - System hangs
- [ ] FAIL - Unexpected behavior

---

### Test 7: Service Auto-Restart
**Objective:** Verify service restarts automatically  
**Duration:** ~10 minutes  
**Risk Level:** MEDIUM

**Steps:**
1. System running normally
2. Stop service: `sudo systemctl stop r2d2-powerbutton.service`
3. Verify stopped: `sudo systemctl status r2d2-powerbutton.service`
4. Wait 5 seconds
5. **Press Button 1** - should have no effect (service stopped)
6. Restart service: `sudo systemctl restart r2d2-powerbutton.service`
7. Verify running: `sudo systemctl status r2d2-powerbutton.service`
8. **Press Button 1 again** - should work normally

**Expected Outcome:**
- Service stops cleanly
- Button unresponsive when stopped
- Service restarts and buttons work immediately
- No manual intervention needed

**Pass/Fail:**
- [ ] PASS - Service lifecycle managed correctly
- [ ] FAIL - Doesn't stop cleanly
- [ ] FAIL - Doesn't restart automatically

---

### Test 8: Log File Verification
**Objective:** Verify all actions are logged  
**Duration:** ~5 minutes  
**Risk Level:** LOW

**Steps:**
1. Run: `tail -f /var/log/r2d2_power_button.log`
2. Perform various button actions:
   - Single press
   - Double press
   - Wait between presses
3. Verify all events appear in log file

**Expected Outcome:**
- All button presses logged with timestamps
- Action taken (low-power/shutdown) logged
- No errors in logs
- Timestamps accurate

**Pass/Fail:**
- [ ] PASS - All events properly logged
- [ ] FAIL - Missing log entries
- [ ] FAIL - Incorrect timestamps

---

## Test Results Summary

### Overall Status
- [ ] All Tests PASSED
- [ ] Some Tests FAILED (see below)
- [ ] Major Issues Found (see below)

### Failed Tests
| Test | Issue | Impact |
|------|-------|--------|
|      |       |        |
|      |       |        |

### Issues Found
1. 
2. 
3. 

### Next Steps
- [ ] All tests passed - system ready for production
- [ ] Minor issues found - document and monitor
- [ ] Major issues found - needs further debugging

---

## Emergency Procedures

### If Button 1 Fails (Software Control)
1. System can still be shut down via command line: `sudo shutdown -h now`
2. Use Button 2 for wake/boot
3. Check logs: `journalctl -u r2d2-powerbutton.service`
4. Restart service: `sudo systemctl restart r2d2-powerbutton.service`

### If Button 2 Fails (Hardware Power)
1. Use traditional power button location or hardware jumpers if available
2. Button 1 still functions for low-power/shutdown control
3. May need UART serial console for recovery

### If Both Buttons Fail
1. System remains running and can be controlled via SSH
2. Use `nvpmodel -m 2` to enter low-power manually
3. Use `sudo shutdown -h now` to shutdown
4. Physical access to Jetson needed to power on

---

## Sign-Off

**Tester Name:** ___________________

**Test Date:** December 9, 2025

**Result:** PASS / FAIL

**Notes:** 

_______________________________________________________________________________

_______________________________________________________________________________
