# Training Helper Script - Implementation Complete

**Date:** December 17, 2025  
**Status:** ✅ READY FOR TESTING

---

## What Was Implemented

### Script: `train_with_service_management.sh`

**Location:** `/home/severin/dev/r2d2/tests/face_recognition/`

**Features:**
- ✅ Automatic service stop/start
- ✅ Trap handler for Ctrl+C safety
- ✅ Pre-flight checks (venv, scripts, permissions)
- ✅ Color-coded status messages
- ✅ Comprehensive error handling
- ✅ Service verification before/after
- ✅ Progress indicators [1/5] through [5/5]

---

## How to Use

### Command

```bash
cd ~/dev/r2d2/tests/face_recognition
./train_with_service_management.sh
```

### What Happens

**Automatic Flow:**
```
[1/5] Pre-flight checks ✓
  ↓
[2/5] Check service status ✓
  ↓
[3/5] Stop camera service (sudo required)
  ↓
[4/5] Activate virtual environment ✓
  ↓
[5/5] Launch train_manager.py
  ↓
YOU TRAIN NORMALLY
  ↓
You exit (option 0 or Ctrl+C)
  ↓
CLEANUP (automatic via trap)
  ↓
Service restarts ✅
  ↓
Verification ✅
  ↓
Done!
```

---

## Safety Features

### Trap Handler

The script uses bash `trap` to **always** run cleanup:

```bash
trap cleanup EXIT INT TERM
```

This means cleanup runs on:
- ✅ Normal exit (option 0)
- ✅ Ctrl+C (SIGINT)
- ✅ Terminal closed (SIGTERM)
- ✅ Script errors (EXIT)

**Result:** Service **ALWAYS** restarts, no matter what!

### State Tracking

```bash
SERVICE_WAS_RUNNING=false

# Sets to true only if service was actually running
# Only restarts if it was running initially
```

**Benefit:** If service was already stopped, script won't restart it.

---

## Testing Checklist

### ✅ Pre-Implementation Tests

- ✅ Script created
- ✅ Script is executable (`chmod +x`)
- ✅ Syntax validated (`bash -n`)
- ✅ Documentation created

### 🧪 User Testing Required

**Test 1: Normal Flow**
1. Run: `./train_with_service_management.sh`
2. Enter sudo password
3. Open train_manager menu
4. Exit with option [0]
5. **Verify:** Service restarted, message shows "✅ Service restarted successfully!"

**Test 2: Ctrl+C Interruption**
1. Run the script
2. Enter sudo password
3. **Press Ctrl+C** during training menu
4. **Verify:** Cleanup runs, service restarts

**Test 3: Verify Functionality After**
1. Check service status:
   ```bash
   systemctl status r2d2-camera-perception.service
   ```
2. Check ROS2 nodes:
   ```bash
   ros2 node list | grep -E "(camera|image_listener)"
   ```
3. **Verify:** Everything is back to normal

---

## Expected Output

### Successful Run

```
========================================================================
               R2D2 Training with Service Management
========================================================================

[1/5] Running pre-flight checks...
✓ Pre-flight checks passed

[2/5] Checking camera service status...
✓ Service is running

[3/5] Stopping camera service...
This requires sudo password.

[sudo] password for severin: 
✓ Service stopped successfully

[4/5] Activating virtual environment...
Virtual env: /home/severin/depthai_env
✓ Virtual environment activated
Python: /home/severin/depthai_env/bin/python3

[5/5] Launching training manager...

========================================================================
                    Training Manager Starting
========================================================================

You can now train face recognition and gestures.
The camera service will automatically restart when you exit.

Press Ctrl+C anytime to exit safely.

========================================================================

[train_manager.py menu appears]

... training happens ...

✓ Training completed

========================================================================
                      Cleaning Up...
========================================================================
Restarting camera perception service...
✓ Service start command executed
✅ Service restarted successfully!

Camera perception is now active and running.
========================================================================
```

---

## Verification Commands

### After Running Script

```bash
# 1. Check service status
systemctl status r2d2-camera-perception.service
# Should be: active (running)

# 2. Check ROS2 ecosystem
ros2 node list
# Should include: /oak_d_camera, /image_listener, /audio_notification_node

# 3. Check topics
ros2 topic echo /r2d2/perception/person_id --once
# Should get a message (blue/red/green status)

# 4. Check camera is being used
ps aux | grep camera_node
# Should see the camera_node process
```

---

## Workflow Integration

### Before This Script

**Manual steps (error-prone):**
1. Stop service manually
2. Activate venv manually
3. Run training manually
4. **Remember to restart service** ← Easy to forget!
5. Verify manually

**Problems:**
- Forgot to restart → Camera stays offline
- Ctrl+C → Service stays stopped
- Complex, many steps

### With This Script

**One command:**
```bash
./train_with_service_management.sh
```

**Benefits:**
- Simple
- Safe
- Automatic
- Error-proof

---

## Integration with Person Registry

This script works seamlessly with the new Person Entity Management system:

```bash
# 1. Run training helper
./train_with_service_management.sh

# 2. In train_manager menu, choose:
#    [1] Train new person → Auto-registers in person registry
#    [8] Train gestures → Auto-updates person registry
#    [14] Manage persons → View/edit person registry

# 3. Exit normally

# 4. Service restarts automatically

# 5. Person is now registered with face/gesture models linked!
```

---

## Future Enhancements (Optional)

### Possible Additions

- Add option to train without stopping service (if camera not needed)
- Add option to restart perception with different parameters
- Add automatic backup before training
- Add training session logging
- Integration with web dashboard

### Current Status

**As implemented:**
- ✅ Stops one service: `r2d2-camera-perception.service`
- ✅ Restarts same service
- ✅ No changes to other services

**Future:**
Could be extended to manage multiple services if needed.

---

## Summary

**Single command for safe training:**
```bash
./train_with_service_management.sh
```

**Guarantees:**
- ✅ Camera service stops for training
- ✅ Training environment properly set up
- ✅ Service always restarts (even with Ctrl+C)
- ✅ Full verification

**Status:** Production ready, pending user testing

---

## Next Steps

1. **Run Test 1** (normal flow)
2. **Run Test 2** (Ctrl+C test)
3. **Verify** service is working after tests
4. **Use this script** for all future training!

The script ensures your R2D2 system is **always safe** during training. 🛡️

