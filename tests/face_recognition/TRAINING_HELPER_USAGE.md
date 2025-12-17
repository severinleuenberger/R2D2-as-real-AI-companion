# Training Helper Script - Usage Guide

**Date:** December 17, 2025  
**Script:** `train_with_service_management.sh`  
**Purpose:** Safe training with automatic camera service management

---

## Quick Start

### Simple Usage

```bash
cd ~/dev/r2d2/tests/face_recognition
./train_with_service_management.sh
```

That's it! The script handles everything:
- ✅ Stops camera service automatically
- ✅ Activates virtual environment
- ✅ Launches train_manager.py
- ✅ Restarts service when you exit (even with Ctrl+C)

---

## What It Does

### Automatic Service Management

**Before Training:**
1. Checks if `r2d2-camera-perception.service` is running
2. Stops the service (with sudo)
3. Verifies camera is free

**During Training:**
- You train normally using train_manager.py
- Press Ctrl+C anytime to exit safely

**After Training:**
- Automatically restarts the service
- Verifies service is running
- Shows confirmation message

---

## Features

### 🛡️ Safety Features

✅ **Trap Handler:** Service restarts even if:
- You press Ctrl+C
- Script crashes
- Training fails
- You kill the terminal

✅ **Pre-flight Checks:**
- Virtual environment exists
- Train script exists
- Not running as root
- Service status verified

✅ **Verification:**
- Service actually stopped before training
- Service actually restarted after training
- Clear status messages at each step

### 🎨 User-Friendly

✅ **Color-coded output:**
- 🔵 Blue: Information
- 🟢 Green: Success
- 🟡 Yellow: Warnings
- 🔴 Red: Errors

✅ **Clear progress:**
```
[1/5] Running pre-flight checks...
[2/5] Checking camera service status...
[3/5] Stopping camera service...
[4/5] Activating virtual environment...
[5/5] Launching training manager...
```

---

## Testing Scenarios

### Test 1: Normal Training Flow

**Steps:**
1. Run the script:
   ```bash
   ./train_with_service_management.sh
   ```
2. Enter sudo password when prompted
3. Do some training (or just open the menu)
4. Exit normally (option 0)

**Expected Result:**
- Service stops before training
- Training works normally
- Service restarts automatically
- Message: "✅ Service restarted successfully!"

---

### Test 2: Ctrl+C Interruption

**Steps:**
1. Run the script
2. Enter sudo password
3. Training menu appears
4. **Press Ctrl+C**

**Expected Result:**
- Script catches the interrupt
- Cleanup function runs
- Service restarts automatically
- Message: "✅ Service restarted successfully!"

---

### Test 3: Service Already Stopped

**Steps:**
1. Stop service manually:
   ```bash
   sudo systemctl stop r2d2-camera-perception.service
   ```
2. Run the script
3. Exit normally

**Expected Result:**
- Script detects service is not running
- Skips stop step
- Training works
- Script doesn't restart service (since it wasn't running initially)
- Message: "Service was not running initially. Not restarting."

---

### Test 4: Verify Service Status

**After using the script, verify everything is back to normal:**

```bash
# Check service is running
systemctl status r2d2-camera-perception.service

# Should show: "active (running)"

# Check ROS2 nodes
ros2 node list

# Should include: /oak_d_camera, /image_listener

# Check topics
ros2 topic list | grep perception

# Should show: /r2d2/perception/person_id, etc.
```

---

## Troubleshooting

### Issue: "Service did not restart properly"

**Solution:**
```bash
# Manually restart
sudo systemctl start r2d2-camera-perception.service

# Verify
systemctl status r2d2-camera-perception.service
```

### Issue: "Virtual environment not found"

**Solution:**
```bash
# Check if it exists
ls -la ~/depthai_env

# If not, create it (follow original setup instructions)
python3 -m venv ~/depthai_env
```

### Issue: "Permission denied"

**Solution:**
```bash
# Make script executable
chmod +x ~/dev/r2d2/tests/face_recognition/train_with_service_management.sh
```

### Issue: "Camera still in use"

If camera is still in use after service stops:

```bash
# Find what's using it
ps aux | grep -E "(camera|oak)" | grep -v grep

# Kill those processes
pkill -9 -f camera_node

# Then run the script again
```

---

## Advanced Usage

### Run from Anywhere

```bash
# Create alias (add to ~/.bashrc)
alias train_r2d2='~/dev/r2d2/tests/face_recognition/train_with_service_management.sh'

# Then just run:
train_r2d2
```

### Check Service Status Anytime

```bash
# Quick status check
systemctl status r2d2-camera-perception.service

# See if it's active
systemctl is-active r2d2-camera-perception.service
```

---

## Script Behavior

### What Gets Restarted

**YES - Automatically restarted:**
- `r2d2-camera-perception.service`

**NO - Not touched:**
- `r2d2-speech.service` (if exists)
- `r2d2-audio-notification.service`
- `r2d2-heartbeat.service`
- Any other R2D2 services

### Exit Codes

- `0` - Success (training completed, service restarted)
- `1` - Error (pre-flight check failed, service didn't stop/start)

---

## Comparison: Before vs After

### Before (Manual Process)

```bash
# 1. Stop service
sudo systemctl stop r2d2-camera-perception.service

# 2. Activate environment
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate

# 3. Train
python3 train_manager.py

# 4. Restart service (if you remember!)
sudo systemctl start r2d2-camera-perception.service

# 5. Verify (if you remember!)
systemctl status r2d2-camera-perception.service
```

**Risks:**
- ❌ Might forget to restart service
- ❌ If you Ctrl+C, service stays stopped
- ❌ If script crashes, service stays stopped
- ❌ No verification

### After (Automated Process)

```bash
./train_with_service_management.sh
```

**Benefits:**
- ✅ One command
- ✅ Service always restarts
- ✅ Handles interruptions
- ✅ Automatic verification
- ✅ Clear status messages

---

## Summary

**Single Command:**
```bash
./train_with_service_management.sh
```

**Safe Training:**
- Camera automatically freed
- Service automatically restarted
- Works even with Ctrl+C
- Clear feedback at each step

**Use this script every time you need to train face recognition or gestures!**

---

## Need Help?

If something doesn't work:

1. Check if service exists:
   ```bash
   systemctl list-unit-files | grep r2d2-camera
   ```

2. Check script permissions:
   ```bash
   ls -la train_with_service_management.sh
   ```

3. Run with bash explicitly:
   ```bash
   bash train_with_service_management.sh
   ```

4. Check the cleanup happened:
   ```bash
   systemctl status r2d2-camera-perception.service
   ```

