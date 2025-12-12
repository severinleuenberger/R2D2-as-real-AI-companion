# Camera Already in Use - Fix Instructions

The camera is currently being used by another process. You need to stop it before running the training capture.

## Quick Fix

### Option 1: Stop ROS 2 Camera Node (if running)

```bash
# Find and stop camera node
ros2 node list | grep camera
# If you see camera_node, stop the launch file that started it
# Press Ctrl+C in the terminal where you launched it
```

### Option 2: Kill All Camera Processes

```bash
# Find camera processes
ps aux | grep -E "(camera|depthai|oak)" | grep -v grep

# Kill them (replace PID with actual process ID)
kill <PID>
```

### Option 3: Restart Camera Service (if running as service)

```bash
# Check for camera service
systemctl list-units | grep camera

# Stop it
sudo systemctl stop <camera-service-name>
```

## After Stopping Camera

Once the camera is free, run the capture:

```bash
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8
python3 1_capture_training_data.py
```

Or use the launcher:

```bash
cd ~/dev/r2d2/tests/face_recognition
./run_improved_capture.sh
```

---

**Note:** The camera can only be used by one process at a time. Make sure all other camera processes are stopped before starting training capture.

