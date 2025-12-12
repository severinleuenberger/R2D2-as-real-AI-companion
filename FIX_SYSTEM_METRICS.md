# Fix System Metrics Not Showing

## Problem
System health metrics (CPU, GPU, temperature) are not displaying in the web dashboard.

## Root Cause
The `r2d2-heartbeat.service` is not installed or not running. This service publishes system metrics to the `/r2d2/heartbeat` ROS 2 topic.

## Solution

### Step 1: Install the Services
```bash
cd /home/severin/dev/r2d2
./install_camera_and_heartbeat_services.sh
```

### Step 2: Start the Heartbeat Service
```bash
sudo systemctl start r2d2-heartbeat.service
```

### Step 3: Verify It's Working
```bash
# Check service status
sudo systemctl status r2d2-heartbeat.service

# Check if topic is publishing
ros2 topic echo /r2d2/heartbeat --once

# Check if node is running
ros2 node list | grep heartbeat
```

### Step 4: Enable Auto-Start (Optional)
```bash
sudo systemctl enable r2d2-heartbeat.service
```

## Expected Result
After starting the service, you should see:
- ✅ System Health panel shows "🟢 R2D2 is Running"
- ✅ CPU usage bar and percentage
- ✅ GPU usage bar and percentage  
- ✅ Temperature with color coding (green <50°C, yellow 50-70°C, red >70°C)
- ✅ Last update timestamp

## Troubleshooting

### If metrics still don't show:
1. **Check rosbridge is running:**
   ```bash
   netstat -tulnp | grep 9090
   # Should show: tcp 0.0.0.0:9090 LISTEN
   ```

2. **Check ROS connection in browser:**
   - Open browser console (F12)
   - Look for ROS connection status
   - Should see: "✓ Real-time Connected"

3. **Check heartbeat topic:**
   ```bash
   ros2 topic hz /r2d2/heartbeat
   # Should show ~1 Hz
   ```

4. **Check service logs:**
   ```bash
   sudo journalctl -u r2d2-heartbeat.service -f
   ```

## Quick Fix Script
```bash
# Install and start heartbeat service
cd /home/severin/dev/r2d2
./install_camera_and_heartbeat_services.sh
sudo systemctl start r2d2-heartbeat.service
sudo systemctl enable r2d2-heartbeat.service
```

