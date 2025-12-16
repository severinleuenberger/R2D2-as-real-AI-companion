# Post-Reboot Verification - Quick Reference

## Quick Verification (30 seconds)

After system reboot, wait 30 seconds, then run:

```bash
cd ~/dev/r2d2
./verify_person_recognition_system.sh
```

## Manual Verification Commands

If you prefer manual verification, use these commands:

### 1. Check Services Are Running
```bash
sudo systemctl status r2d2-audio-notification.service
sudo systemctl status r2d2-camera-perception.service
```
**Expected:** `Active: active (running)`

### 2. Verify Autostart Enabled
```bash
systemctl is-enabled r2d2-audio-notification.service
systemctl is-enabled r2d2-camera-perception.service
```
**Expected:** `enabled`

### 3. Verify Topics Are Publishing
```bash
source /opt/ros/humble/setup.bash
source ~/dev/r2d2/ros2_ws/install/setup.bash
timeout 3s ros2 topic echo /r2d2/audio/person_status --once --no-arr
```
**Expected:** JSON message with `"status": "blue"` or `"status": "red"`

### 4. Verify New Code Is Running
```bash
journalctl -u r2d2-audio-notification.service -n 20 | grep "BLUE_HOLD_TIME"
```
**Expected:** `BLUE_HOLD_TIME: 5.0s (minimum BLUE state duration)`

### 5. Verify Restart-on-Failure
```bash
systemctl show r2d2-audio-notification.service | grep Restart
```
**Expected:** `Restart=on-failure`

## Service Configuration Summary

| Service | Status | Autostart | Restart-on-Failure |
|---------|--------|-----------|-------------------|
| `r2d2-camera-perception.service` | ✅ Active | ✅ Enabled | ✅ Configured |
| `r2d2-audio-notification.service` | ✅ Active | ✅ Enabled | ✅ Configured |

## What Happens on Reboot

1. **System boots** → Services start automatically (enabled)
2. **Camera service starts** → Publishes `/r2d2/perception/person_id`
3. **Audio service starts** → Subscribes to person_id, publishes `/r2d2/audio/person_status`
4. **If service crashes** → Systemd automatically restarts after 5 seconds (restart-on-failure)

## Troubleshooting

If verification fails:

1. **Service not running:**
   ```bash
   sudo systemctl restart r2d2-audio-notification.service
   sudo systemctl restart r2d2-camera-perception.service
   ```

2. **Service not enabled:**
   ```bash
   sudo systemctl enable r2d2-audio-notification.service
   sudo systemctl enable r2d2-camera-perception.service
   ```

3. **Check logs:**
   ```bash
   journalctl -u r2d2-audio-notification.service -n 50
   journalctl -u r2d2-camera-perception.service -n 50
   ```

4. **Rebuild if code changed:**
   ```bash
   cd ~/dev/r2d2/ros2_ws
   colcon build --packages-select r2d2_audio
   sudo systemctl restart r2d2-audio-notification.service
   ```

