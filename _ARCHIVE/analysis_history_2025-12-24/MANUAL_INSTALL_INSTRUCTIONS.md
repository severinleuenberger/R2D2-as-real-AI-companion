# Manual Installation Instructions

Since sudo requires a password, please run these commands **manually on the Jetson**:

## Step 1: Install the Services

```bash
cd /home/severin/dev/r2d2

# Copy service files to systemd
sudo cp r2d2-camera-stream.service /etc/systemd/system/
sudo cp r2d2-heartbeat.service /etc/systemd/system/

# Set correct permissions
sudo chmod 644 /etc/systemd/system/r2d2-camera-stream.service
sudo chmod 644 /etc/systemd/system/r2d2-heartbeat.service

# Reload systemd
sudo systemctl daemon-reload
```

## Step 2: Start the Services

```bash
# Start heartbeat service
sudo systemctl start r2d2-heartbeat.service

# Start camera stream service
sudo systemctl start r2d2-camera-stream.service
```

## Step 3: Enable Auto-Start (Optional)

```bash
sudo systemctl enable r2d2-heartbeat.service
sudo systemctl enable r2d2-camera-stream.service
```

## Step 4: Verify

```bash
# Check service status
sudo systemctl status r2d2-heartbeat.service
sudo systemctl status r2d2-camera-stream.service

# Check if heartbeat topic is publishing
ros2 topic echo /r2d2/heartbeat --once

# Check if camera stream is responding
curl http://localhost:8081/health
```

## Step 5: Refresh Browser

After starting the services, refresh your browser at:
**http://100.95.133.26:8080/**

You should now see:
- ✅ System Health metrics (CPU, GPU, Temperature)
- ✅ Camera stream toggle button
- ✅ Real-time status updates

## Troubleshooting

If services fail to start, check logs:
```bash
sudo journalctl -u r2d2-heartbeat.service -n 50
sudo journalctl -u r2d2-camera-stream.service -n 50
```

