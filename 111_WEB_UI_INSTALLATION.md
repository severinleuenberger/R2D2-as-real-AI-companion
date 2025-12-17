# R2D2 Web UI System - Installation Guide
## Step-by-Step Setup and Configuration

**Date:** December 17, 2025  
**Platform:** NVIDIA Jetson AGX Orin 64GB + ROS 2 Humble  
**Purpose:** Complete installation guide for R2D2 Web Dashboard

---

## Prerequisites

Before starting, ensure the following components are installed and configured:

### Required Components
- ✅ **ROS 2 Humble** - Must be installed and working
- ✅ **Tailscale VPN** - Must be configured (see `012_VPN_SETUP_AND_REMOTE_ACCESS.md`)
- ✅ **R2D2 Camera Perception** - Camera node should be operational
- ✅ **R2D2 Audio Notification** - Audio notification node should be operational
- ✅ **Python 3.10+** - System Python installation

### Verification Commands
```bash
# Check ROS 2 installation
source /opt/ros/humble/setup.bash
ros2 --version

# Check Tailscale
tailscale status
tailscale ip -4  # Note this IP for later

# Check if camera perception is set up
ros2 topic list | grep /oak/rgb/image_raw

# Check if audio notification is set up
ros2 topic list | grep /r2d2/audio/person_status
```

---

## Phase 1: Install System Dependencies

### Step 1.1: Install rosbridge_suite

rosbridge provides the WebSocket bridge between ROS 2 and the web browser.

```bash
sudo apt update
sudo apt install ros-humble-rosbridge-suite
```

**Verify Installation:**
```bash
ros2 pkg list | grep rosbridge
# Should show:
# rosbridge_library
# rosbridge_server
# rosbridge_suite
```

### Step 1.2: Install System Packages

```bash
sudo apt install python3-pip python3-venv
```

---

## Phase 2: Set Up Web Dashboard Project

### Step 2.1: Navigate to Project Directory

```bash
cd ~/dev/r2d2/web_dashboard
```

**If directory doesn't exist:**
```bash
mkdir -p ~/dev/r2d2/web_dashboard
cd ~/dev/r2d2/web_dashboard
```

### Step 2.2: Create Python Virtual Environment

```bash
python3 -m venv web_dashboard_env
source web_dashboard_env/bin/activate
```

**Expected Output:**
```
(web_dashboard_env) severin@jetson:~/dev/r2d2/web_dashboard$
```

### Step 2.3: Install Python Dependencies

**Create requirements.txt** (if not already present):
```bash
cat > requirements.txt << 'EOF'
fastapi==0.104.1
uvicorn[standard]==0.24.0
python-multipart==0.0.6
jinja2==3.1.2
EOF
```

**Install dependencies:**
```bash
pip install -r requirements.txt
```

**Verify Installation:**
```bash
pip list | grep -E "fastapi|uvicorn"
# Should show:
# fastapi         0.104.1
# uvicorn         0.24.0
```

---

## Phase 3: Set Up Sudo Permissions for Service Control

To allow the web dashboard to control systemd services without password prompts:

### Step 3.1: Create Sudoers Configuration

```bash
sudo nano /etc/sudoers.d/r2d2-services
```

**Add this content:**
```
severin ALL=(ALL) NOPASSWD: /bin/systemctl start r2d2-*, /bin/systemctl stop r2d2-*, /bin/systemctl restart r2d2-*, /bin/systemctl status r2d2-*
```

**Save and exit:** Ctrl+X, Y, Enter

### Step 3.2: Set Correct Permissions

```bash
sudo chmod 0440 /etc/sudoers.d/r2d2-services
```

### Step 3.3: Test Sudo Permissions

```bash
sudo systemctl status r2d2-audio-notification.service
# Should work without asking for password
```

**⚠️ Security Note:** This grants passwordless sudo only for specific systemctl commands on r2d2-* services.

---

## Phase 4: Camera Stream Service Setup

The camera stream service converts the ROS 2 camera topic to an MJPEG HTTP stream.

### Step 4.1: Verify Camera Stream Node Exists

```bash
ls ~/dev/r2d2/ros2_ws/src/r2d2_camera/r2d2_camera/camera_stream_node.py
```

**If file doesn't exist**, the camera stream feature won't work. Check if it's in the correct location or refer to the camera setup documentation.

### Step 4.2: Create Camera Stream Systemd Service

```bash
sudo nano /etc/systemd/system/r2d2-camera-stream.service
```

**Add this content:**
```ini
[Unit]
Description=R2D2 Camera Stream Service (MJPEG HTTP)
After=network.target

[Service]
Type=simple
User=severin
Environment="ROS_DOMAIN_ID=0"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/severin/dev/r2d2/ros2_ws/install/setup.bash && ros2 run r2d2_camera camera_stream_node"
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**Save and exit:** Ctrl+X, Y, Enter

### Step 4.3: Enable Camera Stream Service (Optional)

```bash
# Reload systemd
sudo systemctl daemon-reload

# Enable service (auto-start on boot)
sudo systemctl enable r2d2-camera-stream.service

# Start service
sudo systemctl start r2d2-camera-stream.service

# Check status
sudo systemctl status r2d2-camera-stream.service
```

**Note:** You can also start/stop this service from the web dashboard.

---

## Phase 5: Enhanced Heartbeat Service Setup

The heartbeat service provides system health metrics (CPU, GPU, temperature).

### Step 5.1: Verify Enhanced Heartbeat Node

The enhanced heartbeat node should publish system metrics in JSON format.

```bash
# Check if heartbeat node exists
ls ~/dev/r2d2/ros2_ws/src/r2d2_hello/r2d2_hello/heartbeat_node.py
```

### Step 5.2: Create/Update Heartbeat Systemd Service

```bash
sudo nano /etc/systemd/system/r2d2-heartbeat.service
```

**Add this content:**
```ini
[Unit]
Description=R2D2 Heartbeat Service (Enhanced with System Metrics)
After=network.target

[Service]
Type=simple
User=severin
Environment="ROS_DOMAIN_ID=0"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/severin/dev/r2d2/ros2_ws/install/setup.bash && ros2 run r2d2_hello heartbeat_node"
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**Save and exit:** Ctrl+X, Y, Enter

### Step 5.3: Enable Heartbeat Service

```bash
# Reload systemd
sudo systemctl daemon-reload

# Enable service (auto-start on boot)
sudo systemctl enable r2d2-heartbeat.service

# Start service
sudo systemctl start r2d2-heartbeat.service

# Check status
sudo systemctl status r2d2-heartbeat.service
```

### Step 5.4: Test Heartbeat Data

```bash
ros2 topic echo /r2d2/heartbeat --once
```

**Expected Output:**
```json
data: '{"timestamp": "2025-12-17T10:30:00", "status": "running", "cpu_percent": 15.5, "gpu_percent": 8.2, "temperature_c": 42.3}'
```

---

## Phase 6: Configure and Start Services

### Step 6.1: Create Launch File for rosbridge

```bash
mkdir -p ~/dev/r2d2/web_dashboard/launch
nano ~/dev/r2d2/web_dashboard/launch/rosbridge.launch.py
```

**Add this content:**
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'port': 9090,
                'address': '0.0.0.0'
            }]
        )
    ])
```

**Save and exit:** Ctrl+X, Y, Enter

### Step 6.2: Create Startup Scripts

**Create rosbridge startup script:**
```bash
cat > ~/dev/r2d2/web_dashboard/start_rosbridge.sh << 'EOF'
#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/dev/r2d2/ros2_ws/install/setup.bash
ros2 launch ~/dev/r2d2/web_dashboard/launch/rosbridge.launch.py
EOF

chmod +x ~/dev/r2d2/web_dashboard/start_rosbridge.sh
```

**Create web server startup script:**
```bash
cat > ~/dev/r2d2/web_dashboard/start_server.sh << 'EOF'
#!/bin/bash
cd ~/dev/r2d2/web_dashboard
source web_dashboard_env/bin/activate
uvicorn app.main:app --host 0.0.0.0 --port 8080 --reload
EOF

chmod +x ~/dev/r2d2/web_dashboard/start_server.sh
```

**Create combined startup script:**
```bash
mkdir -p ~/dev/r2d2/web_dashboard/scripts
cat > ~/dev/r2d2/web_dashboard/scripts/start_web_dashboard.sh << 'EOF'
#!/bin/bash
# Start R2D2 Web Dashboard (rosbridge + web server)

# Start rosbridge in background
echo "Starting rosbridge..."
cd ~/dev/r2d2/web_dashboard
./start_rosbridge.sh &
ROSBRIDGE_PID=$!

# Wait for rosbridge to initialize
sleep 3

# Start web server
echo "Starting web server..."
./start_server.sh

# Cleanup on exit
trap "kill $ROSBRIDGE_PID" EXIT
EOF

chmod +x ~/dev/r2d2/web_dashboard/scripts/start_web_dashboard.sh
```

---

## Phase 7: Verify Installation

### Step 7.1: Test rosbridge

**Terminal 1: Start rosbridge**
```bash
cd ~/dev/r2d2/web_dashboard
./start_rosbridge.sh
```

**Expected Output:**
```
[rosbridge_websocket-1] [INFO] [rosbridge_websocket]: Rosbridge WebSocket server started on port 9090
```

**Terminal 2: Test WebSocket connection**
```bash
# Check if port is open
netstat -tuln | grep 9090

# Check if rosbridge node is running
ros2 node list | grep rosbridge
```

### Step 7.2: Test Web Server

**Terminal 2: Start web server**
```bash
cd ~/dev/r2d2/web_dashboard
source web_dashboard_env/bin/activate
./start_server.sh
```

**Expected Output:**
```
INFO:     Uvicorn running on http://0.0.0.0:8080 (Press CTRL+C to quit)
INFO:     Started reloader process
INFO:     Started server process
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

**Terminal 3: Test HTTP access**
```bash
curl http://localhost:8080
# Should return HTML content
```

### Step 7.3: Test from Browser (Local)

On the Jetson, open a browser and navigate to:
```
http://localhost:8080
```

You should see the R2D2 Web Dashboard.

### Step 7.4: Test from Remote Computer (via Tailscale)

From your Windows laptop (connected to Tailscale VPN):

1. Open a browser
2. Navigate to: `http://100.95.133.26:8080`
   (Replace with your Jetson's Tailscale IP)
3. Dashboard should load and show real-time updates

---

## Phase 8: Troubleshooting Common Installation Issues

### Issue: rosbridge_suite not found

**Error:** `Package 'rosbridge_suite' not found`

**Solution:**
```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install ros-humble-rosbridge-suite
```

### Issue: FastAPI not found

**Error:** `ModuleNotFoundError: No module named 'fastapi'`

**Solution:**
```bash
cd ~/dev/r2d2/web_dashboard
source web_dashboard_env/bin/activate
pip install fastapi uvicorn
```

### Issue: Permission denied for systemctl

**Error:** `Failed to start service: Permission denied`

**Solution:**
```bash
# Check sudoers file
sudo visudo -c /etc/sudoers.d/r2d2-services

# Fix permissions
sudo chmod 0440 /etc/sudoers.d/r2d2-services

# Test
sudo systemctl status r2d2-audio-notification.service
```

### Issue: Port 8080 already in use

**Error:** `Address already in use`

**Solution:**
```bash
# Find process using port 8080
sudo lsof -i :8080

# Kill the process (if safe)
kill <PID>

# Or use a different port
uvicorn app.main:app --host 0.0.0.0 --port 8081
```

### Issue: rosbridge connection fails

**Symptoms:** Dashboard loads but no real-time updates

**Solution:**
```bash
# Check if rosbridge is running
ros2 node list | grep rosbridge

# Check if port 9090 is open
netstat -tuln | grep 9090

# Restart rosbridge
pkill -f rosbridge_websocket
./start_rosbridge.sh
```

### Issue: Camera stream not working

**Symptoms:** Camera stream service starts but no video

**Solution:**
```bash
# Check if camera is publishing
ros2 topic hz /oak/rgb/image_raw

# Check camera stream logs
sudo journalctl -u r2d2-camera-stream.service -f

# Restart camera service
sudo systemctl restart r2d2-camera-perception.service
sudo systemctl restart r2d2-camera-stream.service
```

---

## Phase 9: Optional - Create Systemd Services for Web Dashboard

For production deployment, create systemd services so the web dashboard starts automatically on boot.

### Step 9.1: Create rosbridge Service

```bash
sudo nano /etc/systemd/system/r2d2-rosbridge.service
```

**Add this content:**
```ini
[Unit]
Description=R2D2 rosbridge WebSocket Server
After=network.target

[Service]
Type=simple
User=severin
Environment="ROS_DOMAIN_ID=0"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/severin/dev/r2d2/ros2_ws/install/setup.bash && ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090 -p address:=0.0.0.0"
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**Save and exit**

### Step 9.2: Create Web Dashboard Service

```bash
sudo nano /etc/systemd/system/r2d2-web-dashboard.service
```

**Add this content:**
```ini
[Unit]
Description=R2D2 Web Dashboard (FastAPI)
After=network.target r2d2-rosbridge.service
Requires=r2d2-rosbridge.service

[Service]
Type=simple
User=severin
WorkingDirectory=/home/severin/dev/r2d2/web_dashboard
Environment="PATH=/home/severin/dev/r2d2/web_dashboard/web_dashboard_env/bin"
ExecStart=/home/severin/dev/r2d2/web_dashboard/web_dashboard_env/bin/uvicorn app.main:app --host 0.0.0.0 --port 8080
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**Save and exit**

### Step 9.3: Enable and Start Services

```bash
# Reload systemd
sudo systemctl daemon-reload

# Enable services (auto-start on boot)
sudo systemctl enable r2d2-rosbridge.service
sudo systemctl enable r2d2-web-dashboard.service

# Start services
sudo systemctl start r2d2-rosbridge.service
sudo systemctl start r2d2-web-dashboard.service

# Check status
sudo systemctl status r2d2-rosbridge.service
sudo systemctl status r2d2-web-dashboard.service
```

### Step 9.4: Test Auto-Start

```bash
# Reboot Jetson
sudo reboot

# After reboot, check if services are running
sudo systemctl status r2d2-rosbridge.service
sudo systemctl status r2d2-web-dashboard.service

# Test dashboard access
curl http://localhost:8080
```

---

## Phase 10: Final Verification Checklist

Before considering the installation complete, verify all components:

### ✅ Core Components
- [ ] rosbridge_suite installed
- [ ] FastAPI and dependencies installed
- [ ] Virtual environment created and activated
- [ ] Sudo permissions configured for service control

### ✅ Services Running
- [ ] rosbridge running (port 9090)
- [ ] Web dashboard running (port 8080)
- [ ] Camera stream service exists (optional)
- [ ] Heartbeat service running (optional)

### ✅ Network Access
- [ ] Dashboard accessible locally (http://localhost:8080)
- [ ] Dashboard accessible via Tailscale (http://100.95.133.26:8080)
- [ ] rosbridge WebSocket connection working
- [ ] Camera stream accessible (http://100.95.133.26:8081/stream)

### ✅ Functionality
- [ ] Recognition status updates in real-time
- [ ] Service control buttons work (start/stop/restart)
- [ ] Volume control slider works
- [ ] Training interface accessible
- [ ] Event stream shows live events
- [ ] System health metrics display (if heartbeat enabled)

### ✅ Optional Features
- [ ] Systemd services enabled for auto-start
- [ ] Camera stream service working
- [ ] Heartbeat service showing metrics

---

## Next Steps

After successful installation:

1. **Read the Quick Start Guide:** [`112_WEB_UI_QUICK_START.md`](112_WEB_UI_QUICK_START.md)
2. **Review the Reference Documentation:** [`110_WEB_UI_REFERENCE.md`](110_WEB_UI_REFERENCE.md)
3. **Test all features:** Try service control, volume adjustment, training
4. **Set up systemd services:** For production auto-start (Phase 9)
5. **Configure firewall (optional):** If needed beyond Tailscale

---

## Support and Troubleshooting

If you encounter issues not covered in this guide:

1. Check the **Troubleshooting Guide** in [`110_WEB_UI_REFERENCE.md`](110_WEB_UI_REFERENCE.md)
2. Review log files:
   - Web dashboard: Terminal output or systemd journal
   - rosbridge: Terminal output or systemd journal
   - Services: `sudo journalctl -u <service-name> -f`
3. Verify ROS 2 topics are publishing: `ros2 topic list`
4. Check network connectivity: `tailscale status`, `ping 100.95.133.26`

---

**Document Version:** 1.0  
**Last Updated:** December 17, 2025  
**Platform:** NVIDIA Jetson AGX Orin 64GB with ROS 2 Humble  
**Installation Time:** Approximately 30-45 minutes


