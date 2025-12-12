# R2D2 Web Dashboard

Web-based monitoring and control dashboard for the R2D2 system, accessible via Tailscale VPN.

## Features

- **Real-time Monitoring**: Live updates of person recognition status, system metrics, and events
- **Service Control**: Start, stop, and restart R2D2 services (audio, camera, etc.)
- **Volume Control**: Adjust audio volume with slider and presets
- **Three-State Visualization**: Visual display of recognition states (RED/BLUE/GREEN)
- **Face Recognition Training**: Complete training interface with all 7 menu options
- **Real-time Stream**: Live event stream showing recognition events and status changes

## Quick Start

### 1. Install Dependencies

```bash
cd ~/dev/r2d2/web_dashboard
python3 -m venv web_dashboard_env
source web_dashboard_env/bin/activate
pip install -r requirements.txt
```

### 2. Install rosbridge_suite

```bash
sudo apt install ros-humble-rosbridge-suite
```

### 3. Start rosbridge (in one terminal)

```bash
cd ~/dev/r2d2/web_dashboard
source /opt/ros/humble/setup.bash
ros2 launch web_dashboard/launch/rosbridge.launch.py
```

### 4. Start Web Dashboard (in another terminal)

```bash
cd ~/dev/r2d2/web_dashboard
source web_dashboard_env/bin/activate
./start_server.sh
```

### 5. Access Dashboard

Open browser and navigate to:
```
http://100.95.133.26:8080
```
(Replace with your Jetson's Tailscale IP)

## Architecture

- **Backend**: FastAPI (Python) - REST API and static file serving
- **Real-time**: rosbridge_server (WebSocket) - ROS 2 topic streaming
- **Frontend**: HTML/CSS/JavaScript - Dashboard UI
- **Access**: Tailscale VPN (already configured)

## API Endpoints

### Services
- `GET /api/services/status` - Get all services status
- `POST /api/services/{service_name}/start` - Start service
- `POST /api/services/{service_name}/stop` - Stop service
- `POST /api/services/{service_name}/restart` - Restart service

### Audio
- `GET /api/audio/volume` - Get current volume
- `POST /api/audio/volume` - Set volume (0.0-1.0)
- `GET /api/audio/parameters` - Get all audio parameters

### Training
- `POST /api/training/capture` - Start image capture
- `POST /api/training/add_pictures` - Add more pictures
- `POST /api/training/retrain` - Retrain model
- `GET /api/training/list` - List all people/models
- `DELETE /api/training/{person_name}` - Delete person
- `GET /api/training/status/{task_id}` - Get training task status

## Configuration

Edit `app/config.py` to change:
- Server host/port
- rosbridge host/port
- Service names
- Training script paths

## Troubleshooting

**Dashboard not loading:**
- Check if FastAPI server is running
- Check if rosbridge is running
- Verify Tailscale VPN connection

**No real-time updates:**
- Verify rosbridge is running on port 9090
- Check browser console for WebSocket errors
- Verify ROS 2 topics are publishing

**Service control not working:**
- Check sudo permissions (may need passwordless sudo)
- Verify service names in config.py match actual services

**Training not working:**
- Ensure training scripts exist in configured directory
- Check OPENBLAS_CORETYPE environment variable
- Verify depthai_env is accessible

## Security Notes

- Dashboard is accessible only via Tailscale VPN (not exposed to public Internet)
- Service control requires sudo (configure passwordless sudo for specific commands)
- No authentication implemented (relies on VPN security)

## Systemd Service (Optional)

Create `/etc/systemd/system/r2d2-web-dashboard.service`:

```ini
[Unit]
Description=R2D2 Web Dashboard
After=network.target

[Service]
Type=simple
User=severin
WorkingDirectory=/home/severin/dev/r2d2/web_dashboard
ExecStart=/home/severin/dev/r2d2/web_dashboard/scripts/start_web_dashboard.sh
Restart=always

[Install]
WantedBy=multi-user.target
```

Then:
```bash
sudo systemctl enable r2d2-web-dashboard.service
sudo systemctl start r2d2-web-dashboard.service
```

