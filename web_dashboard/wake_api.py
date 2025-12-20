"""
Minimal Wake API for R2D2 Web UI Service Mode.
Runs with minimal resources to allow starting/stopping the full Web UI.
"""
import subprocess
import os
import json
import asyncio
from pathlib import Path
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse
import uvicorn

# Configuration
TAILSCALE_IP = "100.95.133.26"
PORT = 8079
SERVICES = [
    "r2d2-rosbridge.service",
    "r2d2-web-dashboard.service",
    "r2d2-camera-stream.service"
]

app = FastAPI(title="R2D2 Wake API", docs_url=None, redoc_url=None)

HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>R2D2 Service Mode</title>
    <style>
        body {
            font-family: 'Segoe UI', Roboto, Helvetica, Arial, sans-serif;
            background-color: #0d1117;
            color: #c9d1d9;
            display: flex;
            justify-content: center;
            align-items: center;
            height: 100vh;
            margin: 0;
        }
        .container {
            background-color: #161b22;
            padding: 2rem;
            border-radius: 12px;
            box-shadow: 0 8px 24px rgba(0,0,0,0.5);
            text-align: center;
            width: 350px;
            border: 1px solid #30363d;
        }
        h1 { margin-top: 0; color: #58a6ff; font-size: 1.5rem; }
        .status-box {
            background-color: #0d1117;
            border: 1px solid #30363d;
            border-radius: 6px;
            padding: 1rem;
            margin: 1.5rem 0;
            text-align: left;
        }
        .status-item {
            display: flex;
            justify-content: space-between;
            margin-bottom: 0.5rem;
            font-size: 0.9rem;
        }
        .status-item:last-child { margin-bottom: 0; }
        .label { color: #8b949e; }
        .value { font-weight: bold; }
        .online-indicator {
            display: inline-block;
            width: 10px;
            height: 10px;
            border-radius: 50%;
            background-color: #238636;
            margin-right: 5px;
        }
        .offline-indicator {
            display: inline-block;
            width: 10px;
            height: 10px;
            border-radius: 50%;
            background-color: #da3633;
            margin-right: 5px;
        }
        .btn {
            background-color: #238636;
            color: white;
            border: none;
            padding: 12px 24px;
            border-radius: 6px;
            font-size: 1rem;
            cursor: pointer;
            transition: background-color 0.2s;
            width: 100%;
            font-weight: bold;
        }
        .btn:hover { background-color: #2ea043; }
        .btn:disabled { background-color: #30363d; cursor: not-allowed; color: #8b949e; }
        .btn-stop { background-color: #da3633; margin-top: 10px; }
        .btn-stop:hover { background-color: #f85149; }
        .spinner {
            border: 3px solid rgba(255,255,255,0.1);
            border-radius: 50%;
            border-top: 3px solid #58a6ff;
            width: 20px;
            height: 20px;
            animation: spin 1s linear infinite;
            display: inline-block;
            vertical-align: middle;
            margin-right: 8px;
        }
        @keyframes spin { 0% { transform: rotate(0deg); } 100% { transform: rotate(360deg); } }
        .service-status { margin-top: 1rem; font-size: 0.85rem; color: #8b949e; }
    </style>
</head>
<body>
    <div class="container">
        <h1>R2D2 Service Mode</h1>
        
        <div class="status-box">
            <div style="margin-bottom: 10px; font-weight: bold; color: #58a6ff;">SYSTEM HEARTBEAT</div>
            <div id="heartbeat-data">
                <div class="status-item"><span class="label">Status:</span> <span>Checking...</span></div>
            </div>
        </div>

        <button id="start-btn" class="btn" onclick="startWebUI()">Start Web UI</button>
        <div id="web-ui-status" class="service-status">Web UI is stopped</div>
    </div>

    <script>
        const TAILSCALE_IP = "100.95.133.26";
        const DASHBOARD_PORT = 8080;
        
        async function updateHeartbeat() {
            try {
                const response = await fetch('/api/heartbeat');
                const data = await response.json();
                
                const box = document.getElementById('heartbeat-data');
                if (data.online) {
                    box.innerHTML = `
                        <div class="status-item">
                            <span class="label">Status:</span> 
                            <span><span class="online-indicator"></span>Online</span>
                        </div>
                        <div class="status-item">
                            <span class="label">CPU:</span> <span class="value">${data.cpu}%</span>
                        </div>
                        <div class="status-item">
                            <span class="label">GPU:</span> <span class="value">${data.gpu}%</span>
                        </div>
                        <div class="status-item">
                            <span class="label">Temp:</span> <span class="value">${data.temp}°C</span>
                        </div>
                        <div class="status-item">
                            <span class="label">Last Seen:</span> <span class="value">${data.ago}s ago</span>
                        </div>
                    `;
                } else {
                    box.innerHTML = `
                        <div class="status-item">
                            <span class="label">Status:</span> 
                            <span><span class="offline-indicator"></span>Offline</span>
                        </div>
                        <div class="status-item" style="color: #da3633; font-size: 0.8rem;">
                            No heartbeat received from ROS 2
                        </div>
                    `;
                }
            } catch (e) {
                console.error("Heartbeat fetch error", e);
            }
        }

        async function checkServiceStatus() {
            try {
                const response = await fetch('/api/status');
                const data = await response.json();
                
                const statusEl = document.getElementById('web-ui-status');
                const btn = document.getElementById('start-btn');
                
                if (data.running) {
                    statusEl.innerHTML = '<span style="color: #238636">Web UI is running</span>';
                    statusEl.innerHTML += ' <a href="http://' + TAILSCALE_IP + ':' + DASHBOARD_PORT + '" style="color: #58a6ff; margin-left: 5px;">Go to Dashboard &rarr;</a>';
                    btn.textContent = "Web UI Running";
                    btn.disabled = true;
                    // Auto-redirect if just started
                    if (localStorage.getItem('starting_web_ui') === 'true') {
                        localStorage.removeItem('starting_web_ui');
                        window.location.href = `http://${TAILSCALE_IP}:${DASHBOARD_PORT}`;
                    }
                } else {
                    statusEl.textContent = 'Web UI is stopped';
                    btn.textContent = "Start Web UI";
                    btn.disabled = false;
                }
            } catch (e) {
                console.error("Status check error", e);
            }
        }

        async function startWebUI() {
            const btn = document.getElementById('start-btn');
            const statusEl = document.getElementById('web-ui-status');
            
            btn.disabled = true;
            btn.innerHTML = '<span class="spinner"></span>Starting...';
            statusEl.textContent = 'Starting services (approx 5s)...';
            
            try {
                const response = await fetch('/api/start', { method: 'POST' });
                const data = await response.json();
                
                if (data.success) {
                    localStorage.setItem('starting_web_ui', 'true');
                    // Poll for status
                    let checks = 0;
                    const interval = setInterval(() => {
                        checkServiceStatus();
                        checks++;
                        if (checks > 20) clearInterval(interval); // Stop after 20s
                    }, 1000);
                } else {
                    btn.textContent = "Start Failed";
                    btn.style.backgroundColor = "#da3633";
                    statusEl.textContent = "Error: " + data.error;
                    setTimeout(() => { 
                        btn.textContent = "Start Web UI"; 
                        btn.disabled = false;
                        btn.style.backgroundColor = "#238636";
                    }, 3000);
                }
            } catch (e) {
                btn.textContent = "Request Failed";
                statusEl.textContent = "Network error";
                console.error(e);
            }
        }

        // Poll every 4 seconds
        setInterval(updateHeartbeat, 4000);
        setInterval(checkServiceStatus, 4000);
        
        // Initial check
        updateHeartbeat();
        checkServiceStatus();
    </script>
</body>
</html>
"""

@app.get("/", response_class=HTMLResponse)
async def index():
    return HTML_TEMPLATE

@app.get("/api/heartbeat")
async def get_heartbeat():
    """Get system heartbeat from ROS 2 topic via subprocess"""
    try:
        # Run ros2 topic echo --once
        # Note: We need to source ROS 2 setup in the service definition, 
        # so this assumes 'ros2' is in PATH or alias
        result = subprocess.run(
            ['ros2', 'topic', 'echo', '/r2d2/heartbeat', '--once', '--no-arr'],
            capture_output=True, 
            text=True, 
            timeout=5
        )
        
        if result.returncode != 0 or not result.stdout:
            return {"online": False}
            
        # Parse output: data: "{\"timestamp\": ...}"
        output = result.stdout.strip()
        if "data: " in output:
            # Get everything after "data: "
            raw_data = output.split("data: ")[1]
            # Take only the first line (ignore --- separator if present)
            json_str = raw_data.split("\n")[0].strip()
            # Remove surrounding quotes
            json_str = json_str.strip("'").strip('"')
            # Handle escaped quotes if present
            json_str = json_str.replace('\\"', '"')
            
            try:
                data = json.loads(json_str)
                return {
                    "online": True,
                    "cpu": data.get("cpu_percent", "--"),
                    "gpu": data.get("gpu_percent", "--"),
                    "temp": data.get("temperature_c", "--"),
                    "ago": 0 
                }
            except json.JSONDecodeError:
                pass
                
        return {"online": True, "cpu": "?", "gpu": "?", "temp": "?", "ago": "?"}
        
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return {"online": False}
    except Exception as e:
        print(f"Heartbeat error: {e}")
        return {"online": False}
    except Exception as e:
        print(f"Heartbeat error: {e}")
        return {"online": False}

@app.get("/api/status")
async def get_status():
    """Check if Web UI services are running"""
    # Check if web dashboard service is active
    result = subprocess.run(
        ['systemctl', 'is-active', 'r2d2-web-dashboard.service'],
        capture_output=True,
        text=True
    )
    is_running = result.stdout.strip() == 'active'
    return {"running": is_running}

@app.post("/api/start")
async def start_services():
    """Start all Web UI services"""
    try:
        # Start all services
        cmd = f"sudo systemctl start {' '.join(SERVICES)}"
        proc = await asyncio.create_subprocess_shell(
            cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE
        )
        stdout, stderr = await proc.communicate()
        
        if proc.returncode == 0:
            return {"success": True}
        else:
            return {"success": False, "error": stderr.decode()}
    except Exception as e:
        return {"success": False, "error": str(e)}

@app.post("/api/stop")
async def stop_services():
    """Stop all Web UI services"""
    try:
        # Stop all services
        cmd = f"sudo systemctl stop {' '.join(SERVICES)}"
        proc = await asyncio.create_subprocess_shell(
            cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE
        )
        stdout, stderr = await proc.communicate()
        
        if proc.returncode == 0:
            return {"success": True}
        else:
            return {"success": False, "error": stderr.decode()}
    except Exception as e:
        return {"success": False, "error": str(e)}

if __name__ == "__main__":
    uvicorn.run(app, host=TAILSCALE_IP, port=PORT)

