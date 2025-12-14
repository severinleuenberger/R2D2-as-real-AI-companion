"""Status and monitoring API endpoints"""
from fastapi import APIRouter
import subprocess
import socket
from typing import Dict

router = APIRouter(prefix="/api/status", tags=["status"])


@router.get("/nodes")
async def get_ros_nodes():
    """Get list of running ROS 2 nodes"""
    result = subprocess.run(
        ['ros2', 'node', 'list'],
        capture_output=True, text=True
    )
    nodes = [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]
    return {"nodes": nodes}


@router.get("/topics")
async def get_ros_topics():
    """Get list of ROS 2 topics"""
    result = subprocess.run(
        ['ros2', 'topic', 'list'],
        capture_output=True, text=True,
        env={**subprocess.os.environ, 'ROS_DOMAIN_ID': '0'}
    )
    topics = [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]
    return {"topics": topics}


@router.get("/rosbridge")
async def get_rosbridge_status() -> Dict:
    """
    Check rosbridge availability (detection only, does not start rosbridge).
    Returns status about whether rosbridge is running and accessible.
    """
    port = 9090
    available = False
    error = None
    
    # Check if rosbridge_websocket process is running
    try:
        result = subprocess.run(
            ['pgrep', '-f', 'rosbridge_websocket'],
            capture_output=True,
            text=True,
            timeout=1
        )
        process_running = result.returncode == 0
    except (subprocess.TimeoutExpired, FileNotFoundError):
        process_running = False
    
    # Check if WebSocket port is accessible
    port_accessible = False
    if process_running:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1)
            result = sock.connect_ex(('localhost', port))
            sock.close()
            port_accessible = (result == 0)
        except Exception:
            port_accessible = False
    
    if process_running and port_accessible:
        available = True
    elif process_running and not port_accessible:
        error = "rosbridge process running but port 9090 not accessible"
    elif not process_running:
        error = "rosbridge not running. Start manually: cd ~/dev/r2d2/web_dashboard && ./start_rosbridge.sh"
    
    return {
        "available": available,
        "port": port,
        "process_running": process_running,
        "port_accessible": port_accessible,
        "error": error
    }


