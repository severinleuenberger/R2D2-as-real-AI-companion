"""
System Health API - On-demand metrics collection

This endpoint collects system metrics (CPU, GPU, Disk, Temperature) only when 
requested, saving resources when the dashboard isn't being viewed.
"""

from fastapi import APIRouter
import subprocess
import re
import os
import shutil

router = APIRouter(prefix="/api/system", tags=["system"])


def get_system_metrics() -> dict:
    """
    Collect system metrics from Jetson tegrastats and thermal zones.
    This runs on-demand only when the API is called.
    """
    metrics = {
        'cpu_percent': 0.0,
        'gpu_percent': 0.0,
        'temperature_c': 0.0,
        'disk_percent': 0.0
    }
    
    # Get disk usage for root partition
    try:
        disk_usage = shutil.disk_usage('/')
        metrics['disk_percent'] = round((disk_usage.used / disk_usage.total) * 100.0, 1)
    except Exception:
        pass
    
    # Try to get metrics from tegrastats (Jetson-specific)
    try:
        process = subprocess.Popen(
            ['sh', '-c', 'timeout 1.5 tegrastats --interval 1000 2>&1 | head -1'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        try:
            stdout, stderr = process.communicate(timeout=2.0)
            if stdout:
                output = stdout.strip()
                
                # Parse CPU usage: CPU [19%@729,28%@729,...]
                cpu_match = re.search(r'CPU \[([^\]]+)\]', output)
                if cpu_match:
                    cpu_str = cpu_match.group(1)
                    cpu_percentages = re.findall(r'(\d+)%@', cpu_str)
                    if cpu_percentages:
                        cpu_values = [float(p) for p in cpu_percentages]
                        metrics['cpu_percent'] = round(sum(cpu_values) / len(cpu_values), 1)
                
                # Parse GPU usage: GR3D_FREQ 0%
                gpu_match = re.search(r'GR3D_FREQ\s+(\d+)%', output)
                if gpu_match:
                    metrics['gpu_percent'] = float(gpu_match.group(1))
                
                # Parse CPU temperature: cpu@41.562C
                temp_match = re.search(r'cpu@([\d.]+)C', output)
                if temp_match:
                    metrics['temperature_c'] = round(float(temp_match.group(1)), 1)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate()
        except Exception:
            pass
                
    except FileNotFoundError:
        pass
    except Exception:
        pass
    
    # Fallback to thermal zones for temperature if not set
    if metrics['temperature_c'] == 0.0:
        try:
            thermal_zones = ['/sys/class/thermal/thermal_zone0/temp',
                           '/sys/class/thermal/thermal_zone1/temp']
            for zone in thermal_zones:
                if os.path.exists(zone):
                    with open(zone, 'r') as f:
                        temp_millidegrees = int(f.read().strip())
                        temp_c = temp_millidegrees / 1000.0
                        if temp_c > 0:
                            metrics['temperature_c'] = round(temp_c, 1)
                            break
        except Exception:
            pass
    
    return metrics


@router.get("/health")
async def get_system_health():
    """
    Get system health metrics on-demand.
    
    Returns CPU, GPU, Disk usage and Temperature.
    This endpoint collects metrics only when called, saving resources
    when the dashboard isn't being viewed.
    """
    metrics = get_system_metrics()
    return {
        "cpu_percent": metrics['cpu_percent'],
        "gpu_percent": metrics['gpu_percent'],
        "disk_percent": metrics['disk_percent'],
        "temperature_c": metrics['temperature_c']
    }

