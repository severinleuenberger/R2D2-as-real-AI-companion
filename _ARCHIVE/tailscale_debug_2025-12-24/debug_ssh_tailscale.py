#!/usr/bin/env python3
"""
Diagnostic script for SSH over Tailscale connectivity issues.
Tests multiple hypotheses about why the connection fails.
"""
import subprocess
import json
import time
import socket
import os

LOG_PATH = "/home/severin/.cursor/debug.log"
SESSION_ID = "debug-session"
RUN_ID = "run1"

def log(hypothesis_id, location, message, data):
    """Write debug log entry in NDJSON format"""
    entry = {
        "id": f"log_{int(time.time())}_{hypothesis_id}",
        "timestamp": int(time.time() * 1000),
        "location": location,
        "message": message,
        "data": data,
        "sessionId": SESSION_ID,
        "runId": RUN_ID,
        "hypothesisId": hypothesis_id
    }
    with open(LOG_PATH, "a") as f:
        f.write(json.dumps(entry) + "\n")

# #region agent log
log("A", "debug_ssh_tailscale.py:start", "Diagnostic script started", {"pid": os.getpid()})
# #endregion

# Hypothesis A: Tailscale service (tailscaled) is not running or has crashed
try:
    result = subprocess.run(["pgrep", "-f", "tailscaled"], capture_output=True, text=True, timeout=5)
    tailscaled_running = len(result.stdout.strip()) > 0
    tailscaled_pids = result.stdout.strip().split("\n") if result.stdout.strip() else []
    # #region agent log
    log("A", "debug_ssh_tailscale.py:hypothesis_a", "Tailscale process check", {
        "running": tailscaled_running,
        "pids": tailscaled_pids,
        "exit_code": result.returncode
    })
    # #endregion
except Exception as e:
    # #region agent log
    log("A", "debug_ssh_tailscale.py:hypothesis_a", "Tailscale process check failed", {"error": str(e)})
    # #endregion
    tailscaled_running = False

# Hypothesis B: SSH service is not running or not listening on port 22
try:
    # Check if port 22 is listening
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(1)
    port_listening = sock.connect_ex(("127.0.0.1", 22)) == 0
    sock.close()
    # #region agent log
    log("B", "debug_ssh_tailscale.py:hypothesis_b", "SSH port 22 check", {
        "listening": port_listening,
        "port": 22
    })
    # #endregion
except Exception as e:
    # #region agent log
    log("B", "debug_ssh_tailscale.py:hypothesis_b", "SSH port check failed", {"error": str(e)})
    # #endregion
    port_listening = False

# Also check sshd process
try:
    result = subprocess.run(["pgrep", "-f", "sshd"], capture_output=True, text=True, timeout=5)
    sshd_running = len(result.stdout.strip()) > 0
    sshd_pids = result.stdout.strip().split("\n") if result.stdout.strip() else []
    # #region agent log
    log("B", "debug_ssh_tailscale.py:hypothesis_b", "SSH daemon process check", {
        "running": sshd_running,
        "pids": sshd_pids,
        "exit_code": result.returncode
    })
    # #endregion
except Exception as e:
    # #region agent log
    log("B", "debug_ssh_tailscale.py:hypothesis_b", "SSH daemon check failed", {"error": str(e)})
    # #endregion
    sshd_running = False

# Hypothesis C: Windows device is disconnected from Tailscale network
try:
    result = subprocess.run(["tailscale", "status"], capture_output=True, text=True, timeout=10)
    status_output = result.stdout
    status_lines = status_output.split("\n")
    windows_device_found = False
    windows_device_status = None
    for line in status_lines:
        if "itxcl883" in line or "100.100.52.23" in line:
            windows_device_found = True
            windows_device_status = line.strip()
            break
    # #region agent log
    log("C", "debug_ssh_tailscale.py:hypothesis_c", "Tailscale status check", {
        "status_output": status_output,
        "windows_device_found": windows_device_found,
        "windows_device_status": windows_device_status,
        "exit_code": result.returncode
    })
    # #endregion
except Exception as e:
    # #region agent log
    log("C", "debug_ssh_tailscale.py:hypothesis_c", "Tailscale status check failed", {"error": str(e)})
    # #endregion
    windows_device_found = False

# Hypothesis D: Tailscale IP address changed or connectivity issue
try:
    result = subprocess.run(["tailscale", "ip", "-4"], capture_output=True, text=True, timeout=10)
    current_ip = result.stdout.strip()
    expected_ip = "100.95.133.26"
    ip_matches = current_ip == expected_ip
    # #region agent log
    log("D", "debug_ssh_tailscale.py:hypothesis_d", "Tailscale IP check", {
        "current_ip": current_ip,
        "expected_ip": expected_ip,
        "ip_matches": ip_matches,
        "exit_code": result.returncode
    })
    # #endregion
except Exception as e:
    # #region agent log
    log("D", "debug_ssh_tailscale.py:hypothesis_d", "Tailscale IP check failed", {"error": str(e)})
    # #endregion
    current_ip = None

# Hypothesis E: DNS/connectivity issue affecting Tailscale (health check warning)
try:
    result = subprocess.run(["tailscale", "status"], capture_output=True, text=True, timeout=10)
    status_output = result.stdout
    has_dns_warning = "DNS" in status_output or "dns" in status_output.lower()
    has_health_warning = "Health check" in status_output or "health" in status_output.lower()
    # #region agent log
    log("E", "debug_ssh_tailscale.py:hypothesis_e", "Tailscale health/DNS check", {
        "has_dns_warning": has_dns_warning,
        "has_health_warning": has_health_warning,
        "status_snippet": status_output[-500:] if len(status_output) > 500 else status_output,
        "exit_code": result.returncode
    })
    # #endregion
except Exception as e:
    # #region agent log
    log("E", "debug_ssh_tailscale.py:hypothesis_e", "Tailscale health check failed", {"error": str(e)})
    # #endregion
    has_dns_warning = False

# Test connectivity to Windows device
try:
    result = subprocess.run(["timeout", "3", "tailscale", "ping", "-c", "1", "100.100.52.23"], 
                          capture_output=True, text=True, timeout=5)
    ping_success = result.returncode == 0
    ping_output = result.stdout + result.stderr
    # #region agent log
    log("C", "debug_ssh_tailscale.py:hypothesis_c", "Ping test to Windows device", {
        "ping_success": ping_success,
        "ping_output": ping_output,
        "exit_code": result.returncode
    })
    # #endregion
except Exception as e:
    # #region agent log
    log("C", "debug_ssh_tailscale.py:hypothesis_c", "Ping test failed", {"error": str(e)})
    # #endregion
    ping_success = False

# #region agent log
log("ALL", "debug_ssh_tailscale.py:end", "Diagnostic script completed", {
    "tailscaled_running": tailscaled_running,
    "port_listening": port_listening,
    "sshd_running": sshd_running,
    "windows_device_found": windows_device_found,
    "ping_success": ping_success,
    "current_ip": current_ip,
    "has_dns_warning": has_dns_warning
})
# #endregion

print("Diagnostic complete. Check logs at:", LOG_PATH)

