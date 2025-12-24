#!/usr/bin/env python3
"""
Compare current Tailscale/SSH state with expected state from golden branch.
Checks for system configuration changes that might affect connectivity.
"""
import subprocess
import json
import time
import os

LOG_PATH = "/home/severin/.cursor/debug.log"
SESSION_ID = "debug-session"
RUN_ID = "comparison-run"

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
log("COMPARE", "debug_tailscale_comparison.py:start", "Comparison script started", {"pid": os.getpid()})
# #endregion

# Check if WireGuard cleanup was done
wireguard_installed = False
wireguard_config_exists = False
try:
    result = subprocess.run(["dpkg", "-l"], capture_output=True, text=True, timeout=10)
    wireguard_installed = "wireguard" in result.stdout.lower()
    # #region agent log
    log("F", "debug_tailscale_comparison.py:wireguard_check", "WireGuard package check", {
        "installed": wireguard_installed,
        "output": result.stdout[:500] if wireguard_installed else "No wireguard packages"
    })
    # #endregion
except Exception as e:
    # #region agent log
    log("F", "debug_tailscale_comparison.py:wireguard_check", "WireGuard check failed", {"error": str(e)})
    # #endregion

try:
    result = subprocess.run(["test", "-d", "/etc/wireguard"], capture_output=True, timeout=5)
    wireguard_config_exists = result.returncode == 0
    # #region agent log
    log("F", "debug_tailscale_comparison.py:wireguard_config", "WireGuard config directory check", {
        "exists": wireguard_config_exists
    })
    # #endregion
except Exception as e:
    # #region agent log
    log("F", "debug_tailscale_comparison.py:wireguard_config", "WireGuard config check failed", {"error": str(e)})
    # #endregion

# Check IP forwarding (may have been removed during cleanup)
ip_forward_enabled = False
try:
    result = subprocess.run(["sysctl", "-n", "net.ipv4.ip_forward"], capture_output=True, text=True, timeout=5)
    ip_forward_enabled = result.stdout.strip() == "1"
    # #region agent log
    log("G", "debug_tailscale_comparison.py:ip_forward", "IP forwarding check", {
        "enabled": ip_forward_enabled,
        "value": result.stdout.strip()
    })
    # #endregion
except Exception as e:
    # #region agent log
    log("G", "debug_tailscale_comparison.py:ip_forward", "IP forwarding check failed", {"error": str(e)})
    # #endregion

# Check Tailscale service status
tailscaled_active = False
tailscaled_enabled = False
try:
    result = subprocess.run(["systemctl", "is-active", "tailscaled"], capture_output=True, text=True, timeout=5)
    tailscaled_active = result.stdout.strip() == "active"
    # #region agent log
    log("H", "debug_tailscale_comparison.py:tailscale_active", "Tailscale active check", {
        "active": tailscaled_active,
        "output": result.stdout.strip()
    })
    # #endregion
except Exception as e:
    # #region agent log
    log("H", "debug_tailscale_comparison.py:tailscale_active", "Tailscale active check failed", {"error": str(e)})
    # #endregion

try:
    result = subprocess.run(["systemctl", "is-enabled", "tailscaled"], capture_output=True, text=True, timeout=5)
    tailscaled_enabled = result.stdout.strip() in ["enabled", "enabled-runtime"]
    # #region agent log
    log("H", "debug_tailscale_comparison.py:tailscale_enabled", "Tailscale enabled check", {
        "enabled": tailscaled_enabled,
        "output": result.stdout.strip()
    })
    # #endregion
except Exception as e:
    # #region agent log
    log("H", "debug_tailscale_comparison.py:tailscale_enabled", "Tailscale enabled check failed", {"error": str(e)})
    # #endregion

# Check if cleanup script exists (indicates cleanup was planned/done)
cleanup_script_exists = os.path.exists("/home/severin/dev/r2d2/vpn_config/remove_wireguard.sh")
# #region agent log
log("F", "debug_tailscale_comparison.py:cleanup_script", "Cleanup script check", {
    "exists": cleanup_script_exists,
    "path": "/home/severin/dev/r2d2/vpn_config/remove_wireguard.sh"
})
# #endregion

# #region agent log
log("ALL", "debug_tailscale_comparison.py:end", "Comparison completed", {
    "wireguard_installed": wireguard_installed,
    "wireguard_config_exists": wireguard_config_exists,
    "ip_forward_enabled": ip_forward_enabled,
    "tailscaled_active": tailscaled_active,
    "tailscaled_enabled": tailscaled_enabled,
    "cleanup_script_exists": cleanup_script_exists
})
# #endregion

print("Comparison complete. Check logs at:", LOG_PATH)

