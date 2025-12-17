# Jetson Freeze Diagnostic Monitor

**Created:** December 16, 2025  
**Status:** ✅ Active and Running  
**Purpose:** Diagnose and analyze periodic Jetson freezes (every 30-60 minutes)

---

## Problem Statement

The Jetson Orin Nano experiences complete system freezes approximately every 30-60 minutes:
- System becomes unresponsive
- Power button doesn't work (can't shutdown)
- SSH connections (both WLAN and USB) become unavailable
- Requires hard reboot
- No logs available after reboot to diagnose the issue

## Solution

Comprehensive real-time monitoring system that logs hardware metrics, kernel messages, system resources, and process information to persistent storage. After a freeze and reboot, logs reveal the system state leading up to the freeze.

---

## Architecture

### Components

1. **freeze_monitor.py** - Python monitoring script
   - Location: `/home/severin/freeze_monitor.py`
   - Logs every 10 seconds
   - Monitors: tegrastats, thermal zones, memory, CPU, kernel messages, processes

2. **freeze-monitor.service** - Systemd service
   - Location: `/etc/systemd/system/freeze-monitor.service`
   - Runs as root for hardware access
   - Auto-starts on boot
   - Resource-limited (10% CPU, 200MB RAM)

3. **Log Files** - Structured logging to `/var/log/freeze_logs/`
   - `summary.log` - Quick overview (check this first)
   - `hardware.log` - Tegrastats, temperatures, power
   - `kernel.log` - Kernel messages and errors
   - `system.log` - Memory, CPU, I/O, uptime
   - `processes.log` - Top processes by CPU

4. **Log Rotation** - Automatic cleanup
   - Location: `/etc/logrotate.d/freeze-monitor`
   - Rotates daily or at 100MB
   - Keeps 14 days of logs
   - Compresses old logs

---

## Installation

The system is already installed and running. To reinstall:

```bash
cd /home/severin/dev/r2d2/system_monitoring
sudo bash install_freeze_monitor.sh
```

---

## Usage

### Check Service Status

```bash
systemctl status freeze-monitor
```

### View Live Logs

```bash
# Summary log (most useful)
tail -f /var/log/freeze_logs/summary.log

# Service logs
journalctl -u freeze-monitor -f

# All log files
ls -lh /var/log/freeze_logs/
```

### After a Freeze

After the system freezes and reboots, analyze the logs:

```bash
# Quick overview - last 100 entries
tail -100 /var/log/freeze_logs/summary.log

# Hardware details
tail -50 /var/log/freeze_logs/hardware.log

# Kernel errors
tail -50 /var/log/freeze_logs/kernel.log

# System resources
tail -50 /var/log/freeze_logs/system.log

# Process activity
tail -50 /var/log/freeze_logs/processes.log
```

### Service Management

```bash
# Stop monitoring
sudo systemctl stop freeze-monitor

# Restart monitoring
sudo systemctl restart freeze-monitor

# Disable auto-start
sudo systemctl disable freeze-monitor

# Re-enable auto-start
sudo systemctl enable freeze-monitor
```

---

## What to Look For

### Memory Issues
- Memory usage reaching 95%+
- Swap thrashing (high swap usage)
- OOM (Out of Memory) killer messages in kernel log

### Thermal Issues
- Temperatures above 85°C
- Thermal throttling warnings
- Sudden temperature spikes

### GPU/Hardware Issues
- NVIDIA GPU errors
- PCIe errors
- Hardware driver crashes

### Process Issues
- Runaway processes consuming excessive resources
- Memory leaks (steadily growing memory usage)
- Deadlocks or hung processes

### I/O Issues
- Disk errors
- High I/O wait times
- File system errors

---

## Log Format

### Summary Log Example
```
[2025-12-16 21:22:11.327] Cycle 1 | Mem:6% | Load:0.7 | Temp:42.0°C
[2025-12-16 21:22:24.427] Cycle 2 | Mem:6% | Load:0.8 | Temp:42.1°C
[2025-12-16 21:22:37.728] Cycle 3 | Mem:6% | Load:0.8 | Temp:41.8°C
```

### Hardware Log Example
```
[2025-12-16 21:22:11.326] === Hardware Metrics ===
[2025-12-16 21:22:11.326] Tegrastats: RAM 1899/7471MB (lfb 1588x4MB) CPU [2%@730,0%@729,0%@729,0%@729,0%@729,0%@729]
[2025-12-16 21:22:11.326] Thermal: {"BCPU-therm": "42.0°C", "CV0-therm": "41.5°C", "GPU-therm": "41.2°C"}
```

---

## Current Status

**Monitoring Started:** December 16, 2025 at 21:22:07 CET  
**Service Status:** Active and running  
**Log Location:** `/var/log/freeze_logs/`  
**Log Interval:** 10 seconds  
**Auto-start:** Enabled (starts on boot)

**Baseline Metrics:**
- Memory: 6% used (~1.9GB / 7.5GB)
- CPU Load: 0.7-0.8 (1-minute average)
- Temperature: 41-42°C (normal operating temp)

---

## Disk Space Management

**Current Status:** 93% disk usage (50GB / 57GB)

The monitoring system includes safeguards:
- Automatic log rotation (daily or at 100MB per file)
- 14-day retention (older logs deleted automatically)
- Compression of rotated logs
- Disk space warnings when < 1GB free

**Expected Log Growth:** ~50-100MB per day (all logs combined)  
**Total with rotation:** ~700MB-1.4GB (14 days of logs)

---

## Troubleshooting

### Service Won't Start

```bash
# Check status and error messages
systemctl status freeze-monitor
journalctl -u freeze-monitor -n 50
```

### No Logs Being Written

```bash
# Check log directory permissions
ls -la /var/log/freeze_logs/

# Should show: root root with 644 permissions
```

### Adjust Logging Interval

If disk space is a concern, edit the interval in the monitoring script:

```bash
sudo nano /home/severin/freeze_monitor.py
# Find: LOG_INTERVAL = 10
# Change to: LOG_INTERVAL = 30  # or 60 for less frequent logging
sudo systemctl restart freeze-monitor
```

---

## Files Reference

### Source Files (in system_monitoring/)
- `freeze_monitor.py` - Main monitoring script
- `freeze-monitor.service` - Systemd service file
- `freeze-monitor.logrotate` - Logrotate configuration
- `install_freeze_monitor.sh` - Installation script
- `FREEZE_MONITOR_README.md` - User guide

### System Files (installed)
- `/etc/systemd/system/freeze-monitor.service` - Active service
- `/etc/logrotate.d/freeze-monitor` - Active log rotation
- `/var/log/freeze_logs/*.log` - Log files

---

## Analysis Workflow

When a freeze occurs:

1. **After reboot**, immediately copy logs before they rotate:
   ```bash
   cd /home/severin
   mkdir -p freeze_logs_backup/$(date +%Y%m%d_%H%M%S)
   sudo cp -r /var/log/freeze_logs/* freeze_logs_backup/$(date +%Y%m%d_%H%M%S)/
   ```

2. **Analyze the summary** for patterns:
   ```bash
   tail -200 freeze_logs_backup/*/summary.log
   ```

3. **Look for anomalies** in the last 5-10 minutes before freeze:
   - Memory percentage increasing?
   - Temperature rising?
   - Load average spiking?

4. **Check kernel log** for hardware errors:
   ```bash
   grep -i "error\|fail\|thermal\|oom" freeze_logs_backup/*/kernel.log
   ```

5. **Check processes** for resource hogs:
   ```bash
   grep -A 2 "Cycle" freeze_logs_backup/*/processes.log | tail -50
   ```

6. **Document findings** and patterns for troubleshooting

---

## Historical Context

**First Freeze Detected:** December 16, 2025 at ~21:17  
**Previous Boot Duration:** 48 minutes (20:29 - 21:17)  
**Freeze Pattern:** Every 30-60 minutes  
**Symptoms:** Complete system unresponsiveness, no SSH, power button disabled

**Known Issues from Historical Logs:**
- PCIe errors related to WiFi card (rtl88x2ce)
- USB-C controller I2C transfer failures
- Minor NVIDIA GPU warnings (RPC DCE control call failures)

---

## Next Steps

1. **Monitor for next freeze** (expected in 30-60 minutes)
2. **Collect logs** immediately after reboot
3. **Analyze patterns** across multiple freeze events
4. **Identify root cause** from log correlation
5. **Implement fix** based on findings

---

## Maintenance

### Weekly Tasks
- Check disk space: `df -h /var/log`
- Verify service is running: `systemctl status freeze-monitor`

### After Finding Root Cause
Once the freeze issue is resolved, you can:
- Increase logging interval to reduce disk usage
- Disable the service: `sudo systemctl disable freeze-monitor`
- Keep files for future troubleshooting

---

**Related Documentation:**
- `system_monitoring/FREEZE_MONITOR_README.md` - Detailed user guide
- `003_JETSON_FLASHING_AND_DISPLAY_SETUP.md` - System setup
- `000_INTERNAL_AGENT_NOTES.md` - Development context
