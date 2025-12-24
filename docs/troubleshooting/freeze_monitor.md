# Jetson Freeze Monitor - Setup Guide

## Overview

This freeze monitoring system will help diagnose the Jetson freezes you're experiencing every 30+ minutes. It continuously logs hardware metrics, kernel messages, system resources, and process information so you can analyze what happened after a freeze and restart.

## Files Created

1. **freeze_monitor.py** - Main monitoring script
2. **freeze-monitor.service** - Systemd service configuration
3. **freeze-monitor.logrotate** - Log rotation configuration
4. **install_freeze_monitor.sh** - Installation script

## Quick Start

### Installation

Run the installation script with sudo:

```bash
cd /home/severin
sudo bash install_freeze_monitor.sh
```

The script will:
- Create the log directory `/var/log/freeze_logs/`
- Install the systemd service
- Install logrotate configuration
- Enable and start the service
- Show verification output

### Verification

Check that the service is running:

```bash
systemctl status freeze-monitor
```

View live summary logs:

```bash
tail -f /var/log/freeze_logs/summary.log
```

## What Gets Logged

The monitor logs data every **10 seconds** to separate log files:

### 1. **summary.log**
Quick overview with timestamps - check this first after a freeze
- Memory usage percentage
- CPU load average
- Temperature
- Cycle counter

### 2. **hardware.log**
Jetson-specific hardware metrics
- Tegrastats output (CPU, GPU, memory, power)
- Thermal zone temperatures
- Power consumption

### 3. **kernel.log**
Kernel messages and errors
- Recent dmesg output (errors and warnings only)
- Critical system messages

### 4. **system.log**
System resource usage
- Memory (total, available, free)
- Swap usage
- CPU load averages
- Uptime
- Disk I/O stats

### 5. **processes.log**
Process information
- Top 10 processes by CPU usage
- Memory usage per process

## After a Freeze

When the system freezes and you restart it, check the logs:

### 1. Check Summary First

```bash
tail -100 /var/log/freeze_logs/summary.log
```

Look for patterns before the last timestamp (when the freeze occurred):
- Memory suddenly filling up?
- High CPU load?
- Rising temperatures?

### 2. Check Hardware Details

```bash
tail -50 /var/log/freeze_logs/hardware.log
```

Look for:
- Temperature spikes
- GPU throttling
- Power issues
- Memory pressure

### 3. Check Kernel Messages

```bash
tail -50 /var/log/freeze_logs/kernel.log
```

Look for:
- Hardware errors
- Driver crashes
- OOM (Out of Memory) killer messages
- Thermal throttling warnings

### 4. Check Process Activity

```bash
tail -50 /var/log/freeze_logs/processes.log
```

Look for:
- Processes consuming excessive CPU
- Memory leaks (growing memory usage)
- Hung processes

## Log Management

### Automatic Rotation

Logs are automatically rotated by logrotate:
- **Daily** or when files exceed **100MB**
- Keeps **7 days** of logs
- Old logs are compressed

### Manual Cleanup

If you need to clear logs manually:

```bash
sudo rm /var/log/freeze_logs/*.log
sudo systemctl restart freeze-monitor
```

### Disk Space

Your system is at 93% disk usage (50GB / 57GB). The monitor includes:
- Disk space warnings when < 1GB free
- Automatic log rotation to prevent disk fill
- Compressed old logs

Monitor disk usage:

```bash
df -h /var/log
du -sh /var/log/freeze_logs/
```

## Service Management

### View Service Status

```bash
systemctl status freeze-monitor
```

### View Live Service Logs

```bash
journalctl -u freeze-monitor -f
```

### Restart Service

```bash
sudo systemctl restart freeze-monitor
```

### Stop Service

```bash
sudo systemctl stop freeze-monitor
```

### Disable Service (won't start on boot)

```bash
sudo systemctl disable freeze-monitor
```

### Re-enable Service

```bash
sudo systemctl enable freeze-monitor
sudo systemctl start freeze-monitor
```

## Troubleshooting

### Service Won't Start

Check the service status and logs:

```bash
systemctl status freeze-monitor
journalctl -u freeze-monitor -n 50
```

### No Logs Being Written

Check permissions:

```bash
ls -la /var/log/freeze_logs/
```

Should be owned by root with 644 permissions.

### Logs Growing Too Fast

If disk space is a concern, you can:

1. Increase the logging interval (edit freeze_monitor.py, change LOG_INTERVAL from 10 to 30 or 60)
2. Reduce log retention (edit /etc/logrotate.d/freeze-monitor, change rotate from 7 to 3)
3. Restart service after changes

## Expected Log Size

With 10-second intervals:
- ~8,640 log entries per day
- Estimated ~50-100MB per day (all logs combined)
- With rotation: ~350-700MB total (7 days)

## Common Freeze Causes to Look For

Based on the logs, look for these patterns:

1. **Memory Issues**
   - Memory usage reaching 95%+
   - Swap thrashing
   - OOM killer in kernel log

2. **Thermal Issues**
   - Temperatures above 85°C
   - Thermal throttling messages
   - Sudden shutdowns

3. **Power Issues**
   - Power consumption spikes
   - Voltage drops
   - Power supply warnings

4. **GPU Issues**
   - GPU hangs
   - CUDA errors
   - Graphics driver crashes

5. **I/O Issues**
   - Disk errors
   - High I/O wait times
   - File system errors

6. **Process Issues**
   - Runaway processes
   - Deadlocks
   - Resource exhaustion

## Next Steps

1. Let the monitor run for a few days
2. After each freeze, collect the logs before they rotate
3. Look for patterns in the logs before the freeze
4. Share findings if you need help analyzing

## Uninstallation

If you want to remove the monitor:

```bash
sudo systemctl stop freeze-monitor
sudo systemctl disable freeze-monitor
sudo rm /etc/systemd/system/freeze-monitor.service
sudo rm /etc/logrotate.d/freeze-monitor
sudo systemctl daemon-reload
sudo rm -rf /var/log/freeze_logs/
```

## Contact

If you need help analyzing the logs after collecting data from freezes, you can share:
- Last 100 lines of summary.log
- Last 50 lines of hardware.log and kernel.log
- Your observations about when freezes occur

---

**Created:** December 16, 2025
**Log Location:** /var/log/freeze_logs/
**Log Interval:** 10 seconds
**Retention:** 7 days

