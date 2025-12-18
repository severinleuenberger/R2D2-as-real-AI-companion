# Regular Log Files - R2D2 System

**Last Updated:** December 17, 2025  
**Purpose:** Overview of all logging systems and where to find logs for troubleshooting

---

## System Logging Overview

The R2D2 system uses multiple logging mechanisms for different purposes. This document serves as a central reference for all logging locations and tools.

---

## 1. Freeze Monitor Logs

**Purpose:** Diagnose periodic system freezes (every 30-60 minutes)  
**Status:** ✅ Active since December 16, 2025  
**Documentation:** See `050_FREEZE_MONITOR_SYSTEM.md`

### Location
```
/var/log/freeze_logs/
```

### Log Files
| File | Content | Check First |
|------|---------|-------------|
| `summary.log` | Quick overview with timestamps, disk warnings | ✅ Yes |
| `hardware.log` | Tegrastats, temperatures, power | After summary |
| `kernel.log` | Kernel messages and errors | If hardware issues |
| `system.log` | Memory, CPU, I/O, uptime | For resource issues |
| `processes.log` | Top processes by CPU/memory | For runaway processes |

### Audio Warnings
- **Threshold:** ≥ 92% disk usage
- **Sound:** R2-D2 warning (`/usr/local/share/r2d2/sounds/disk_warning.mp3`)
- **Frequency:** Once per hour (throttled)
- **Log Entry:** `⚠️ DISK SPACE WARNING: XX% used!` in summary.log

### Retention
- **Rotation:** Daily or at 100MB per file
- **Retention:** 3 days
- **Compression:** Yes (older logs compressed)
- **Total Disk Usage:** ~150-300MB

### Quick Commands

**View live summary:**
```bash
tail -f /var/log/freeze_logs/summary.log
```

**After a freeze or disk warning:**
```bash
tail -100 /var/log/freeze_logs/summary.log  # Check for warnings
tail -50 /var/log/freeze_logs/hardware.log
tail -50 /var/log/freeze_logs/kernel.log
grep "DISK SPACE WARNING" /var/log/freeze_logs/summary.log  # Find all disk warnings
```

**Check service status:**
```bash
systemctl status freeze-monitor
journalctl -u freeze-monitor -f
```

**Disk usage:**
```bash
du -sh /var/log/freeze_logs/
ls -lh /var/log/freeze_logs/
```

---

## 2. System Logs (Ubuntu/Linux)

**Location:** `/var/log/`

### Key System Log Files

| File | Purpose | View Command |
|------|---------|--------------|
| `/var/log/syslog` | General system messages | `tail -f /var/log/syslog` |
| `/var/log/kern.log` | Kernel messages | `grep -i error /var/log/kern.log` |
| `/var/log/auth.log` | Authentication logs (SSH, sudo) | `tail -f /var/log/auth.log` |
| `/var/log/dmesg` | Boot messages | `dmesg -T \| tail -50` |
| `/var/log/apt/` | Package management logs | `ls -lh /var/log/apt/` |

### Journal (systemd logs)

**View all logs:**
```bash
journalctl -n 100
```

**Follow live:**
```bash
journalctl -f
```

**For specific service:**
```bash
journalctl -u SERVICE_NAME -f
```

**Boot logs:**
```bash
journalctl -b 0        # Current boot
journalctl -b -1       # Previous boot
```

**Time range:**
```bash
journalctl --since "1 hour ago"
journalctl --since "2025-12-16 20:00" --until "2025-12-16 21:00"
```

---

## 3. ROS2 Service Logs

**Location:** Systemd journal (via journalctl)

### R2D2 Services

| Service | Purpose | Log Command |
|---------|---------|-------------|
| `r2d2-camera-stream` | Camera streaming | `journalctl -u r2d2-camera-stream -f` |
| `r2d2-camera-perception` | Person detection | `journalctl -u r2d2-camera-perception -f` |
| `r2d2-rosbridge` | ROS bridge for web UI | `journalctl -u r2d2-rosbridge -f` |
| `r2d2-web-dashboard` | Web dashboard | `journalctl -u r2d2-web-dashboard -f` |
| `r2d2-heartbeat` | System heartbeat | `journalctl -u r2d2-heartbeat -f` |
| `r2d2-powerbutton` | Power button handler | `journalctl -u r2d2-powerbutton -f` |
| `r2d2-audio-notification` | Audio notifications | `journalctl -u r2d2-audio-notification -f` |
| `freeze-monitor` | Freeze diagnostic monitor | `journalctl -u freeze-monitor -f` |

**Check all R2D2 services:**
```bash
systemctl status 'r2d2-*' --no-pager
journalctl -u 'r2d2-*' -f
```

---

## 4. Application-Specific Logs

### Speech System
**Location:** Project directories, depends on implementation

```bash
# Example locations
/home/severin/dev/r2d2/r2d2_speech/logs/
/home/severin/dev/r2d2/log/
```

### Camera/Perception
**Logs:** Via ROS2 services (see above)

### Web Dashboard
**Logs:** Via systemd journal
```bash
journalctl -u r2d2-web-dashboard -n 100
```

---

## 5. Hardware-Specific Logs

### Jetson-Specific Tools

**tegrastats** (real-time monitoring):
```bash
tegrastats --interval 1000
```

**Check temperatures:**
```bash
cat /sys/class/thermal/thermal_zone*/temp
cat /sys/class/thermal/thermal_zone*/type
```

**GPU info:**
```bash
nvidia-smi
```

**Memory info:**
```bash
free -h
cat /proc/meminfo
```

**CPU info:**
```bash
cat /proc/cpuinfo
cat /proc/loadavg
```

---

## 6. Log Rotation Configuration

**Location:** `/etc/logrotate.d/`

### Current Configurations

| Config File | Manages | Retention |
|-------------|---------|-----------|
| `/etc/logrotate.d/freeze-monitor` | Freeze monitor logs | 3 days |
| `/etc/logrotate.d/rsyslog` | System logs | ~4 weeks |
| `/etc/logrotate.d/apt` | APT logs | ~1 year |

**Test logrotate:**
```bash
sudo logrotate -d /etc/logrotate.d/freeze-monitor  # Dry run
sudo logrotate -f /etc/logrotate.d/freeze-monitor  # Force rotation
```

---

## 7. Disk Space Management

### Check Disk Usage

**Overall:**
```bash
df -h
```

**Log directories:**
```bash
du -sh /var/log/
du -h /var/log/ | sort -rh | head -20
```

**Freeze monitor logs:**
```bash
du -sh /var/log/freeze_logs/
ls -lh /var/log/freeze_logs/
```

### Current Status (Dec 17, 2025)
- **Total Disk:** 57GB
- **Used:** 50GB (93%)
- **Available:** 4.1GB
- **Freeze Logs:** ~150-300MB (3 days retention)

### Cleanup If Needed

**Clear old journals:**
```bash
sudo journalctl --vacuum-time=7d    # Keep 7 days
sudo journalctl --vacuum-size=500M  # Keep 500MB max
```

**Check journal size:**
```bash
journalctl --disk-usage
```

**Clear APT cache:**
```bash
sudo apt clean
sudo apt autoclean
```

**Find large files:**
```bash
sudo du -ah /var/log/ | sort -rh | head -20
```

---

## 8. Troubleshooting Workflows

### System Freeze Investigation

1. **Check freeze monitor logs:**
   ```bash
   tail -100 /var/log/freeze_logs/summary.log
   tail -50 /var/log/freeze_logs/hardware.log
   tail -50 /var/log/freeze_logs/kernel.log
   ```

2. **Check for OOM (Out of Memory):**
   ```bash
   grep -i "out of memory" /var/log/freeze_logs/kernel.log
   grep -i "oom" /var/log/kern.log
   ```

3. **Check thermal issues:**
   ```bash
   grep -i "thermal\|overheat" /var/log/freeze_logs/kernel.log
   ```

4. **Check hardware errors:**
   ```bash
   grep -i "error\|fail" /var/log/freeze_logs/hardware.log
   ```

### Service Not Working

1. **Check service status:**
   ```bash
   systemctl status SERVICE_NAME
   ```

2. **View recent logs:**
   ```bash
   journalctl -u SERVICE_NAME -n 100
   ```

3. **Follow live logs:**
   ```bash
   journalctl -u SERVICE_NAME -f
   ```

4. **Check errors only:**
   ```bash
   journalctl -u SERVICE_NAME -p err
   ```

### Performance Issues

1. **Check CPU/Memory:**
   ```bash
   htop
   top
   ```

2. **Check load average:**
   ```bash
   uptime
   cat /proc/loadavg
   ```

3. **Check temperatures:**
   ```bash
   tail -20 /var/log/freeze_logs/hardware.log
   ```

4. **Check disk I/O:**
   ```bash
   iostat -x 1
   ```

### Network Issues

1. **Check network interfaces:**
   ```bash
   ip addr
   ifconfig
   ```

2. **Check SSH logs:**
   ```bash
   grep -i "ssh" /var/log/auth.log | tail -50
   ```

3. **Check network errors:**
   ```bash
   dmesg | grep -i "network\|eth\|wlan"
   ```

---

## 9. Log Monitoring Best Practices

### Regular Checks (Weekly)

```bash
# Check disk space
df -h /var/log

# Verify freeze monitor is running
systemctl status freeze-monitor

# Check for service failures
systemctl --failed

# Review recent errors
journalctl -p err --since "7 days ago" | tail -100
```

### After System Issues

```bash
# Backup logs before they rotate
sudo mkdir -p /home/severin/log_backup/$(date +%Y%m%d)
sudo cp -r /var/log/freeze_logs/* /home/severin/log_backup/$(date +%Y%m%d)/

# Check what happened
tail -200 /home/severin/log_backup/$(date +%Y%m%d)/summary.log
```

### Before Updates/Changes

```bash
# Save current system state
sudo journalctl -b 0 > /home/severin/system_state_$(date +%Y%m%d_%H%M%S).log
```

---

## 10. Quick Reference Commands

### Most Useful Commands

```bash
# Freeze monitor status
systemctl status freeze-monitor
tail -f /var/log/freeze_logs/summary.log

# All R2D2 services
systemctl status 'r2d2-*'

# Recent system errors
journalctl -p err --since "1 hour ago"

# Disk space
df -h
du -sh /var/log/

# Live system monitoring
htop
tegrastats --interval 1000

# Check temperatures
cat /sys/class/thermal/thermal_zone*/temp

# Memory usage
free -h

# Last boot messages
journalctl -b 0 -p err

# SSH login attempts
grep "sshd" /var/log/auth.log | tail -20
```

---

## 11. Log Files Summary Table

| Log Type | Location | Retention | Size | Purpose |
|----------|----------|-----------|------|---------|
| Freeze Monitor | `/var/log/freeze_logs/` | 3 days | ~150-300MB | Diagnose system freezes |
| System Log | `/var/log/syslog` | ~4 weeks | Varies | General system messages |
| Kernel Log | `/var/log/kern.log` | ~4 weeks | Varies | Kernel messages |
| Auth Log | `/var/log/auth.log` | ~4 weeks | Varies | Authentication events |
| Journal | `/var/log/journal/` | Persistent | Varies | systemd logs |
| APT Logs | `/var/log/apt/` | ~1 year | Small | Package management |
| DMesg | `/var/log/dmesg` | Current boot | Small | Boot messages |

---

## 12. Related Documentation

- **`050_FREEZE_MONITOR_SYSTEM.md`** - Detailed freeze monitor documentation
- **`003_JETSON_FLASHING_AND_DISPLAY_SETUP.md`** - System setup and configuration
- **`000_INTERNAL_AGENT_NOTES.md`** - Development and maintenance notes
- **ROS2 Service Documentation** - Individual service setup guides

---

## Notes

- **Disk Space Critical:** At 93% usage, monitor disk space regularly
- **Log Rotation:** Automatic for most logs, configured via logrotate
- **Journal Size:** Can grow large, clean periodically with `journalctl --vacuum-*`
- **Freeze Monitor:** Specifically designed for the periodic freeze issue
- **Real-time Monitoring:** Use `tegrastats`, `htop`, or live log tailing

---

**For issues or questions about logging, refer to this document first.**


