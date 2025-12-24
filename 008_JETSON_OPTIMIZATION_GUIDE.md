# Jetson Optimization Guide

## Overview

This guide provides condensed best practices for optimizing the NVIDIA Jetson AGX Orin 64GB for R2D2's AI workloads, including vision, speech processing, and autonomous navigation.

For comprehensive details, see **[Jetson Best Practices](docs/reference/jetson_best_practices.md)**.

---

## Quick Optimization Checklist

### ✅ Essential Settings

- [ ] **Power Mode:** MAXN (for real-time processing)
- [ ] **Thermal Monitoring:** Enabled with freeze monitor
- [ ] **GPU Acceleration:** Container with `--runtime nvidia`
- [ ] **Memory:** Monitor usage, keep <90% utilization
- [ ] **Swap:** Enabled (8-16GB for safety)

---

## 1. Power Management

### Power Modes

```bash
# Check current power mode
sudo nvpmodel -q --verbose

# Set to MAXN (maximum performance)
sudo nvpmodel -m 0

# List available modes
sudo nvpmodel -l
```

**Recommended for R2D2:** MAXN mode (mode 0)
- All CPU cores enabled
- GPU at maximum frequency
- ~25-30W continuous power

### Thermal Management

**Operating Limits:**
- **Optimal Range:** 40-75°C
- **Warning Threshold:** 75°C (throttling begins)
- **Danger Zone:** 80°C+ (performance degradation)
- **Thermal Shutdown:** 95°C (hardware enforced)

**Monitoring:**
```bash
# Real-time thermal & power monitoring
tegrastats

# Check thermal zones
cat /sys/class/thermal/thermal_zone*/temp

# Check throttling status
cat /proc/tegra_throttle_alert
```

**R2D2 Integration:**
- Freeze monitor active (`freeze-monitor.service`)
- Logs thermal events to `/var/log/freeze_monitor.log`
- See: **[docs/troubleshooting/freeze_monitor.md](docs/troubleshooting/freeze_monitor.md)**

---

## 2. Memory Optimization

### Memory Architecture

- **Total Memory:** 64GB LPDDR5 unified memory
- **Shared:** CPU and GPU use same physical RAM
- **Bandwidth:** 204.8 GB/s

### Memory Monitoring

```bash
# Check memory usage
free -h

# Detailed memory stats
cat /proc/meminfo

# Per-process memory usage
ps aux --sort=-%mem | head -10
```

### Best Practices

✅ **Keep memory usage < 90%** (leave headroom for bursts)  
✅ **Enable swap** (8-16GB for safety)  
✅ **Monitor ROS 2 nodes** (some nodes can leak memory)  
✅ **Use GPU memory** for large models (offload from system RAM)  

**R2D2 Typical Usage:**
- Base system: ~2-3GB
- ROS 2 nodes: ~3-4GB
- Speech processing (GPU container): ~4-6GB
- Camera/perception: ~2-3GB
- **Total:** ~12-16GB (comfortable margin on 64GB)

---

## 3. GPU Acceleration

### CUDA Configuration

```bash
# Verify CUDA installation
nvcc --version

# Check GPU status
tegrastats | grep GPU

# Monitor GPU memory usage
sudo jetson_stats
```

### Container Configuration

R2D2 uses official NVIDIA containers for GPU acceleration:

```bash
# Test GPU in container
sudo docker run --rm --runtime nvidia \
  dustynv/l4t-pytorch:r36.4.0 \
  python3 -c "import torch; print(f'CUDA: {torch.cuda.is_available()}')"
```

**See:** **[007_GPU_ACCELERATION_REFERENCE.md](007_GPU_ACCELERATION_REFERENCE.md)**

---

## 4. ROS 2 Integration Best Practices

### Performance Settings

**DDS Configuration (CycloneDDS):**
```bash
# Already configured in ~/.bashrc
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/severin/dev/r2d2/cyclonedds.xml
```

**ROS 2 QoS Profiles:**
- Use `BEST_EFFORT` for high-frequency sensor data (camera, IMU)
- Use `RELIABLE` for commands and state changes

### Node Optimization

```bash
# Monitor ROS 2 node CPU usage
ros2 wtf

# Check topic frequencies
ros2 topic hz /oak/rgb/image_raw

# Monitor node memory
ros2 node info /camera_node
```

---

## 5. Storage & I/O

### NVMe SSD Configuration

R2D2 uses NVMe SSD for fast storage:

```bash
# Check disk usage
df -h

# Monitor I/O performance
iostat -x 5

# Check SSD health
sudo smartctl -a /dev/nvme0n1
```

### Best Practices

✅ **Keep root filesystem < 80% full**  
✅ **Use separate partition for logs** (or log rotation)  
✅ **Enable TRIM** for SSD longevity  
✅ **Regular backups** (see 004_BACKUP_AND_RESTORE.md)  

---

## 6. Network Optimization

### Tailscale VPN

```bash
# Check Tailscale status
tailscale status

# Verify connection quality
tailscale ping <device-name>

# Monitor Tailscale logs
journalctl -u tailscaled -f
```

**See:** **[docs/troubleshooting/tailscale_vpn.md](docs/troubleshooting/tailscale_vpn.md)**

### ROS 2 Network

- CycloneDDS configured for low latency
- No network discovery needed (single machine)
- Tailscale for remote monitoring only

---

## 7. Common Issues & Solutions

### High Temperature

**Symptoms:** Throttling, performance degradation

**Solutions:**
1. Check fan is running: `grep -i fan /var/log/syslog`
2. Improve airflow (ensure 2cm clearance)
3. Reduce power mode: `sudo nvpmodel -m 1` (15W mode)
4. Check thermal paste on heatsink

### Memory Pressure

**Symptoms:** Slow performance, OOM killer

**Solutions:**
1. Enable swap: `sudo swapon -a`
2. Restart memory-hungry nodes
3. Check for memory leaks: `watch -n 1 free -h`

### GPU Not Accessible

**Symptoms:** `CUDA: False` in container

**Solutions:**
1. Ensure `--runtime nvidia` flag: `docker run --runtime nvidia ...`
2. Verify NVIDIA runtime: `docker info | grep -i nvidia`
3. Restart Docker daemon: `sudo systemctl restart docker`

**See:** **[007_GPU_ACCELERATION_REFERENCE.md](007_GPU_ACCELERATION_REFERENCE.md#troubleshooting)**

---

## 8. Monitoring Tools

### Essential Commands

```bash
# Comprehensive system stats (thermal, power, CPU, GPU, memory)
tegrastats

# Interactive system monitor (highly recommended)
sudo jtop

# GPU-specific monitoring
nvidia-smi  # (limited on Jetson, use tegrastats instead)

# ROS 2 system health
ros2 doctor --report

# Check all systemd services
systemctl status r2d2-*.service
```

### R2D2 Monitoring Script

```bash
# Minimalistic one-line monitor
cd /home/severin/dev/r2d2/tools
python3 minimal_monitor.py
```

**See:** **[006_SYSTEM_STATUS_AND_MONITORING.md](006_SYSTEM_STATUS_AND_MONITORING.md)**

---

## 9. Recommended Regular Maintenance

### Daily
- Monitor thermal status (freeze monitor does this automatically)
- Check ROS 2 service status

### Weekly
- Review freeze monitor logs
- Check disk space
- Verify Tailscale connectivity

### Monthly
- System updates: `sudo apt update && sudo apt upgrade`
- Check SSD health: `sudo smartctl -a /dev/nvme0n1`
- Review ROS 2 log sizes: `du -sh ~/.ros/log`
- Backup system: See **[004_BACKUP_AND_RESTORE.md](004_BACKUP_AND_RESTORE.md)**

---

## 10. Performance Expectations

### R2D2 Benchmarks

| Component | Performance | Notes |
|-----------|-------------|-------|
| **Camera (OAK-D)** | 30 FPS @ 1920×1080 | RGB stream |
| **Face Detection** | 13 Hz | Haar Cascade |
| **Face Recognition** | 6.5 Hz | LBPH |
| **Speech STT (GPU)** | 1-2s per 10s audio | faster-whisper |
| **Speech TTS (GPU)** | 0.5-1s per response | piper-tts |
| **System CPU Usage** | 10-15% idle | With all services |
| **Temperature** | 55-65°C | MAXN mode, ambient 20°C |

### Optimization Targets

✅ **CPU usage:** < 60% average (leave headroom)  
✅ **Memory usage:** < 50% average (~32GB used)  
✅ **Temperature:** < 75°C continuous  
✅ **Response latency:** < 3s total (speech to speech)  

---

## Detailed Reference

For comprehensive documentation including:
- Advanced power management
- Container deployment strategies
- CUDA profiling and optimization
- ROS 2 performance tuning
- Industrial-grade thermal solutions
- Memory carveout configuration
- Custom kernel parameters

See: **[docs/reference/jetson_best_practices.md](docs/reference/jetson_best_practices.md)**

---

## Related Documentation

- **[001_ARCHITECTURE_OVERVIEW.md](001_ARCHITECTURE_OVERVIEW.md)** - System architecture
- **[007_GPU_ACCELERATION_REFERENCE.md](007_GPU_ACCELERATION_REFERENCE.md)** - GPU setup
- **[005_SYSTEMD_SERVICES_REFERENCE.md](005_SYSTEMD_SERVICES_REFERENCE.md)** - Service management
- **[006_SYSTEM_STATUS_AND_MONITORING.md](006_SYSTEM_STATUS_AND_MONITORING.md)** - Monitoring tools
- **[docs/reference/jetson_best_practices.md](docs/reference/jetson_best_practices.md)** - Full reference

---

**Status:** ✅ Optimized for R2D2 workloads  
**Last Updated:** December 2025  
**Target Performance:** Real-time perception + speech  
**Power Mode:** MAXN (mode 0)  

