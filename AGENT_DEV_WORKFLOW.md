# R2D2 Agent Development Workflow

**Purpose:** Step-by-step procedures for building, testing, and installing R2D2 features.

**When to Use:** During feature development and testing (before finalization).

**After Testing Complete:** Switch to [`000_AGENT_FINALIZATION_GUIDE.md`](000_AGENT_FINALIZATION_GUIDE.md) for deployment.

**Last Updated:** January 5, 2026

---

## Development Workflow Overview

```
START → Build & Test (this guide) → TESTED & WORKING → Finalize & Deploy (000_AGENT_FINALIZATION_GUIDE.md) → DONE
```

---

## Phase 1: Development & Testing

### Goal
Implement and verify feature works manually.

### Checklist

- [ ] **UX consultation complete** (if feature affects user experience - see Core Rules)
- [ ] Feature implemented and tested manually
- [ ] All ROS 2 packages built successfully (`colcon build`)
- [ ] No linter errors or Python syntax errors
- [ ] Manual testing confirms functionality (`ros2 launch` or `ros2 run`)
- [ ] Code changes committed to git (but not pushed yet)

### Build Commands

```bash
# Navigate to workspace
cd ~/dev/r2d2/ros2_ws

# Build single package
colcon build --packages-select r2d2_<package_name>

# Source the build
source install/setup.bash

# Test manually
ros2 launch r2d2_<package> <launch_file>.py
```

### Clean Rebuild (when cache is stale)

```bash
cd ~/dev/r2d2/ros2_ws
rm -rf build install log
colcon build --packages-select r2d2_<package_name>
source install/setup.bash
```

---

## Phase 2: Production Installation (SYSTEMD SERVICES)

### Goal
Install service so it runs automatically on boot.

### ⚠️ CRITICAL: DO NOT SKIP THIS PHASE

**Common Mistake:** Testing with `ros2 launch` works, but service was never installed/enabled, so after reboot the system is broken.

**A feature is NOT complete until it survives a reboot.**

### Systemd Service Installation Checklist

If your feature involves a systemd service, ALL of these are MANDATORY:

- [ ] Service file created/updated in project root (`~/dev/r2d2/*.service`)
- [ ] **Service file copied to `/etc/systemd/system/`**
  ```bash
  sudo cp ~/dev/r2d2/r2d2-your-service.service /etc/systemd/system/
  ```
- [ ] **`systemctl daemon-reload` executed**
  ```bash
  sudo systemctl daemon-reload
  ```
- [ ] **`systemctl enable <service>` executed**
  ```bash
  sudo systemctl enable r2d2-your-service.service
  ```
- [ ] **`systemctl start <service>` executed**
  ```bash
  sudo systemctl start r2d2-your-service.service
  ```
- [ ] **Verify service is enabled:**
  ```bash
  systemctl is-enabled r2d2-your-service.service  # Must return: enabled
  ```
- [ ] **Verify service is running:**
  ```bash
  systemctl status r2d2-your-service.service  # Must show: active (running)
  ```

### Complete Installation Example

```bash
# Install and enable service
sudo cp ~/dev/r2d2/r2d2-your-service.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable r2d2-your-service.service
sudo systemctl start r2d2-your-service.service

# Verify
systemctl is-enabled r2d2-your-service.service  # Must return: enabled
systemctl status r2d2-your-service.service      # Must show: active (running)
```

---

## ROS 2 Command Reference

### Topics

```bash
# List all topics
ros2 topic list

# Echo topic data (20 messages)
ros2 topic echo /topic_name -n 20

# Check topic frequency
ros2 topic hz /topic_name -w 5

# Show topic info
ros2 topic info /topic_name
```

### Parameters

```bash
# List parameters for a node
ros2 param list /node_name

# Get parameter value
ros2 param get /node_name parameter_name

# Set parameter value (runtime)
ros2 param set /node_name parameter_name value
```

### Services

```bash
# List services
ros2 service list

# Call a service
ros2 service call /service_name service_type "{data: value}"

# Show service type
ros2 service type /service_name
```

### Nodes

```bash
# List running nodes
ros2 node list

# Show node info
ros2 node info /node_name
```

---

## Systemd Service Management

**For complete service documentation:** See [`005_SYSTEMD_SERVICES_REFERENCE.md`](005_SYSTEMD_SERVICES_REFERENCE.md)

### Quick Commands

```bash
# Status
sudo systemctl status r2d2-<service>.service

# Start
sudo systemctl start r2d2-<service>.service

# Stop
sudo systemctl stop r2d2-<service>.service

# Restart
sudo systemctl restart r2d2-<service>.service

# Enable auto-start
sudo systemctl enable r2d2-<service>.service

# Disable auto-start
sudo systemctl disable r2d2-<service>.service

# View logs (live)
journalctl -u r2d2-<service>.service -f

# View logs (last 50 lines)
journalctl -u r2d2-<service>.service -n 50
```

---

## Debugging & Troubleshooting

### Process Management

```bash
# Kill stuck ROS processes
pkill -9 -f ros2 && sleep 2

# Check running processes
ps aux | grep ros2
```

### System Performance

```bash
# CPU/memory
top

# Jetson-specific stats
tegrastats

# Disk usage
df -h
```

### Log Inspection

```bash
# Service logs
journalctl -u r2d2-<service>.service -n 100

# System logs
dmesg | tail -50
```

---

## Documentation Standards

### Before Documenting Parameters

**ALWAYS check if already documented:**

```bash
# Search for existing documentation
grep -ri "parameter_name" ~/dev/r2d2/*.md
```

### Source of Truth Hierarchy

| Topic | Authoritative Document |
|-------|----------------------|
| **Config parameters** | Actual YAML files in `ros2_ws/src/*/config/` |
| **Systemd services** | `005_SYSTEMD_SERVICES_REFERENCE.md` |
| **State machine (RED/GREEN/BLUE)** | `100_PERCEPTION_STATUS_REFERENCE.md` |
| **ROS 2 topics** | `001_ARCHITECTURE_OVERVIEW.md` |
| **Monitoring commands** | `006_SYSTEM_STATUS_AND_MONITORING.md` |
| **Person Registry** | `250_PERSON_MANAGEMENT_SYSTEM_REFERENCE.md` |
| **Speech system** | `200_SPEECH_SYSTEM_REFERENCE.md` |
| **Troubleshooting** | `103_*_TROUBLESHOOTING.md` or `203_*_TROUBLESHOOTING.md` |

### Cross-Reference Format

**For configurable parameters:**
```markdown
> **Source of Truth:** [`audio_params.yaml`](ros2_ws/src/r2d2_audio/config/audio_params.yaml) - check config file for current values
```

**For procedures/commands:**
```markdown
> **See:** [`005_SYSTEMD_SERVICES_REFERENCE.md`](005_SYSTEMD_SERVICES_REFERENCE.md) for complete service management
```

**NEVER duplicate parameter values in documentation - always reference source.**

---

## When You've Completed Phases 1-2

✅ Feature implemented  
✅ Built successfully  
✅ Tested manually and works  
✅ Service installed and running  

**→ NEXT:** Switch to [`000_AGENT_FINALIZATION_GUIDE.md`](000_AGENT_FINALIZATION_GUIDE.md) for:
- Phase 3: Verification (reboot testing)
- Phase 4: Documentation
- Phase 5: Git & Deployment

---

## Common Issues During Development

### Issue: "Illegal instruction (core dumped)"

**Cause:** OpenBLAS optimized for wrong CPU architecture on ARM

**Fix:**
```bash
export OPENBLAS_CORETYPE=ARMV8
```

**Prevention:** Add to `~/.bashrc` (should already be there)

**Details:** See [`103_PERCEPTION_STATUS_TROUBLESHOOTING.md`](103_PERCEPTION_STATUS_TROUBLESHOOTING.md)

### Issue: Build cache is stale

**Solution:** Clean rebuild
```bash
cd ~/dev/r2d2/ros2_ws
rm -rf build install log
colcon build --packages-select <package_name>
```

### Issue: Service won't start after installation

**Diagnosis:**
```bash
journalctl -u r2d2-your-service.service -n 50 --no-pager
```

**Common causes:**
- Missing environment variables (check `Environment=` in service file)
- Wrong working directory (check `WorkingDirectory=` in service file)
- Missing ROS 2 sourcing (add `source /opt/ros/humble/setup.bash` to ExecStartPre)
- File permissions (ensure script is executable: `chmod +x`)

---

**End of Development Workflow Guide**

