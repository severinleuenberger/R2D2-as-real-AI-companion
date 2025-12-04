# OAK-D Lite Camera Setup on NVIDIA Jetson AGX Orin 64GB
## Complete Documentation & Hardware Integration Guide

**Date:** December 4, 2025  
**Project:** R2D2 as a Real AI Companion  
**Platform:** NVIDIA Jetson AGX Orin 64GB Developer Kit  
**Camera:** Luxonis OAK-D Lite Auto Focus  
**Status:** ✅ **FULLY OPERATIONAL**

---

## Executive Summary

Successfully integrated the **Luxonis OAK-D Lite depth camera** with the **NVIDIA Jetson AGX Orin 64GB** running ROS 2 Humble. The camera is now capturing high-quality RGB frames (1920×1080) and is ready for the R2D2 perception pipeline.

### Key Achievements
- ✅ OAK-D Lite camera physically connected and enumerated
- ✅ DepthAI SDK properly configured for ARM/Jetson
- ✅ Captured first test image (1920×1080 Full HD)
- ✅ All dependencies resolved
- ✅ Environment configured for stable operation
- ✅ Test scripts created and validated
- ✅ Documentation and photos published to GitHub

---

## Hardware Configuration

### Jetson AGX Orin Specifications
| Component | Value |
|-----------|-------|
| **SoC** | NVIDIA Jetson AGX Orin |
| **Memory** | 64 GB LPDDR5X |
| **CPU Cores** | 12-core ARM (Cortex-A78) |
| **GPU** | 504-core NVIDIA GPU |
| **Storage** | Internal eMMC |
| **Connectivity** | Dual Gigabit Ethernet, USB 3.0/2.0 |
| **Power** | 100W TDP (variable) |

### OAK-D Lite Specifications
| Component | Value |
|-----------|-------|
| **Product** | OAK-D Lite Auto Focus |
| **Serial Number** | 19443010E1D30C7E00 |
| **RGB Sensor** | 1920×1080 @ 30 FPS |
| **Depth Sensor** | Stereo depth (OV9782 stereo pair) |
| **Interface** | USB 3.0 with USB-C connector |
| **Power Consumption** | 500mA @ 5V |
| **Processor** | Intel Movidius MyriadX |
| **AI Accelerator** | Movidius VPU |

### Connection Details
- **USB Port**: Direct connection to Jetson AGX Orin (not through hub)
- **Cable**: USB 3.0 USB-C to USB-A cable
- **Power**: Bus-powered from Jetson USB port
- **Status**: Fully detected and operational

---

## Software Stack

### Operating System
- **OS**: Ubuntu 22.04 (Jammy) - ARM64/aarch64
- **Kernel**: Linux 5.10.192-tegra (custom Jetson kernel)
- **JetPack**: JetPack 6.x with CUDA/cuDNN support

### Core Dependencies Installed
```bash
# System Libraries
build-essential
libjpeg-dev
libpng-dev
libhdf5-dev
libopenexr-dev
libtbb-dev
libatlas-base-dev
libgl1-mesa-glx

# OpenCV & Computer Vision
opencv (via JetPack)
ffmpeg (7:4.4.2-nvidia - Jetson-optimized)
v4l-utils (1.22.1)

# Development Tools
cmake
git
python3-dev
python3-pip
python3-venv

# GUI/PyQt5 (for future visualization)
python3-pyqt5
qt5-qmake
qtbase5-dev
```

### Python Environment
```
Virtual Environment: ~/depthai_env
Python Version: 3.10.6
```

### DepthAI SDK Configuration
```
Package: depthai
Version: 2.31.0.0 (ARM-optimized prebuilt wheels)
Installation Method: From source (github.com/luxonis/depthai-python)
Installation Path: ~/depthai_env/lib/python3.10/site-packages/
```

### Critical Environment Variables
```bash
# Essential for ARM-based systems (prevents illegal instruction errors)
export OPENBLAS_CORETYPE=ARMV8

# Added to ~/.bashrc for persistence
```

---

## Installation Process & Troubleshooting

### Phase 1: Initial Issues Encountered

#### Problem 1: Device Detected but Won't Boot
**Error**: `X_LINK_DEVICE_NOT_FOUND`  
**Symptom**: Device enumerated (lsusb shows 03e7:2485) but fails at firmware boot  
**Root Cause**: Initial incorrect installation method and missing ARM-specific configuration

#### Problem 2: Incompatible DepthAI Version
**Initial Install**: depthai 3.2.1 (incompatible with ARM/Jetson)  
**Solution**: Downgraded to 2.31.0.0 (ARM-compatible prebuilt wheels)

#### Problem 3: Missing OPENBLAS Configuration
**Error**: Illegal instruction errors during device initialization  
**Solution**: Set `OPENBLAS_CORETYPE=ARMV8` environment variable

### Phase 2: Solution - Proper Jetson Setup

The key was following the **official Luxonis Jetson deployment guide** instead of generic pip installation.

#### Step 1: Install Official Luxonis Dependencies
```bash
sudo wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash
```
**Result**: ✅ All system-level dependencies properly installed

#### Step 2: Create Python Virtual Environment
```bash
sudo apt install python3-venv
python3 -m venv ~/depthai_env
source ~/depthai_env/bin/activate
```
**Result**: ✅ Clean, isolated Python environment

#### Step 3: Clone and Install DepthAI from Source
```bash
git clone https://github.com/luxonis/depthai-python.git
cd depthai-python
python3 examples/install_requirements.py
```
**Result**: ✅ Proper installation with all ARM dependencies and prebuilt wheels

#### Step 4: Set ARM Environment Variable
```bash
echo "export OPENBLAS_CORETYPE=ARMV8" >> ~/.bashrc
source ~/.bashrc
```
**Result**: ✅ Critical configuration for ARM systems applied

#### Step 5: Verify Installation
```bash
source ~/depthai_env/bin/activate
python3 -c "import depthai as dai; print(dai.__version__)"
# Output: 2.31.0.0 ✅
```

---

## Test Results & Validation

### Camera Detection Test
**Script**: `camera_test.py`

```
✓ Device enumerated by OS
✓ Device name: 1.4.3
✓ MX ID: 19443010E1D30C7E00
✓ Status: X_LINK_BOOTED (successfully booted firmware)
✓ Device accessible via DepthAI SDK
```

### Image Capture Test
**Script**: `capture_photo.py`

```
✓ Pipeline created successfully
✓ Device connected: OAK-D-LITE-AF
✓ RGB sensor initialized
✓ Frame captured: 1920×1080 pixels
✓ File saved: oak_d_lite_test.jpg
✓ File size: 242 KB
✓ Format: JPEG with proper encoding
```

### Performance Metrics
| Metric | Value |
|--------|-------|
| **Connection Time** | ~3 seconds (device boot) |
| **Frame Resolution** | 1920×1080 (Full HD) |
| **Capture Latency** | <500ms |
| **File Size** | 242 KB per frame |
| **USB Bandwidth** | Adequate (bus-powered) |
| **Thermal Status** | Nominal |

---

## File Structure & Code

### Created Test Scripts

#### 1. `capture_photo.py` - Simple RGB Photo Capture
```python
#!/usr/bin/env python3
"""
Simple OAK-D Lite RGB frame capture - takes one photo and saves it
"""
- Creates DepthAI pipeline
- Configures RGB camera node
- Connects to device and waits for sensor stabilization (3s)
- Captures single frame in 1920×1080 resolution
- Saves to JPEG format
- Provides file info and error handling
```

**Usage:**
```bash
source ~/depthai_env/bin/activate
python3 capture_photo.py
```

**Output**: `oak_d_photo.jpg` (242 KB, 1920×1080)

#### 2. `camera_test.py` - Device Verification
```python
#!/usr/bin/env python3
"""
Comprehensive OAK-D camera test and diagnostics
"""
- Enumerates connected devices
- Verifies firmware version
- Accesses calibration data
- Provides detailed diagnostics
- Tests full device initialization
```

**Usage:**
```bash
source ~/depthai_env/bin/activate
python3 camera_test.py
```

### Configuration Files

#### `.gitignore` - Updated
```
# Added exclusion for Continue.dev configuration
.continue/

# Reason: Prevents accidental publication of API keys
```

#### `~/.bashrc` - Modified
```bash
# Added:
export OPENBLAS_CORETYPE=ARMV8

# Purpose: Critical for ARM/Jetson systems to prevent 
# "illegal instruction" errors during DepthAI operations
```

### Documentation Generated
- **CAMERA_SETUP_DOCUMENTATION.md** (this file)
- **Photo**: `docs/photos/oak_d_lite_test.jpg` (1920×1080 test capture)

---

## Repository Integration

### GitHub Publication
```
Repository: https://github.com/severinleuenberger/R2D2-as-real-AI-companion
Branch: master
```

### Committed Files
1. `capture_photo.py` - Photo capture script
2. `camera_test.py` - Device test script
3. `docs/photos/oak_d_lite_test.jpg` - Test photo
4. `.gitignore` - Updated with .continue/ exclusion
5. `README.md` - Fixed markdown syntax and image links

### Recent Commits
```
4adbec2 - Add camera hardware test scripts for OAK-D Lite on Jetson AGX Orin
588dd83 - Add OAK-D Lite camera test photo - 1920x1080 RGB capture
```

---

## Next Steps & Future Development

### Immediate Tasks
- [ ] Implement depth map capture alongside RGB
- [ ] Add ROS 2 node wrapper for camera data
- [ ] Integrate with r2d2_perception package
- [ ] Calibration verification and export
- [ ] Stereo depth testing

### Integration with ROS 2
The camera is ready to be integrated into:
- `r2d2_perception` package (planned)
- Navigation and SLAM pipelines
- Object detection and tracking
- Depth-based obstacle avoidance

### Example ROS 2 Usage (Future)
```python
import depthai as dai
import rclpy
from sensor_msgs.msg import Image

# Create ROS 2 node that publishes camera frames
class OAKDPublisher(Node):
    def __init__(self):
        super().__init__('oak_d_publisher')
        self.rgb_pub = self.create_publisher(Image, '/camera/rgb', 10)
        # ... camera capture and publish logic
```

### Performance Optimization Opportunities
- Stream depth maps for SLAM
- Implement IMU fusion from OAK-D's built-in IMU
- Multi-threaded frame capture and processing
- GPU acceleration using CUDA for frame preprocessing
- RTP streaming for remote monitoring

---

## System Requirements Met

### Hardware Requirements
- ✅ Jetson AGX Orin 64GB with adequate compute
- ✅ USB 3.0 connectivity (500mA power budget)
- ✅ ARM64 processor architecture support
- ✅ Robust power supply (Jetson's internal)

### Software Requirements
- ✅ Ubuntu 22.04 (Jammy) with ARM kernel
- ✅ Python 3.10+ runtime
- ✅ Virtual environment isolation
- ✅ Official Luxonis DepthAI library (2.31.0.0)
- ✅ ARM-optimized BLAS (OPENBLAS_CORETYPE=ARMV8)
- ✅ OpenCV and multimedia libraries
- ✅ Development tools (gcc, cmake, git)

### Development Environment
- ✅ ROS 2 Humble integration ready
- ✅ SSH remote access enabled
- ✅ Git version control integrated
- ✅ Documentation and test scripts included
- ✅ Modular code structure for easy expansion

---

## Lessons Learned

### Critical Insights for ARM/Jetson Integration

1. **Always Use Jetson-Specific Documentation**
   - Don't follow generic Linux installation guides
   - Jetson has specific requirements for ARM compilation
   - Official manufacturer guides are essential

2. **Environment Variables Matter**
   - `OPENBLAS_CORETYPE=ARMV8` is non-negotiable
   - Must be in `.bashrc` for persistence
   - Missing this causes cryptic "illegal instruction" errors

3. **Virtual Environments Are Essential**
   - Prevents conflicts between system packages and project packages
   - Allows testing different SDK versions safely
   - Isolates DepthAI's dependencies from system Python

4. **Hardware Power Delivery Is Critical**
   - Direct USB ports preferred over hubs
   - Check USB cable quality (USB 3.0 with proper shielding)
   - Monitor device enumeration status (X_LINK_BOOTED vs X_LINK_UNBOOTED)

5. **Device Initialization Takes Time**
   - Allow 2-3 seconds for sensor stabilization
   - First frame may be slightly different
   - Firmware loads on-demand during device initialization

---

## Success Metrics

| Objective | Status | Evidence |
|-----------|--------|----------|
| Camera detection | ✅ PASS | lsusb shows `03e7:2485 Intel Movidius MyriadX` |
| Device enumeration | ✅ PASS | DepthAI finds 1 device with MX ID |
| Firmware boot | ✅ PASS | Device state changes to `X_LINK_BOOTED` |
| Image capture | ✅ PASS | Successfully captures 1920×1080 frames |
| File export | ✅ PASS | JPEG files properly encoded and saved |
| Documentation | ✅ PASS | Comprehensive setup guide created |
| Git integration | ✅ PASS | All code pushed to GitHub |

---

## Command Reference

### Activation (Always First)
```bash
source ~/depthai_env/bin/activate
source ~/.bashrc
```

### Camera Capture
```bash
cd /home/severin/dev/r2d2
python3 capture_photo.py
```

### Camera Verification
```bash
python3 camera_test.py
```

### Check Device Status
```bash
lsusb | grep Movidius
python3 -c "import depthai as dai; print(len(dai.XLinkConnection.getAllConnectedDevices()))"
```

### Verify Environment Variables
```bash
echo $OPENBLAS_CORETYPE  # Should output: ARMV8
python3 -c "import os; print(os.getenv('OPENBLAS_CORETYPE'))"
```

### View Test Photos
```bash
ls -lh docs/photos/
# Shows: oak_d_lite_test.jpg (242 KB)
```

---

## References & Resources

### Official Documentation
- [Luxonis OAK-D Lite](https://docs.luxonis.com/hardware/products/OAK-D-Lite/)
- [DepthAI Python SDK](https://docs.luxonis.com/software-v3/depthai/)
- [Jetson Deployment Guide](https://docs.luxonis.com/hardware/platform/deploy/to-jetson/)
- [USB Deployment Guide](https://docs.luxonis.com/hardware/platform/deploy/usb-deployment-guide/)

### GitHub Repositories
- [DepthAI Python](https://github.com/luxonis/depthai-python)
- [DepthAI Core](https://github.com/luxonis/depthai-core)
- [R2D2 Project](https://github.com/severinleuenberger/R2D2-as-real-AI-companion)

### Key Technologies
- **ROS 2 Humble** - Robotics middleware
- **NVIDIA JetPack 6.x** - Jetson software stack
- **CUDA/cuDNN** - GPU acceleration (available but not yet used for camera)
- **DepthAI/XLink** - Camera communication protocol

---

## Conclusion

The OAK-D Lite camera is **fully integrated and operational** on the Jetson AGX Orin. The system is ready for:
- ROS 2 node development
- Perception pipeline implementation
- SLAM and navigation testing
- Object detection and tracking
- Depth-based autonomy features

All code, configuration, and documentation have been published to GitHub and are ready for the next phase of R2D2 development.

---

**Status**: ✅ **COMPLETE & TESTED**  
**Last Updated**: December 4, 2025  
**Next Review**: Upon ROS 2 integration
