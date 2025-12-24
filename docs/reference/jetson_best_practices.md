# NVIDIA Jetson AGX Orin: Official Best Practices & Recommendations

**Last Updated:** December 2025  
**Sources:** NVIDIA Official Documentation (r36.4.4, r38.2.1), jetson-containers, NVIDIA Developer Forums

---

## 1. POWER MANAGEMENT & THERMAL OPTIMIZATION

### 1.1 Power Modes (NVPMODEL)

**Supported Modes:**
- **MAXN**: All CPU cores enabled, GPU at maximum frequency (high power, ~25-30W continuous)
- **10W**: Single CPU core enabled, reduced GPU/memory clocks
- **15W**: Limited CPU core count, reduced frequencies
- **Custom**: User-defined power budgets

**Best Practices:**
- Use `nvpmodel -l` to list supported modes
- Use `nvpmodel -m <mode>` to switch modes dynamically
- Query current mode: `nvpmodel -q --verbose`
- **Default thermal-aware switching**: System automatically throttles when threshold approaching
- For sustained operations: Use 15W mode for continuous workloads to maintain thermal headroom
- Reserve 5-10°C margin below 80°C thermal limit

### 1.2 Thermal Management

**Critical Specifications:**
- **Thermal Shutdown Point**: 95°C (hardware enforced)
- **Recommended Operating Limit**: 80°C
- **Throttling Begins**: ~75°C (frequency reduction starts)
- **Optimal Range**: 40-75°C for sustained operation

**Linux Thermal Framework:**
- Thermal zones: `/sys/class/thermal/thermal_zone*/temp`
- Current throttling state: `/proc/tegra_throttle_alert`
- Thermal sensors available:
  - CPU thermal zones (multiple cores)
  - GPU thermal zone
  - Memory controller thermal zone
  - Power management IC (PMIC) thermal zone

**Cooling Strategies:**
1. **Fan Control**:
   - Use `nvfancontrol` service (default: enabled)
   - Custom profiles: `/etc/nvfancontrol.conf`
   - Fan profiles: Conservative, Moderate, or Maximum
   - Monitor fan speed: `grep -i fan /var/log/syslog`

2. **Thermal Optimization**:
   - Improve airflow around heatsink
   - Ensure 2cm clearance minimum around module
   - Use thermal interface material (TIM) between module and heatsink
   - Install carrier board fans if available
   - Consider liquid cooling for industrial applications (P3701-0008 Industrial grade)

3. **Software Mitigation**:
   - Monitor thermal state continuously in production
   - Implement graceful performance degradation at 70°C
   - Pre-emptively reduce workload at 72°C
   - Aggressive throttle recovery may take 30-60 seconds after cooling

### 1.3 Power Consumption Modeling

**Jetson AGX Orin Power Budget (MAXN mode):**
- Typical: 25-30W continuous (CPU-intensive tasks)
- Peak: 35W (full CPU + GPU utilization)
- Idle: 8-10W
- Sleep (SC7): <1W

**Power Rail Configuration (via PMIC):**
- Monitor via: `cat /sys/class/hwmon/hwmon*/in*_label`
- Individual rail monitoring available
- Power limits can be configured in device tree

**Measurement:**
- Real-time: `tegrastats` (updates every 1 second)
- Output format: CPU%, GPU%, SOC%, EMC%, APE%, TEMP

---

## 2. MEMORY MANAGEMENT & OPTIMIZATION

### 2.1 Memory Architecture

**LPDDR5 Memory:**
- AGX Orin 64GB: 64GB LPDDR5 (1,456 Gbit/s bandwidth)
- AGX Orin 32GB: 32GB LPDDR5 (same bandwidth per byte)
- Unified memory with CPU/GPU coherency

**Memory Spaces:**
- **System RAM**: All user applications
- **Carveouts**: Reserved for specific subsystems (bootloader configuration)
  - IMU carveout
  - Display carveout
  - NVENC/NVDEC carveouts (typically 500MB-1GB)

### 2.2 Memory Optimization

**Query Memory Status:**
```bash
free -h                           # Overall memory
cat /proc/meminfo                 # Detailed breakdown
cat /sys/kernel/debug/kmemleak    # Memory leak detection
nvidia-smi                        # GPU memory (if available)
tegrastats                        # Real-time memory usage
```

**Memory Frequency Scaling:**
- EMC (External Memory Controller) frequency: `/sys/kernel/debug/clk/emc/clk_rate`
- Default: 3,200 MHz (fastest safe rate)
- EMC frequency locked to highest available unless on power-constrained mode
- **Critical**: EMC frequency directly affects system performance

**Optimization Techniques:**
1. **Swap Configuration** (optional):
   - Pre-create swap file if physical memory will approach 90%
   - Not recommended for performance-critical operations
   - Add to boot time: ~5-10 seconds per GB swap

2. **Cache Management**:
   - Enable L3 cache: Default enabled
   - Page cache sizing: Kernel auto-manages
   - Drop caches (testing only): `sync && echo 3 > /proc/sys/vm/drop_caches`

3. **NUMA Awareness** (if applicable):
   - Single NUMA node on AGX Orin
   - Not a concern for this platform

4. **GPU Memory Management**:
   - Unified memory: CPU and GPU share address space
   - CUDA unified memory: Automatic migration between CPU/GPU
   - Monitor with `nvidia-smi` if CUDA-capable GPU operations active

### 2.3 Memory Bandwidth Optimization

**Peak Memory Bandwidth:**
- ~1,456 Gbit/s = 182 GB/s (theoretical)
- ~140-160 GB/s (practical, accounting for overhead)

**Best Practices:**
- Align data structures to cache lines (64 bytes)
- Use NEON/SIMD intrinsics for CPU operations
- Batch memory operations to reduce round trips
- Use DMA when available (video decode, multimedia)
- Pin critical memory regions if using CUDA

---

## 3. ROS 2 INTEGRATION BEST PRACTICES

### 3.1 ROS 2 on Jetson

**Compatible Distributions:**
- ROS 2 Humble (on JetPack 5.x)
- ROS 2 Iron (on JetPack 6.x)
- Real-Time Kernel support available (custom build required)

**Installation Methods:**
1. **jetson-containers** (Recommended):
   ```bash
   jetson-containers run $(autotag ros:humble-desktop)
   ```
   - Provides pre-built, optimized containers
   - Includes necessary GPU support
   - Avoids long build times on Jetson hardware

2. **Native Installation**:
   ```bash
   apt install ros-humble-desktop  # Full desktop install
   ```
   - Longer initial setup time
   - Uses Jetson-optimized packages from NVIDIA repositories

### 3.2 ROS 2 Performance on Jetson

**Latency Characteristics:**
- **DDS Middleware Latency**: 1-5ms typical (varies by QoS)
- **Context Switch Overhead**: 0.5-2ms (standard kernel)
- **Message Serialization**: 0.1-0.5ms (depends on message size)
- **Total End-to-End Latency**: 2-10ms for typical robotic applications

**Real-Time Challenges:**
- Standard Linux kernel (5.15): Not real-time capable
- Maximum achievable determinism: ~10ms jitter
- For hard real-time: Requires PREEMPT-RT kernel patch
  - Reduces latency to 0.5-2ms range
  - Available as optional installation via JetPack OTA

### 3.3 ROS 2 Optimization Strategies

**QoS (Quality of Service) Configuration:**
- **Reliability**: Use BEST_EFFORT for sensor streams, RELIABLE for critical commands
- **History**: Keep minimal (1 or 5) to reduce memory overhead
- **Deadline**: Set realistic deadlines (default 0 = infinite)
- **Lifespan**: Discard stale messages automatically

**Example QoS for Real-Time:**
```python
# Minimize latency for time-critical sensors
qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE,
    lifespan=Duration(seconds=0.1)
)
```

**Performance Tuning:**
1. **Executor Selection**:
   - SingleThreadedExecutor: Lower latency, sequential processing
   - MultiThreadedExecutor: Better throughput, higher variance in latency
   - Recommend SingleThreadedExecutor for real-time robotics

2. **Node Allocation**:
   - Pin critical nodes to specific CPU cores
   - Use cgroup isolation: `cgcreate -g cpuset:/robot_critical`
   - Reserve 1-2 cores for OS/system services

3. **Memory Tuning**:
   - Pre-allocate message pools
   - Use rmw_cyclonedds_cpp (faster than rmw_opensplice)
   - Disable DDS discovery on subsequent launches if topology static

4. **Network Configuration**:
   - For localhost only: Use `ROS_LOCALHOST_ONLY=1`
   - Avoids multicast DNS overhead
   - Reduces latency by ~1-2ms for local communication

### 3.4 Container-Based ROS 2 (Recommended)

**Advantages:**
- Deterministic environment (same across devices)
- GPU support automatic via runtime
- Cleaner system management
- Easy version upgrades

**Container ROS 2 Build:**
```bash
# Build with essential packages
jetson-containers build --name ros2-robot \
  ros:humble-desktop \
  cmake build-essential python3-pip
```

**Mounting Considerations:**
- Mount `/dev/shm` with sufficient size for message queues
- Mount with `-u 0:0` if accessing privileged hardware (GPIO, I2C)
- Use `--ipc=host` for optimal IPC performance

---

## 4. GPU-ACCELERATED AI INFERENCE PATTERNS

### 4.1 Inference Engines

**Available Options (JetPack 5.x/6.x):**

1. **TensorRT** (Recommended for most use cases):
   - Native NVIDIA runtime
   - Latency: 10-50ms per inference (depends on model size)
   - Throughput: 20-100 FPS (4-8W power)
   - Supports INT8, INT4 quantization
   - Dynamic shape support

2. **ONNX Runtime**:
   - Model portability
   - Latency: 20-100ms per inference
   - Good for cross-platform deployment
   - May be slower than TensorRT by 20-40%

3. **PyTorch (native)**:
   - Development-focused
   - Latency: 50-200ms per inference
   - Higher power consumption (8-12W for inference)
   - Not recommended for production due to overhead

4. **MediaPipe (on-device ML)**:
   - Optimized for mobile/edge
   - Latency: 5-20ms per inference
   - Lower power consumption
   - Limited model flexibility

### 4.2 Quantization Strategy

**Recommended Quantization Path:**
1. **Development**: Float32 model (full precision)
2. **Optimization**: Float16 (half precision) - 2x speedup, negligible accuracy loss
3. **Production**: INT8 (8-bit integer) - 4x speedup, requires calibration dataset
4. **Ultra-efficient**: INT4 (requires custom kernels, 8x speedup)

**TensorRT Quantization Example:**
```bash
# Convert PyTorch → ONNX → TensorRT with INT8 calibration
python3 -m torch.onnx.export model.pt model.onnx ...
trtexec --onnx=model.onnx --saveEngine=model.trt --int8
```

**Latency Expectations (ResNet50 on AGX Orin MAXN):**
- Float32: ~40ms
- Float16: ~20ms
- INT8: ~10ms

### 4.3 Inference Pipeline Optimization

**Batching:**
- Batch size 1-4 recommended for real-time (latency <50ms requirement)
- Batch size 8-16 for throughput-optimized applications
- Measure latency vs. throughput tradeoff

**Pipelining:**
- Overlap input preparation, inference, output processing
- Use asynchronous CUDA kernels
- Pre-allocate GPU memory for all batch sizes

**Memory Management:**
- Pre-allocate GPU buffers at startup
- Avoid repeated allocations in inference loop
- Use pinned (page-locked) host memory for CPU-GPU transfers

**Example Pattern:**
```python
# Pre-allocate buffers
input_buffer = cuda.pagelocked_empty(shape, dtype=np.float32)
output_buffer = cuda.pagelocked_empty(out_shape, dtype=np.float32)

# Inference loop
for batch in data_stream:
    np.copyto(input_buffer, batch)  # CPU → pinned memory
    cuda_input = cuda.mem_alloc_pinned(input_buffer)
    engine.infer(cuda_input, output_buffer)  # GPU inference
    np.copyto(result, output_buffer)  # GPU → pinned memory
```

### 4.4 Multi-Model Inference

**Concurrent Model Execution:**
- Maximum 2-3 models simultaneously (resource contention)
- Share GPU memory pool across models
- Use separate CUDA streams for different models
- Avoid synchronization bottlenecks

**Resource Allocation Pattern:**
- Model 1 (detect): 50% GPU + 4 CPU cores
- Model 2 (track): 40% GPU + 2 CPU cores
- Remaining for system and monitoring

---

## 5. CONTAINER DEPLOYMENT BEST PRACTICES

### 5.1 Docker on Jetson

**Runtime Configuration:**
- Use NVIDIA Container Runtime: `--runtime nvidia`
- Automatic GPU access within container
- Shared CUDA/cuDNN libraries from host

**Container Build Optimization:**
1. **Base Image**:
   ```dockerfile
   # Use l4t-base (minimal) or l4t-ml (with ML libraries)
   FROM nvcr.io/nvidia/l4t-ml:r36.2.0
   ```

2. **Multi-Stage Builds**:
   - Reduce final image size
   - Minimize build time on slow Jetson storage

3. **Example Dockerfile**:
   ```dockerfile
   FROM nvcr.io/nvidia/l4t-ml:r36.2.0
   
   # Minimize layers
   RUN apt-get update && apt-get install -y \
       python3-pip \
       && rm -rf /var/lib/apt/lists/*
   
   COPY requirements.txt .
   RUN pip install --no-cache-dir -r requirements.txt
   
   COPY app/ /app/
   WORKDIR /app
   
   CMD ["python3", "main.py"]
   ```

### 5.2 Image Size Optimization

**Target Image Size:**
- Minimal: 500MB-1GB
- With ML libs: 3-5GB
- With ROS: 5-8GB

**Reduction Strategies:**
1. Remove unnecessary packages: `-y` avoid package manager caches
2. Use multi-stage builds: Copy only binaries to final stage
3. Strip binaries: `find /app -name '*.so*' -exec strip {} \;`
4. Compress layers: Enable compression in registry

### 5.3 Storage and Performance

**Storage Considerations:**
- eMMC (16/32GB typical): 200-400 MB/s write
- NVMe expansion (if available): 1000+ MB/s
- Container images on eMMC: No major issue (modern filesystems optimize)

**Performance Tips:**
1. **Memory File System**:
   ```bash
   mount -t tmpfs -o size=2G tmpfs /mnt/ramdisk
   # Copy frequently-accessed files here
   ```

2. **Volume Mounts**:
   - Use `-v /path/host:/path/container` for persistent data
   - Avoid high-frequency I/O in mounted volumes
   - Measured overhead: 5-10% for I/O operations

3. **Device Access**:
   ```bash
   docker run --runtime nvidia \
     --device /dev/video0:/dev/video0 \
     --device /dev/i2c-1:/dev/i2c-1
   ```

### 5.4 jetson-containers Framework

**Key Advantages:**
- Pre-built, JetPack-compatible images
- Automatic GPU runtime configuration
- Extensive package library (ROS, PyTorch, TensorRT, etc.)
- Simplified build process

**Build Commands:**
```bash
# List available packages
jetson-containers list

# Build custom container
jetson-containers build --name my-app pytorch transformers opencv

# Run with auto-detection
jetson-containers run $(autotag pytorch)

# Run with custom args
docker run --rm -it $(autotag l4t-pytorch) python3 -c "import torch; print(torch.__version__)"
```

**Best Practices:**
- Use autotag for version compatibility
- Pin CUDA versions for reproducibility
- Test locally before deployment

---

## 6. REAL-TIME COMMUNICATION vs ROS 2 OVERHEAD

### 6.1 Latency Profile

**Standard ROS 2 Pipeline:**
```
Sensor Input (0.5ms) 
  → Publisher Queue (0.5-1ms) 
  → DDS Middleware (1-3ms) 
  → Subscriber Queue (0.5-1ms) 
  → Callback (variable)
  → Processing (application dependent)
Total: 2-10ms end-to-end
```

**Measured Jitter (Standard Kernel):**
- Single-threaded executor: 2-5ms jitter
- Multi-threaded executor: 5-15ms jitter
- With background processes: 10-25ms jitter

### 6.2 Reducing ROS 2 Overhead

**Strategy 1: Executor Optimization**
- SingleThreadedExecutor: 0.5ms overhead
- MultiThreadedExecutor: 2-3ms overhead
- Callback Group Strategy: Isolate critical callbacks

**Strategy 2: DDS Tuning**
```python
# Minimize DDS overhead
qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # UDP instead of TCP
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,                                         # Single message buffer
    deadline=Duration(milliseconds=100),
    lifespan=Duration(milliseconds=50)
)
```

**Strategy 3: Network Configuration**
```bash
# ROS2 specific network optimization
export ROS_LOCALHOST_ONLY=1              # Disable multicast DNS
export ROS_DISCOVERY_TIMEOUT=300         # Speed up discovery
export CYCLONEDDS_URI=file:///config.xml # Custom DDS config
```

### 6.3 Hard Real-Time Alternative: Direct Communication

**For sub-5ms Latency Requirements:**
1. **Inter-Process Communication (IPC)**:
   - Shared memory: <1ms latency
   - Message queues: 0.5-2ms latency
   - Requires manual synchronization

2. **Direct Hardware Access**:
   - Memory-mapped I/O: 0.1-0.5ms
   - DMA transfers: Asynchronous, <1ms startup
   - Requires kernel driver or privileged access

**Example Direct IPC (vs ROS 2):**
```cpp
// Shared memory approach (vs ROS 2's ~3-5ms)
// Latency: ~0.5ms
struct SharedData {
    float sensor_values[10];
    uint64_t timestamp;
};

// Access directly from multiple processes
// Zero-copy, <0.5ms latency
```

### 6.4 Decision Matrix

| Requirement | Recommended Approach | Expected Latency |
|------------|----------------------|------------------|
| <5ms, High Reliability | Real-Time Kernel + ROS 2 | 2-5ms |
| 5-20ms, High Throughput | Standard ROS 2 + Tuning | 5-15ms |
| <1ms, Single System | Direct IPC / Shared Memory | 0.5-2ms |
| Distributed System | ROS 2 with DDS | 10-50ms (network dependent) |

---

## 7. AUDIO/MULTIMEDIA HANDLING ON JETSON

### 7.1 Audio Infrastructure

**Audio System:**
- ALSA (Advanced Linux Sound Architecture): Primary audio API
- PulseAudio/PipeWire: Optional user-space audio server
- HDA (High Definition Audio) or I2S interface available

**Audio Hardware:**
- HDMI audio output (from HDMI connector)
- I2S/DMIC connectors (via GPIO or dedicated pins)
- 40-pin expansion header audio options

### 7.2 Audio Capture and Playback

**Recommended Libraries:**
1. **GStreamer** (Primary):
   - `nvv4l2camerasrc` for camera
   - `alsasrc`/`alsasink` for audio
   - Hardware-accelerated codec plugins

2. **ALSA** (Low-level):
   - Direct audio device access
   - Lower latency (~5-20ms) than PulseAudio
   - Recommended for real-time audio

3. **PortAudio** (Cross-platform):
   - Abstraction over ALSA/PULSE
   - Easier portability

**Audio Latency:**
- ALSA direct: 5-20ms
- PulseAudio: 20-50ms
- GStreamer pipeline: 30-100ms (depends on buffer sizes)

### 7.3 Multimedia Encoding/Decoding

**Hardware Acceleration Available:**
1. **NVENC** (Video Encoding):
   - H.264, H.265 (HEVC), AV1 support
   - Throughput: 4K@60fps H.265 (~5-8W)
   - Latency: 10-30ms per frame

2. **NVDEC** (Video Decoding):
   - H.264, H.265, VP9, AV1 support
   - Throughput: 4K@120fps capable
   - Latency: 5-15ms per frame

3. **VIC** (Video Image Composer):
   - Scaling, color conversion
   - Throughput: 4K@60fps capable
   - Latency: 2-5ms

**GStreamer Pipeline Example:**
```bash
# Hardware-accelerated video decode + encode
gst-launch-1.0 \
  filesrc location=input.h264 ! \
  h264parse ! \
  nvv4l2decoder ! \
  nvvidconv ! \
  'video/x-raw,format=NV12,width=1280,height=720' ! \
  nvv4l2h265enc bitrate=2000 ! \
  h265parse ! \
  mp4mux ! \
  filesink location=output.mp4
```

### 7.4 Audio/Video Synchronization

**Best Practices:**
1. **Timestamp Alignment**:
   - Use monotonic system clock for sync reference
   - RTP timestamp for network streams
   - Hardware sync points in encoder/decoder

2. **Buffer Management**:
   - Audio buffer: 5-20ms (lower = more responsive)
   - Video buffer: 20-40ms (jitter resilience)
   - Adjust based on network/disk performance

3. **Drift Compensation**:
   - Monitor audio/video delay continuously
   - Implement resampling or frame skipping if drift >100ms
   - GStreamer audio-sink can auto-adjust

**Typical A/V Sync Error:** 20-50ms (perceptually acceptable)

---

## 8. ADDITIONAL OPTIMIZATION RECOMMENDATIONS

### 8.1 CPU Frequency Scaling

**cpufreq Configuration:**
```bash
# Check current scaling
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor  # Show governor

# Switch to performance mode (highest clock)
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Switch to ondemand (dynamic frequency based on load)
echo ondemand | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# View available frequencies
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_available_frequencies
```

**Recommendation:**
- Development: ondemand or schedutil
- Production: fixed frequency (performance) for determinism, or carefully tuned ondemand

### 8.2 GPU Frequency Scaling

**GPU Clock Settings:**
```bash
# Check available clocks
cat /sys/devices/gpu.0/devfreq/*/available_frequencies

# Set fixed frequency (max 1.4 GHz typical)
echo <frequency_hz> | sudo tee /sys/devices/gpu.0/devfreq/*/userspace/set_freq
```

### 8.3 Process Pinning & Priority

**CPU Affinity:**
```bash
# Run critical process on specific cores (0-3, leaving 4-7 for OS)
taskset -c 0-3 ./critical_app

# Adjust priority (higher = more CPU time)
nice -n -10 ./important_task  # Higher priority
nice -n 10 ./background_task  # Lower priority
```

**cgroup Isolation:**
```bash
# Create isolated CPU set for critical tasks
sudo cgcreate -g cpuset:/critical_path
echo 0-3 | sudo tee /sys/fs/cgroup/cpuset/critical_path/cpuset.cpus
echo 0 | sudo tee /sys/fs/cgroup/cpuset/critical_path/cpuset.mems

# Move process to group
echo $PID | sudo tee /sys/fs/cgroup/cpuset/critical_path/cgroup.procs
```

### 8.4 USB and I/O Performance

**USB Device Classes:**
- USB 2.0 (480 Mbps): Typical sensors, controllers
- USB 3.0 (5 Gbps): Video capture, NVMe external drives

**I2C Optimization:**
```bash
# Check clock frequency
i2cdetect -l  # List I2C buses

# Typical speed: 100 kHz (standard), 400 kHz (fast mode)
# Device tree configuration controls clock
```

**SPI Optimization:**
```bash
# SPI clock speeds: 10, 25, 50 MHz typical
# Higher speeds require proper signal integrity
```

### 8.5 Kernel Parameter Tuning

**Performance-Oriented sysctl Settings:**
```bash
# Increase file descriptor limits
sysctl -w fs.file-max=2097152

# Optimize TCP for robotics (UDP-based ROS preferred)
sysctl -w net.core.rmem_max=134217728
sysctl -w net.core.wmem_max=134217728

# Disable swap entirely (if sufficient RAM)
sysctl -w vm.swappiness=0

# Increase network buffer
sysctl -w net.ipv4.tcp_window_scaling=1
```

---

## 9. COMMON PITFALLS & TROUBLESHOOTING

### 9.1 Thermal Issues

**Symptom:** Performance drops suddenly  
**Likely Cause:** Thermal throttling (75°C+)  
**Solution:**
1. Check temperature: `tegrastats | grep temp`
2. Improve cooling (fan speed, airflow)
3. Reduce workload or switch to lower power mode
4. Monitor with thermal graph over time

### 9.2 Out of Memory (OOM)

**Symptom:** Application crashes without error  
**Likely Cause:** 64GB memory limit exceeded (unlikely) or memory leak  
**Solutions:**
1. Profile with: `top -b -n 1 | head -15`
2. Check swap usage: `free -h`
3. Use memory profiler: `pip install memory-profiler`
4. Monitor GPU memory if using CUDA: `nvidia-smi`

### 9.3 ROS 2 Latency Spikes

**Symptom:** Occasional 50-100ms latency in otherwise <10ms system  
**Likely Cause:** OS context switches, garbage collection  
**Solutions:**
1. Enable PREEMPT-RT kernel (JetPack OTA)
2. Use `chrt` to set real-time priority for critical nodes
3. Reduce background processes (disable GUI, unused services)
4. Profile with `perf record -g` to find hot spots

### 9.4 Docker GPU Access Issues

**Symptom:** GPU not visible in container (`nvidia-smi` fails)  
**Solution:**
```bash
# Ensure NVIDIA Container Runtime installed
docker run --rm --runtime=nvidia nvidia/cuda:11.4.2-runtime nvidia-smi

# If still failing:
# 1. Restart docker daemon
sudo systemctl restart docker

# 2. Check host nvidia-smi first
nvidia-smi  # Should work on host

# 3. Verify container runtime
docker info | grep nvidia
```

---

## 10. SUMMARY DECISION TREE

```
┌─ What's your primary requirement?
│
├─ [Real-time Robotics]
│  ├─ Latency < 5ms? → Install PREEMPT-RT kernel + ROS 2
│  ├─ Latency 5-20ms? → Standard ROS 2 + SingleThreadedExecutor
│  └─ Distributed? → ROS 2 + DDS tuning
│
├─ [AI Inference]
│  ├─ Accuracy critical? → TensorRT INT8 after validation
│  ├─ Speed critical? → TensorRT INT8 + batch optimization
│  ├─ Multiple models? → Separate CUDA streams + memory pooling
│  └─ Deployment? → Container via jetson-containers
│
├─ [Multimedia/Audio]
│  ├─ Real-time encoding/decoding? → GStreamer + NVENC/NVDEC
│  ├─ Audio processing? → ALSA direct or GStreamer
│  ├─ A/V sync? → Use monotonic clock + RTP timestamps
│  └─ High bitrate video? → H.265 + NVENC, monitor thermals
│
├─ [Power Constrained]
│  ├─ Continuous operation → 15W mode
│  ├─ Peak performance needed? → Optimize code, use 10W mode intermittently
│  └─ Thermal limited? → Monitor constantly, pre-throttle at 70°C
│
└─ [Development/Deployment]
   ├─ Quick iteration? → Native install + ROS 2
   ├─ Reproducibility? → Containers via jetson-containers
   └─ Production? → Container + hardware redundancy + thermal monitoring
```

---

## REFERENCES

- NVIDIA Jetson Linux Developer Guide (r36.4.4, r38.2.1)
- jetson-containers Documentation & Examples
- ROS 2 Official Documentation (Humble, Iron)
- NVIDIA Developer Forums & Technical Blogs
- Jetson AGX Orin Module Datasheet & Thermal Guidelines

