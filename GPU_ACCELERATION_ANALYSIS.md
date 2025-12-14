# R2D2 System GPU Acceleration Analysis

**Date:** December 9, 2025  
**Platform:** NVIDIA Jetson AGX Orin 64GB  
**Objective:** Assess GPU suitability for all R2D2 services

---

## Analysis Methodology

For each service, we evaluate:
1. **Current implementation** - what the service actually does
2. **GPU suitability classification:**
   - **CPU-only**: No benefit from GPU acceleration
   - **GPU-optional**: Small or incremental benefit, current CPU implementation is sufficient
   - **GPU-strong-candidate**: Clear benefit from GPU acceleration, would significantly improve performance

3. **Justification** - 2-3 bullet points explaining the classification

---

## Service Analysis

### 1. camera_node (OAKDCameraNode)
**Package:** `r2d2_camera`  
**Description:** OAK-D Lite camera driver that interfaces with DepthAI SDK, captures RGB frames at 30 FPS, and publishes them as ROS 2 Image messages to `/oak/rgb/image_raw`.

**Classification:** **CPU-only**

**Justification:**
- Pure I/O operation: interfaces with USB camera via DepthAI SDK, performs minimal frame conversion (DepthAI → OpenCV → ROS Image message)
- No image processing or computation: just data transfer and format conversion
- Camera hardware (OAK-D Lite) has its own onboard processor (Intel Movidius MyriadX) for depth processing; RGB capture is a simple USB transfer

---

### 2. camera_stream_node (CameraStreamNode)
**Package:** `r2d2_camera`  
**Description:** Subscribes to `/oak/rgb/image_raw`, resizes frames (if needed), encodes to JPEG, and serves MJPEG stream via HTTP on port 8081.

**Classification:** **GPU-optional**

**Justification:**
- JPEG encoding could benefit from GPU hardware encoders (NVENC), but current CPU-based OpenCV encoding is sufficient for 15 FPS streaming
- Image resizing is lightweight (single frame, occasional resize) and doesn't justify GPU overhead
- Current CPU usage is low (2-5%), so GPU acceleration would provide minimal benefit relative to implementation complexity

---

### 3. image_listener (ImageListener)
**Package:** `r2d2_perception`  
**Description:** Main perception node that processes camera frames: downscales 1920×1080 → 640×360, converts to grayscale, computes brightness, detects faces using Haar Cascade, and recognizes faces using LBPH (when enabled).

**Classification:** **GPU-strong-candidate**

**Justification:**
- Heavy image processing pipeline: downscaling, color conversion, brightness computation, and face detection on every frame (13 Hz)
- Haar Cascade face detection is CPU-intensive and could be replaced with GPU-accelerated DNN-based detectors (e.g., OpenCV DNN with CUDA backend, TensorRT)
- LBPH face recognition is CPU-bound; modern GPU-accelerated face recognition models (e.g., FaceNet, ArcFace) would provide better accuracy and speed
- Current CPU usage is 8-15% on one core; GPU acceleration could enable higher frame rates or lower CPU usage

---

### 4. audio_notification_node (AudioNotificationNode)
**Package:** `r2d2_audio`  
**Description:** State machine that subscribes to face recognition results, manages 3-state recognition system (RED/BLUE/GREEN), triggers MP3 audio alerts, and publishes status JSON messages.

**Classification:** **CPU-only**

**Justification:**
- Pure state machine logic: subscribes to topics, manages timers, publishes status messages
- No image processing or computation: just conditional logic, state transitions, and subprocess calls for audio playback
- Audio playback (ffplay) is I/O-bound, not compute-bound

---

### 5. status_led_node (StatusLEDNode)
**Package:** `r2d2_audio`  
**Description:** Subscribes to `/r2d2/audio/person_status`, parses JSON, and controls RGB LED via GPIO pins (17, 27, 22) based on recognition state.

**Classification:** **CPU-only**

**Justification:**
- Pure GPIO control: reads JSON messages and sets GPIO pin states (HIGH/LOW)
- No computation: just message parsing and hardware I/O
- GPIO operations are hardware-level and cannot be GPU-accelerated

---

### 6. database_logger_node (DatabaseLoggerNode)
**Package:** `r2d2_audio`  
**Description:** Subscribes to `/r2d2/audio/person_status`, logs state transitions and recognition events to console (future: SQLite database).

**Classification:** **CPU-only**

**Justification:**
- Pure logging/I/O operation: parses JSON messages and writes to console or database
- No computation: just data serialization and file I/O
- Database operations are I/O-bound, not compute-bound

---

### 7. heartbeat_node (HeartbeatNode)
**Package:** `r2d2_hello`  
**Description:** System health monitor that runs `tegrastats`, parses CPU/GPU/temperature metrics, and publishes JSON heartbeat messages at 1 Hz.

**Classification:** **CPU-only**

**Justification:**
- Pure system monitoring: runs system commands, parses text output, publishes metrics
- No computation: just command execution, regex parsing, and message publishing
- System monitoring tools (tegrastats) are already optimized and don't benefit from GPU

---

### 8. audio_beep_node
**Package:** `r2d2_hello`  
**Description:** Simple beep demo node that generates audio beeps.

**Classification:** **CPU-only**

**Justification:**
- Minimal audio generation: simple beep signal generation
- No computation: just audio I/O operations
- Audio synthesis is I/O-bound, not compute-bound

---

### 9. web_dashboard (FastAPI Server)
**Package:** `web_dashboard`  
**Description:** FastAPI web server that provides REST API endpoints for service control, volume control, training management, and serves static HTML/CSS/JavaScript files.

**Classification:** **CPU-only**

**Justification:**
- Pure web server: handles HTTP requests, serves static files, manages systemd services
- No computation: just request routing, file serving, and subprocess calls for service management
- Web servers are I/O-bound (network, disk), not compute-bound

---

### 10. rosbridge_server
**Package:** External ROS 2 package  
**Description:** WebSocket bridge that exposes ROS 2 topics to web clients, forwarding messages bidirectionally.

**Classification:** **CPU-only**

**Justification:**
- Pure message forwarding: converts ROS 2 messages to WebSocket messages and vice versa
- No computation: just serialization/deserialization and network I/O
- Message bridging is I/O-bound, not compute-bound

---

## Summary Table

| Service | Classification | Primary Reason |
|---------|---------------|----------------|
| **camera_node** | CPU-only | Pure I/O operation (USB camera interface, format conversion) |
| **camera_stream_node** | GPU-optional | JPEG encoding could use NVENC, but CPU is sufficient for 15 FPS |
| **image_listener** | GPU-strong-candidate | Heavy image processing (downscaling, face detection, recognition) |
| **audio_notification_node** | CPU-only | Pure state machine logic, no computation |
| **status_led_node** | CPU-only | GPIO control, no computation |
| **database_logger_node** | CPU-only | Logging/I/O operation, no computation |
| **heartbeat_node** | CPU-only | System monitoring, command parsing, no computation |
| **audio_beep_node** | CPU-only | Simple audio I/O, no computation |
| **web_dashboard** | CPU-only | Web server, I/O-bound operations |
| **rosbridge_server** | CPU-only | Message forwarding, serialization, I/O-bound |

---

## Key Findings

1. **Only one service is a strong GPU candidate:** `image_listener` (perception node) performs the majority of compute-intensive operations and would benefit significantly from GPU acceleration.

2. **Most services are I/O-bound:** The majority of services (camera driver, audio, LED control, logging, web server) are I/O-bound or pure logic, with no benefit from GPU acceleration.

3. **Optional GPU benefit:** `camera_stream_node` could use GPU hardware encoders for JPEG compression, but the benefit is marginal given current low CPU usage.

4. **Current system efficiency:** The system runs at 15-25% CPU usage, indicating that CPU resources are not a bottleneck for most services.

---

*Analysis completed: December 9, 2025*
