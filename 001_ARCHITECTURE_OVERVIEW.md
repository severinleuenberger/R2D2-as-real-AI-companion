# R2D2 System Architecture Overview
**Date:** December 7, 2025  
**Platform:** NVIDIA Jetson AGX Orin 64GB + ROS 2 Humble  
**Phase:** 1 - Perception & Face Recognition (Complete)

---

## Executive Summary

The R2D2 perception system is a modular ROS 2-based pipeline that captures video from an OAK-D Lite camera, processes frames in real-time, detects human faces, and recognizes specific individuals. The system prioritizes efficiency (10-15% CPU usage) and extensibility (easy to add new components).

**Current Processing Chain:**
```
OAK-D Lite → r2d2_camera node → /oak/rgb/image_raw (30 Hz)
             ↓
             r2d2_perception node:
             ├─ Downscale (1920×1080 → 640×360)
             ├─ Brightness computation → /r2d2/perception/brightness (13 Hz)
             ├─ Haar Cascade face detection → /r2d2/perception/face_count (13 Hz)
             └─ LBPH face recognition → /r2d2/perception/person_id (6.5 Hz, optional)
             ↓
             Downstream consumers (future: speech, navigation, interaction)
```

---

## 1. System-Level Architecture

### 1.1 Hardware Components

```
┌─────────────────────────────────────────────────────────────────┐
│                     PHYSICAL HARDWARE                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────────┐              ┌──────────────────────┐   │
│  │  OAK-D Lite      │              │  NVIDIA Jetson       │   │
│  │  Camera          │──USB 3.0────▶│  AGX Orin 64GB       │   │
│  │                  │              │                      │   │
│  │ • RGB Sensor     │              │ • 12-core ARM CPU    │   │
│  │   1920×1080@30Hz │              │ • 504-core GPU       │   │
│  │ • Stereo Pair    │              │ • 64GB LPDDR5X RAM   │   │
│  │ • Depth Engine   │              │ • 100W TDP (variable)│   │
│  │ • Auto Focus     │              │ • Ubuntu 22.04       │   │
│  │                  │              │ • JetPack 6.x        │   │
│  │ Serial:          │              │ • ROS 2 Humble       │   │
│  │ 19443010E1D30C7E00               │                      │   │
│  └──────────────────┘              └──────────────────────┘   │
│                                                                 │
│  Connection: USB 3.0 (direct to Jetson, not through hub)      │
│  Power: Bus-powered from Jetson (500mA @ 5V)                  │
│  Status: Fully detected and operational                        │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

**Key Hardware Specs:**
- **Camera:** OAK-D Lite Auto Focus
  - RGB: 1280×1080 actual, 1920×1080 reported (includes padding)
  - Depth: Stereo depth with OV9782 pair
  - FPS: 30 (native camera rate)
  - Interface: USB 3.0 with USB-C connector
  - Processor: Intel Movidius MyriadX

- **Jetson AGX Orin:** NVIDIA's flagship edge AI platform
  - CPU: 12-core ARM Cortex-A78 @ 2.4 GHz
  - GPU: 504-core NVIDIA (can accelerate deep learning)
  - RAM: 64 GB LPDDR5X
  - Storage: Internal eMMC (~32GB usable after JetPack)
  - Thermal: Passive/active cooling (can sustain 100W)

---

### 1.2 Software Stack (Layered)

```
┌─────────────────────────────────────────────────────────────────┐
│                      APPLICATION LAYER                         │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  r2d2_perception (Python ROS 2 Node)                    │  │
│  │  └─ image_listener.py: 354 lines                        │  │
│  │     • Brightness computation                            │  │
│  │     • Haar Cascade face detection                       │  │
│  │     • LBPH face recognition (optional)                  │  │
│  │     • Topic publishing (6 channels)                     │  │
│  └──────────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  r2d2_camera (Python ROS 2 Node)                        │  │
│  │  └─ OAK-D camera driver                                 │  │
│  │     • Initializes DepthAI pipeline                      │  │
│  │     • Streams RGB frames at 30 FPS                      │  │
│  │     • Publishes /oak/rgb/image_raw                      │  │
│  └──────────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  r2d2_hello (ROS 2 Nodes) [Basic Infrastructure]       │  │
│  │  ├─ heartbeat_node: Publish /r2d2/heartbeat (1 Hz)     │  │
│  │  └─ beep_node: Alive signal demo                       │  │
│  └──────────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  r2d2_bringup (Launch Files)                           │  │
│  │  └─ r2d2_camera_perception.launch.py                   │  │
│  │     • Orchestrates camera + perception nodes           │  │
│  │     • Passes parameters to perception node             │  │
│  │     • Enables/disables face recognition                │  │
│  └──────────────────────────────────────────────────────────┘  │
├─────────────────────────────────────────────────────────────────┤
│                    ROS 2 HUMBLE LAYER                          │
│  • rclpy (Python client library)                              │
│  • sensor_msgs (Image message type)                           │
│  • std_msgs (Float32, Int32, String, Bool types)             │
│  • cv_bridge (ROS Image ↔ OpenCV conversion)                 │
│  • roslaunch (launch file system)                            │
├─────────────────────────────────────────────────────────────────┤
│                  COMPUTER VISION & PROCESSING                  │
│  • OpenCV (face detection, image manipulation)                │
│  • OpenCV contrib (LBPH face recognizer)                      │
│  • NumPy (numerical computing)                                │
│  • cv_bridge (image format conversion)                        │
├─────────────────────────────────────────────────────────────────┤
│                    HARDWARE ABSTRACTION                        │
│  • DepthAI SDK 2.31.0.0 (OAK-D camera interface)             │
│  • DepthAI Python bindings                                    │
│  • USB driver (kernel-level USB 3.0 support)                │
├─────────────────────────────────────────────────────────────────┤
│                    OPERATING SYSTEM                            │
│  • Ubuntu 22.04 Jammy (ARM64 aarch64)                        │
│  • Linux kernel 5.10.192-tegra (custom Jetson kernel)        │
│  • JetPack 6.x (NVIDIA system image)                         │
│  • CUDA 12.x + cuDNN (GPU acceleration, optional)            │
├─────────────────────────────────────────────────────────────────┤
│                  HARDWARE (Jetson AGX Orin)                    │
│  • 12-core ARM CPU + 504-core GPU                            │
│  • 64 GB LPDDR5X memory                                      │
│  • Internal eMMC storage                                      │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Data Flow Architecture

### 2.1 Complete Message Flow (Simplified)

```
OAK-D Camera (30 Hz)
      ↓
r2d2_camera node
      ↓
/oak/rgb/image_raw (30 Hz)
      ↓
r2d2_perception node
      ├─ Downscale + Grayscale
      ├─ Brightness calculation → /r2d2/perception/brightness (13 Hz)
      ├─ Face detection → /r2d2/perception/face_count (13 Hz)
      └─ Face recognition (optional) → /r2d2/perception/person_id (6.5 Hz)
      ↓
Downstream consumers (Phase 2-4)
```

---

### 2.2 ROS 2 Topic Reference

Complete list of all topics published:

```
TOPIC                                  TYPE                  FREQ   NOTES
─────────────────────────────────────────────────────────────────────────
/oak/rgb/image_raw                     sensor_msgs/Image     30 Hz  Raw camera
/r2d2/perception/brightness            std_msgs/Float32      13 Hz  Mean brightness
/r2d2/perception/face_count            std_msgs/Int32        13 Hz  Number of faces
/r2d2/perception/person_id             std_msgs/String       6.5 Hz* Person name
/r2d2/perception/face_confidence       std_msgs/Float32      6.5 Hz* Confidence score
/r2d2/perception/is_severin            std_msgs/Bool         6.5 Hz* Severin present?
/r2d2/heartbeat                        std_msgs/String       1 Hz   Health indicator

* Only published if enable_face_recognition=true
```

---

## 3. Node Architecture

### 3.1 Node Details

| Node | Package | Type | FPS In | FPS Out | CPU | Status |
|------|---------|------|--------|---------|-----|--------|
| **camera_node** | r2d2_camera | Sensor driver | N/A | 30 Hz | 2-3% | ✅ |
| **image_listener** | r2d2_perception | Computer vision | 30 Hz | 6 topics | 8-15% | ✅ |
| **heartbeat_node** | r2d2_hello | Health monitor | N/A | 1 Hz | <0.1% | ✅ |

### 3.2 Launch Sequence

```
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py
        ↓
r2d2_camera_perception.launch.py loads
        ↓
   ┌────┴─────────────────────┐
   ↓                           ↓
camera.launch.py      perception.launch.py
   ↓                           ↓
camera_node started    image_listener started
(1-2 sec to ready)     (waits for /oak/rgb/image_raw)
   ↓                           ↓
   └────┬─────────────────────┘
        ↓
   System Ready (5-7 sec total)
   All topics publishing
   Ready for subscribers
```

---

## 4. Processing Pipeline

### 4.1 Step-by-Step Image Processing

```
Each frame (30 FPS):
  1. Receive /oak/rgb/image_raw
  2. Convert ROS Image → OpenCV (BGR array)
  3. Downscale 1920×1080 → 640×360 (4× size reduction)
  4. Convert to grayscale
  5. Compute brightness (mean pixel value)
  6. Detect faces (Haar Cascade)
  7. (If face_count > 0 AND recognition enabled AND skip counter met)
     → Extract face, resize to 100×100, run LBPH recognizer
  8. Publish all results on respective topics

Frame processing time: ~10 ms (without recognition)
                      ~25 ms (with recognition, when triggered)
```

### 4.2 Performance Characteristics

```
RESOURCE USAGE:
  Camera node:         2-3% CPU (one core)
  Perception node:     8-15% CPU (one core) depending on recognition
  Memory total:        ~500 MB
  GPU usage:           ~0% (not accelerated yet)
  
HEADROOM (for Phase 2-4):
  CPU unused:          ~91% (11 cores idle)
  Memory unused:       ~62 GB free
  GPU compute:         >95% available
```

---

## 5. Launch Configuration

### 5.1 Launch Parameters

```
LAUNCH COMMAND:
  ros2 launch r2d2_bringup r2d2_camera_perception.launch.py [ARGS]

AVAILABLE PARAMETERS:
  enable_face_recognition        (bool, default: false)
  recognition_frame_skip         (int, default: 2)
  recognition_confidence_threshold (float, default: 70.0)
  face_recognition_model_path    (string, default: ~/dev/r2d2/data/...)
  log_every_n_frames            (int, default: 30)
  log_face_detections           (bool, default: false)
  save_debug_gray_frame         (bool, default: false)

EXAMPLES:
  # Default (no recognition)
  ros2 launch r2d2_bringup r2d2_camera_perception.launch.py
  
  # With recognition enabled
  ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
    enable_face_recognition:=true
  
  # With verbose logging
  ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
    enable_face_recognition:=true \
    log_every_n_frames:=10 \
    log_face_detections:=true
```

---

## 6. Integration Points for Future Features

### 6.1 Adding Phase 2 Components (Speech/Conversation)

New nodes should:
- Subscribe to `/r2d2/perception/person_id` (who's speaking?)
- Publish to `/r2d2/speech/*` topics (defined by you)
- Follow same naming convention: `/r2d2/<subsystem>/<metric>`

### 6.2 Adding Phase 3 Components (Navigation)

New nodes should:
- Subscribe to `/r2d2/perception/face_count` (obstacle avoidance)
- Subscribe to `/r2d2/perception/person_id` (follow person)
- Publish to `/r2d2/cmd_vel` (movement commands, geometry_msgs/Twist)

### 6.3 General Pattern for New Nodes

```python
from rclpy.node import Node
from std_msgs.msg import String, Float32

class MyNewNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # Subscribe to perception results
        self.person_sub = self.create_subscription(
            String, '/r2d2/perception/person_id',
            self.person_callback, 10)
        
        # Publish new results
        self.output_pub = self.create_publisher(
            String, '/r2d2/subsystem/metric', 10)
    
    def person_callback(self, msg):
        if msg.data == "severin":
            # React to Severin
            pass
```

---

## 7. Key Files Reference

```
MAIN LAUNCH:
  ~/dev/r2d2/ros2_ws/src/r2d2_bringup/launch/r2d2_camera_perception.launch.py

NODES:
  ~/dev/r2d2/ros2_ws/src/r2d2_camera/r2d2_camera/oak_camera_node.py
  ~/dev/r2d2/ros2_ws/src/r2d2_perception/r2d2_perception/image_listener.py

FACE RECOGNITION MODEL:
  ~/dev/r2d2/data/face_recognition/models/severin_lbph.xml

DOCUMENTATION:
  ~/dev/r2d2/020_CAMERA_SETUP_DOCUMENTATION.md
  ~/dev/r2d2/030_PERCEPTION_PIPELINE_SETUP.md
  ~/dev/r2d2/04_FACE_DETECTION_SETUP.md
  ~/dev/r2d2/05_FACE_RECOGNITION_INTEGRATION.md
```

---

## 8. Monitoring Commands

```
# Watch person identification
ros2 topic echo /r2d2/perception/person_id

# Check publication rate (should be 6.5 Hz with recognition)
ros2 topic hz /r2d2/perception/person_id

# Monitor CPU usage
watch -n 1 'top -bn1 | grep -E "python|Cpu" | head -10'

# List all active topics
ros2 topic list

# View node information
ros2 node info /image_listener
```

---

## Next Steps

1. **Understand**: Read this document, visualize the data flow
2. **Verify**: Run system and confirm all topics are publishing
3. **Extend**: Add Phase 2 features using integration patterns above
4. **Monitor**: Use provided commands to track performance

**Next Document**: `010_PROJECT_GOALS_AND_SETUP.md` (project roadmap and setup) or `QUICK_START.md` (quick reference)

---

*Architecture Overview created: December 7, 2025*
