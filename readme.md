# AI R2-D2 Companion Bot


![Empty DeAgostini Body](docs/photos/empty_body.jpg)
*1:2-Scale DeAgostini R2-D2 → Autonomous AI Companion: Indoor Nav, Person Recognition, Conversation (Llama-3-8B + Grok-4 Fallback), Fetch/Carry.*

![Build Photo Nov 7](docs/photos/20251107_105518.jpg)

**Core:** Jetson AGX Orin 64GB, ROS2 Humble, OAK-D Lite, ReSpeaker mic. 1/2 Scale: 48cm tall.

**Goal:** Convert stock kit DeAgostini 1:2 R2-D2 to autonomous AI companion:  90% pre-built ROS2 packages; <50 lines custom code. Focus: Modularity, reliability. With 
 indoor nav, person recognition, real speach conversations  (local Llama-3-8B + Grok-4 fallback if conf <70%).

**Assumptions:** specified parts/tutorials available

---

## Quick Start

### Prerequisites
- Flash Jetson AGX Orin with [JetPack 6.x](https://developer.nvidia.com/embedded/jetpack) (Ubuntu 22.04).
- Install ROS2 Humble: Follow [official docs](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
- Clone repo: `git clone https://github.com/severinleuenberger/R2D2-as-real-AI-companion.git && cd R2D2-as-real-AI-companion`.
- Build: `colcon build --symlink-install && source install/setup.bash`.

### Launch
```bash
# In one terminal (for nav + interaction)
ros2 launch r2d2_navigation nav_launch.py  # SLAM + Nav2

# In another terminal
ros2 launch r2d2_llm tts_stt_launch.py     # Speech → LLM → Actions
````


### Section: 1. Features and Requirements

## 1. Features and Requirements

### 1.1 Intelligent Speech with Fallback Logic
- **Pipeline:** Speech → STT → Local LLM → (if logprobs <0.7) → Grok-4 API → TTS/Action.
- **Local LLM:** Llama-3-8B via [llama.cpp](https://github.com/ggerganov/llama.cpp) + [llama_ros](https://github.com/mgonzs13/llama_ros).
- **Fallback:** Custom ROS2 node evaluates confidence; switches to [xAI Grok-4 API](https://x.ai/api).
- **Impl:** [jetson-voice](https://github.com/dusty-nv/jetson-voice) for audio + Python requests node (≤30 lines).

### 1.2 Person Recognition & Memory
- **Detection:** [Isaac ROS YOLO](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_yolo) or [jetson-inference](https://github.com/dusty-nv/jetson-inference).
- **Persistence:** Embeddings with [NVIDIA ReMEmbR](https://github.com/NVIDIA/NeMo-Aligner) stored in SQLite.
- **Privacy:** Voice command "Forget me" deletes DB row.
- **Impl:** See `src/r2d2_perception/memory.py`.

### 1.3 Contextual Conversation
- **I/O:** [ros2_speech_recognition](https://github.com/Roboy/ros2_speech_recognition) (Roboy).
- **Association:** Link face/audio to thread.
- **Storage:** Per-person history in SQLite (see `src/r2d2_llm/memory.py`).
- **Impl:** Custom node for thread management (≤20 lines).

### 1.4 Autonomous Navigation & Mapping
- **SLAM:** [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) (online sync mode).
- **Stack:** AMCL localization + Nav2 planning/avoidance.
- **Ref:** [Waveshare UGV Rover Tutorial](https://www.waveshare.com/wiki/ROS2-based_UGV_ROVER_Tutorial) for launch files.
- **Impl:** Adapted launch in `src/r2d2_navigation/`.

### 1.5 Multi-Room & Semantic Mapping
- **Merging:** slam_toolbox multi-session.
- **Semantics:** [Isaac ROS Segmentation](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_segmentation) + jetson-inference labels.
- **Memory:** Object poses in map YAML.
- **Impl:** YAML extensions + perception node.

### 1.6 Object Manipulation
- **Control:** [ros2_control](https://github.com/ros-controls/ros2_control).
- **Pick/Place:** Adapt [ROS2_pick_and_place_UR5](https://github.com/JuoTungChen/ROS2_pick_and_place_UR5).
- **Gripper:** Servo mount (see `hardware/cad/`).
- **Impl:** ros2_control config for DeAgostini arms.



## 2. Hardware Components

### Base Model
| Spec | Value |
|------|-------|
| DeAgostini 1:2-Scale R2-D2 Kit | [Buy](https://www.deagostini.com/) |
| Height | 48 cm |
| Width | 28 cm |
| External Ø | 20 cm |
| Internal Volume | 4.5–7.2 L |
| Reuse | Drive/arms/dome |
| Cost | 300–700 CHF |

### Compute
| Part | Specs | Link |
|------|--------|------|
| NVIDIA Jetson AGX Orin 64GB Dev Kit | 100×87×47 mm, 15–60 W | [Reichelt](https://www.reichelt.com/ch/de/shop/produkt/nvidia_jetson_agx_orin_dev_kit_12-kern_cpu_64_gb_ddr5-383698) |

### Sensors
| Part | Link |
|------|------|
| Luxonis OAK-D Lite Auto Focus | [Mouser](https://www.mouser.ch/ProductDetail/Luxonis/OAK-D-Lite-AF) |
| ReSpeaker 2-Mic HAT | [Reichelt](https://www.reichelt.com/de/de/shop/produkt/respeaker_2-mic_hat_fuer_raspberry_pi-248718) |

### Drive System
| Part | Link |
|------|------|
| Stock R2-D2 Motors | Reuse DeAgostini |
| Pololu Dual MC33926 | [Pololu](https://www.pololu.com/product/2995) |

### Power
| Part | Specs | Link |
|------|--------|------|
| 4× Turnigy 2200mAh 4S 60C LiPo | 14.8V, ~32.56Wh ea., 107×35×36mm, 255g | [HobbyKing](https://hobbyking.com/) |
| ISDT 608AC Charger | 50W AC/200W DC, 8A | [AliExpress](https://de.aliexpress.com/item/1005007512739386.html) |

![Motor Wiring Example](docs/photos/motor_wiring.jpg)  <!-- Add if you have one -->



## 3. Software Stack

**Why this Stack?** Optimized for Jetson edge-AI; leverages NVIDIA/ROS2 pre-builts for fast integration (fits ≤50 lines custom code goal).

### Base System
- **OS:** [JetPack 6.x](https://docs.nvidia.com/jetson/jetpack/index.html) (Ubuntu 22.04).
- **ROS2:** Humble ([Dockerized](https://docs.ros.org/en/humble/Installation.html) for easy Jetson deploys).

### Navigation & Mapping
- **SLAM:** [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) [](https://navigation.ros.org/).
- **Nav:** Nav2 for planning/avoidance [](https://navigation.ros.org/tutorials/docs/index.html).

### Perception
- **Visual AI:** [NVIDIA Isaac ROS](https://github.com/NVIDIA-ISAAC-ROS) (cuVSLAM, YOLOv8, FoundationPose).
- **Processing:** OpenCV + [jetson-inference](https://github.com/dusty-nv/jetson-inference).

### Interaction
- **Speech:** [ros2_speech_recognition](https://github.com/Roboy/ros2_speech_recognition).
- **TTS:** gTTS or [jetson-voice](https://github.com/dusty-nv/jetson-voice).
- **LLM:** [llama.cpp](https://github.com/ggerganov/llama.cpp) + [llama_ros](https://github.com/mgonzs13/llama_ros) <a href="https://jetson-ai-lab.com/ollama.html" target="_blank" rel="noopener noreferrer nofollow"></a>.

### Control
- **Actuators:** [ros2_control](https://control.ros.org/) for motors/servos.

### Integration Strategy
- **Pre-Builts:** Launch files from [Waveshare UGV](https://www.waveshare.com/wiki/ROS2_based_ROS_Car_Kit), [Isaac ROS Examples](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common), [Yahboom ROSMASTER](https://category.yahboom.net/blogs/news), [Stereolabs ZED](https://www.stereolabs.com/docs/ros2/).
- **Custom:** YAML params + tiny Python nodes (≤50 lines total).



## Repo Layout

```text
.
├─ docs/photos/          # Build progress pics (e.g., empty shell)
├─ hardware/
│   ├─ wiring/           # Schematics (Pololu MC33926)
│   └─ cad/              # 3D prints (gripper STEP/STL)
├─ src/                  # ROS2 packages
│   ├─ r2d2_description/ # URDF (48cm model)
│   ├─ r2d2_navigation/  # Nav2 + slam_toolbox launches
│   ├─ r2d2_perception/  # YOLO + ReMEmbR
│   └─ r2d2_llm/         # grok_fallback.py + memory DB
├─ docker/               # Dockerfile.humble + compose
├─ .gitignore            # Ignore API keys
├─ LICENSE               # MIT (code + hardware)
└─ README.md

````

---

### Section: Status (November 16, 2025)

| Component | Progress | Notes |
|-----------|----------|-------|
| ...       | ...      | ...   |


## Community & Contributing

- [@s_leuenberger](https://x.com/s_leuenberger) | Switzerland | [R2 Builders Club](https://r2builders.club/) (DeAgostini mods welcome!).
- **Contribute:** Fork, PR for launch tweaks. Report issues: [New Issue](https://github.com/severinleuenberger/R2D2-as-real-AI-companion/issues).
- **License:** [MIT](LICENSE) – Free to copy/modify/distribute (code + CAD).

