# R2-D2 as a Real AI Companion  
Transforming the 1:2 DeAgostini R2-D2 model into a fully autonomous, indoor AI companion robot powered by an NVIDIA Jetson AGX Orin and ROS 2 Humble.

![R2D2 Full Robot](docs/photos/20251107_105518.jpg)

This project aims to rebuild the iconic R2-D2 as a modern, ROS-based AI robot capable of perception, navigation, speech, memory, and real-time interaction.  
Development takes place directly on the Jetson AGX Orin using a clean ROS 2 workspace, custom packages, and modular hardware integration.

The repository contains:

- the full ROS 2 workspace (`ros2_ws`)
- bringup system and first functional nodes (heartbeat + beep)
- hardware documentation and build photos
- upcoming perception, navigation, and AI components

This is a long-term hobby project focused on learning, robotics, and open-source collaboration.

---

## 📚 Documentation

Complete setup guides and technical documentation for the R2D2 project:

### Internal Reference (For AI Agents & Developers)

**[00_INTERNAL_AGENT_NOTES.md](00_INTERNAL_AGENT_NOTES.md)**  
Quick reference guide for AI agents and future developers working on this project. Documents platform-specific patterns, ARM architecture quirks, performance baselines, environment setup order, debugging sequences, and institutional knowledge learned during development. Covers Jetson AGX Orin specifics, OAK-D camera integration details, expected performance metrics (12.8 Hz perception @ 132-136 brightness), and common issue solutions.

### User & Technical Documentation

1. **[01_R2D2_BASIC_SETUP_AND_FINDINGS.md](01_R2D2_BASIC_SETUP_AND_FINDINGS.md)**  
   Initial Jetson AGX Orin setup, ROS 2 Humble installation, workspace configuration, and first functional tests (heartbeat/beep nodes).

2. **[02_CAMERA_SETUP_DOCUMENTATION.md](02_CAMERA_SETUP_DOCUMENTATION.md)**  
   OAK-D Lite camera integration with ROS 2, DepthAI Python SDK setup, camera node implementation, frame capture, and topic publishing.

3. **[03_PERCEPTION_SETUP_DOCUMENTATION.md](03_PERCEPTION_SETUP_DOCUMENTATION.md)**  
   Real-time image processing pipeline with brightness metrics, downscaling (1920×1080 → 640×360), grayscale conversion, integrated launch system, and validated behavior tests.

### Face Recognition System

4. **[06_FACE_RECOGNITION_TRAINING_AND_STATUS.md](06_FACE_RECOGNITION_TRAINING_AND_STATUS.md)**  
   Complete face recognition training workflow, status monitoring, and LED integration guide. Covers training data collection (4-stage interactive system), model training with LBPH, real-time status reporting via JSON file, and LED integration examples for GPIO/HTTP control.

5. **[COMPUTE_COST_ANALYSIS.md](COMPUTE_COST_ANALYSIS.md)**  
   Detailed analysis of face recognition service CPU usage and performance characteristics. Includes measured data: face detection (1.69 ms), face recognition (18.07 ms), total pipeline cost (~16 ms per frame), CPU usage at different settings (10-15% at default), and scaling analysis for multiple people recognition.

---

## Project Status (as of December 2025)

The project is currently in **Phase 1: Core System Bringup**.  
The NVIDIA Jetson AGX Orin (64 GB) is fully operational and runs a clean, modern ROS 2 Humble setup with a professional workspace structure.

![R2D2 Empty Body](docs/photos/empty_body.jpg)

### ✔️ Completed so far

- Jetson AGX Orin successfully flashed with JetPack 6.x  
- Headless development workflow via VS Code Remote SSH  
- Clean ROS 2 workspace at `~/dev/r2d2/ros2_ws`  
- Two fully functional custom ROS 2 packages:
  - `r2d2_hello` → first minimal nodes  
    - `beep_node`: timer-based alive signal  
    - `heartbeat_node`: publishes `/r2d2/heartbeat`  
  - `r2d2_bringup` → unified bringup launch  
- Launch file to start both nodes:
  ```bash
  ros2 launch r2d2_bringup bringup.launch.py
  ```


- Hardware photo documentation synchronized with GitHub

- Repository reset, cleaned, and realigned to current architecture

## 🔧 Next steps (short-term roadmap)

- Touch-the-ground hardware tests:
  - GPU compute validation (CUDA / Python CuPy test)
  - Audio out (Jetson → speaker)
  - Audio in (microphone recording test)
  - Camera test (single-frame capture)

- Add placeholder packages for:
  - r2d2_description (URDF/Xacro)
  - r2d2_perception (OAK-D)
  - r2d2_navigation (Nav2 + SLAM)
  - Add a system health/status ROS 2 node

The goal of this phase is to have a fully verified hardware/software baseline
before adding perception, navigation, speech, or AI.



## Hardware Build Progress

The 1:2 DeAgostini R2-D2 body has been fully opened, stripped, and prepared for the integration of modern electronics.  
The Jetson AGX Orin is already mounted inside the body as the central compute unit.

![R2D2 Body With Jetson](docs/photos/body%20with%20jetson.jpg)

The internal structure was reinforced and cable pathways were cleaned up to support upcoming power distribution, audio components, motor drivers, and the depth camera system.

A secondary angle of the current internal layout:

![R2D2 Body With Jetson - Angle 2](docs/photos/body%20with%20jetson%202.jpg)

Future hardware integrations will include:

- Motor drivers for dome and leg motors  
- LiPo battery system with DC-DC regulation  
- ReSpeaker / microphone array  
- OAK-D Lite (or Pro) depth camera  
- Internal cooling & airflow improvements  
- Power distribution board and safety electronics  

These photos will be updated as the internal structure evolves and new components are mounted.



## Repository Structure

The repository reflects the active development environment on the Jetson AGX Orin.  
Generated ROS 2 build artifacts are excluded via `.gitignore`, resulting in a clean and minimal source tree.

```text
.
├─ ros2_ws/
│  ├─ src/
│  │  ├─ r2d2_hello/      # First functional nodes (beep + heartbeat)
│  │  └─ r2d2_bringup/    # Bringup launch to start the robot system
│  ├─ build/              # (generated, ignored)
│  ├─ install/            # (generated, ignored)
│  └─ log/                # (generated, ignored)
├─ docs/
│  └─ photos/             # Build documentation images
├─ tests/                 # GPU/Audio/Camera “touch-the-ground” tests (planned)
└─ scripts/               # Utility scripts
```



## Quick Start (Current State)

### Clone the project

```bash
git clone git@github.com:severinleuenberger/R2D2-as-real-AI-companion.git
cd R2D2-as-real-AI-companion
```

### Build the ROS 2 workspace
```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Start the core system
```bash

ros2 launch r2d2_bringup bringup.launch.py
```

### Observe the heartbeat
```bash
ros2 topic echo /r2d2/heartbeat
```

These commands reflect the current minimum viable robot system.
Additional nodes, packages, and hardware integrations will be added incrementally as the project evolves.

 

## Bill of Materials (BOM)

| Qty | Item | Model / Part Number | Purpose | Approx. Price (USD) | Link / Source | Status |
|-----|---------------------------------------------|---------------------|--------|---------------------|---------------|--------|
| 1   | DeAgostini R2-D2 1:2 Kit | Complete 100-issue set | Main body, legs, dome, panels | ~1,385  | eBay / RPF Forums | got it |
| 1   | NVIDIA Jetson AGX Orin 64 GB | 945-13730-0005-000 | Main AI brain (ROS2 + Grok fallback) | 1,999 | NVIDIA / Amazon | ordered |
| 1   | OAK-D Lite depth camera | Luxonis OAK-D-Lite | SLAM, person recognition, obstacle avoidance | 149 | Luxonis Store | ordered |
| 1   | ReSpeaker 4-Mic Array for Raspberry Pi | Seeed Studio | Voice input for LLM node | 30 | Seeed / Amazon | ordered |
| 2   | Pololu Dual MC33926 Motor Driver | #2135 | Drives stock DeAgostini DC motors | 20 × 2 = 40 | Pololu | got it |
| 2   | Stock DeAgostini DC motors + gearboxes | Original leg motors | Locomotion (2-wheel diff-drive) | Included in kit | — | got it |
| 1   | LiPo battery 4S 22.2 V 5000 mAh | Turnigy / HobbyKing | Main power | 40–60 | HobbyKing | got it |
| 1   | DC-DC buck converter 14 V → 12 V / 5 V | Various | Powers Jetson, ReSpeaker, motors | 10 | Amazon / AliExpress | ? |
| 1   | IMU (in OAK-D Lite) | BMI270 + BMM150 | Used by robot_localization EKF | Included | — | ? |

**Total estimated cost (without DeAgostini kit):** ~2,200 USD  
**Total with full DeAgostini kit:** ~3,600 USD


## 2. Hardware Components

### Base Model
| Spec | Value |
|------|-------|
| DeAgostini 1:2-Scale R2-D2 Kit | [Buy the kit or some magazines out of it](https://www.fanhome.com/us/star-wars/r2d2-build-up) |
| Height | 48 cm |
| Width | 28 cm |
| External Ø | 20 cm |
| Internal Volume | 4.5–7.2 L |
| Reuse | Drive/arms/dome, LED |
| Cost | 300–700 CHF |

### Compute
| Part | Specs | Link |
|------|--------|------|
| NVIDIA Jetson AGX Orin 64GB Dev Kit | 100×87×47 mm, 15–60 W | [Reichelt](https://www.reichelt.com/ch/de/shop/produkt/nvidia_jetson_agx_orin_dev_kit_12-kern_cpu_64_gb_ddr5-383698) |

### Sensors
| Part | Link |
|------|------|
| Luxonis OAK-D Lite Auto Focus | [Mouser](https://mou.sr/4aaEfYZ) |
| ReSpeaker 2-Mic HAT | [Mouser](https://mou.sr/49B7DHD) |

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

---



## Community & Contributing
- **Discussions:** https://forums.developer.nvidia.com/t/ros2-humble-r2-d2-ai-companion-robot-jetson-agx-orin-64gb-deagostini-build-open-repo-hardware-schematics/351685
- [@s_leuenberger](https://x.com/s_leuenberger) | Switzerland |
- **Contribute:** Fork, PR for launch tweaks. Report issues: [New Issue](https://github.com/severinleuenberger/R2D2-as-real-AI-companion/issues).
- **License:** [MIT](LICENSE) – Free to copy/modify/distribute (code + CAD).

