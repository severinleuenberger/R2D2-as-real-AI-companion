# AI R2-D2 Companion Bot

![Build Photo Nov 7](docs/photos/20251107_105518.jpg)

**Goal:** DeAgostini 1:2 R2-D2 → autonomous AI companion: indoor nav, person recognition, conversation (local Llama-3-8B + Grok-4 fallback if conf <70%), fetch/carry.

**Core:** Jetson AGX Orin 64GB, ROS2 Humble, OAK-D Lite, ReSpeaker mic.  
**Assumptions:** 48cm; 90% pre-built; ≤50 lines custom; specified parts/tutorials only.

---

## Quick Start

```bash
docker compose up --build
ros2 launch r2d2_navigation nav_launch.py
ros2 launch r2d2_llm tts_stt_launch.py


1. Features and Requirements
1.1 Intelligent Speech with Fallback Logic

Pipeline:Speech → Text → Local LLM → [Confidence < 70%] → xAI Grok-4 API → Text → Speech/Action
Local LLM: Llama-3-8B via llama.cpp + llama_ros
Fallback Trigger: Logprobs < 0.7 → auto-switch to Grok-4 API
Implementation:jetson-voice + llama_ros + custom requests ROS2 node (logprob evaluation)

1.2 Person Recognition & Memory

Face Detection: Isaac ROS YOLO or dusty-nv/jetson-inference
Identity Persistence: Embeddings via NVIDIA ReMEmbR + SQLite DB
Memory Management: Voice command to delete personal data (DB row deletion)

1.3 Contextual Conversation

Speech I/O:Roboy/ros2_speech_recognition (GitHub)
Speaker Association: Link audio + visual detection to conversation thread
Storage: Per-person dialogue history in SQLite

1.4 Autonomous Navigation & Mapping

SLAM:slam_toolbox (GitHub: SteveMacenski/slam_toolbox)
Localization: AMCL (Nav2)
Path Planning: Nav2 stack
Reference: Waveshare UGV Rover Tutorial (pre-built launch)

1.5 Multi-Room & Semantic Mapping

Multi-Map Merging:slam_toolbox multi-session support
Semantic Layer: Isaac ROS Segmentation + dusty-nv/jetson-inference
Object Memory: Store labeled objects with pose in map

1.6 Object Manipulation

Arm Control:ros2_control
Pick-and-Place: Adapted from JuoTungChen/ROS2_pick_and_place_UR5 (GitHub)
Gripper: Small servo-based gripper (custom mount)


2. Hardware Components
Base Model

DeAgostini 1:2-Scale R2-D2 Kit
Height: 48 cm
Width: 28 cm
External Ø: 20 cm
Internal Volume: 4.5–7.2 L
Reuse: Drive system, arms, dome
Cost: 300–700 CHF


Compute

NVIDIA Jetson AGX Orin 64GB Dev Kit
Dimensions: 100×87×47 mm
Power: 15–60 W
Link


Sensors

Depth Camera: Luxonis OAK-D Lite Auto Focus
Link

Microphone Array: ReSpeaker 2-Mic HAT für Raspberry Pi
Link


Drive System

Motors: Stock R2-D2 motors
Controller Upgrade: Pololu Dual MC33926 Motor Driver
Link
Wiring Guide


Power

Battery: 4× Turnigy Heavy Duty 2200mAh 4S 60C LiPo
14.8V nominal, ~32.56Wh each, 107×35×36mm, ~255g

Charger: ISDT 608AC (50W AC / 200W DC, 8A, BattGo Smart)
Link



3. Software Stack
Base System

OS: JetPack SDK v6.x (Ubuntu 22.04)
ROS2 Distro: Humble (Dockerized for modularity)
Docs:NVIDIA JetPack Docs

Navigation & Mapping

SLAM:slam_toolbox
Navigation: Nav2 (path planning, obstacle avoidance)
Docs:Nav2 Docs

Perception

Visual AI: NVIDIA Isaac ROS
cuVSLAM
YOLOv8
FoundationPose
Docs

Image Processing: OpenCV
Inference Backend:dusty-nv/jetson-inference

Interaction

Speech Recognition:ros2_speech_recognition (Roboy)
TTS: gTTS or jetson-voice
Local LLM:llama.cpp + llama_ros
Tutorial: Jetson AI Lab Ollama


Control

Actuators:ros2_control (motors, servos)
Docs:ROS2 Control Docs

Integration Strategy

Use pre-built launch files from:
Waveshare UGV Rover
NVIDIA Isaac ROS examples
Yahboom ROSMASTER Tutorial
Stereolabs ROS2 Tutorial

Custom Code: YAML config tweaks + small Python nodes (≤50 lines)


Repo Layout

.
├─ docs/photos/          # Build pics
├─ hardware/
│   ├─ wiring/           # Pololu MC33926 schematic
│   └─ cad/              # Gripper mount STEP/STL
├─ src/
│   ├─ r2d2_description/ # URDF (48 cm)
│   ├─ r2d2_navigation/
│   ├─ r2d2_perception/
│   └─ r2d2_llm/         # grok_fallback.py
├─ docker/
│   └─ Dockerfile.humble
├─ .gitignore
├─ LICENSE               # MIT
└─ README.md


Status (November 15, 2025)


MilestoneStatusRepo + README + BOM✅Photos & wiring✅URDF base⏳Drive system⏳SLAM (slam_toolbox)⏳TTS/STT⏳LLM + Grok fallback⏳Pick-and-place⏳

Community
@s_leuenberger | Switzerland | R2 Builders Club
MIT License – copy, modify, distribute freely.
