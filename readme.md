\# AI R2-D2 Companion Bot





\*\*Goal:\*\* Convert a DeAgostini 1:2-Scale R2-D2 model into a fully autonomous AI companion bot capable of indoor navigation, person recognition, natural conversation (local LLM with API fallback), and simple object manipulation tasks (e.g., fetch and carry).



\*\*Core Platform:\*\*  

\- \*\*Compute:\*\* NVIDIA Jetson AGX Orin 64GB (Edge-AI)  

\- \*\*Robotics Framework:\*\* ROS2  

\- \*\*Perception:\*\* Depth + audio sensors  



\*\*Key Assumptions:\*\*  

\- Scale: 1:2 (48 cm tall)  

\- Development Focus: 90% pre-built packages/launches; ≤10–50 lines custom code  

\- Use \*\*only\*\* the specified components, libraries, and pre-existing tutorials. Prioritize integration speed, modularity, and reliability.  

\- \*\*Scope:\*\* Hardware modification, software integration, feature implementation  

\- \*\*Reference Community:\*\* R2 Builders Club (mod tips)  



---



\## Quick Start (Dockerized)



```bash

docker compose up --build

ros2 launch r2d2\_navigation nav\_launch.py   # SLAM + Nav2

ros2 launch r2d2\_llm tts\_stt\_launch.py      # jetson-voice + llama\_ros + Grok fallback



1\. Features and Requirements

1.1 Intelligent Speech with Fallback Logic



Pipeline:Speech → Text → Local LLM → \[Confidence < 70%] → xAI Grok-4 API → Text → Speech/Action

Local LLM: Llama-3-8B via llama.cpp + llama\_ros

Fallback Trigger: Logprobs < 0.7 → auto-switch to Grok-4 API

Implementation:jetson-voice + llama\_ros + custom requests ROS2 node (logprob evaluation)



1.2 Person Recognition \& Memory



Face Detection: Isaac ROS YOLO or dusty-nv/jetson-inference

Identity Persistence: Embeddings via NVIDIA ReMEmbR + SQLite DB

Memory Management: Voice command to delete personal data (DB row deletion)



1.3 Contextual Conversation



Speech I/O:Roboy/ros2\_speech\_recognition (GitHub)

Speaker Association: Link audio + visual detection to conversation thread

Storage: Per-person dialogue history in SQLite



1.4 Autonomous Navigation \& Mapping



SLAM:slam\_toolbox (GitHub: SteveMacenski/slam\_toolbox)

Localization: AMCL (Nav2)

Path Planning: Nav2 stack

Reference: Waveshare UGV Rover Tutorial (pre-built launch)



1.5 Multi-Room \& Semantic Mapping



Multi-Map Merging:slam\_toolbox multi-session support

Semantic Layer: Isaac ROS Segmentation + dusty-nv/jetson-inference

Object Memory: Store labeled objects with pose in map



1.6 Object Manipulation



Arm Control:ros2\_control

Pick-and-Place: Adapted from JuoTungChen/ROS2\_pick\_and\_place\_UR5 (GitHub)

Gripper: Small servo-based gripper (custom mount)





2\. Hardware Components

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







3\. Software Stack

Base System



OS: JetPack SDK v6.x (Ubuntu 22.04)

ROS2 Distro: Humble (Dockerized for modularity)

Docs:NVIDIA JetPack Docs



Navigation \& Mapping



SLAM:slam\_toolbox

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



Speech Recognition:ros2\_speech\_recognition (Roboy)

TTS: gTTS or jetson-voice

Local LLM:llama.cpp + llama\_ros

Tutorial: Jetson AI Lab Ollama





Control



Actuators:ros2\_control (motors, servos)

Docs:ROS2 Control Docs



Integration Strategy



Use pre-built launch files from:

Waveshare UGV Rover

NVIDIA Isaac ROS examples

Yahboom ROSMASTER Tutorial

Stereolabs ROS2 Tutorial



Custom Code: YAML config tweaks + small Python nodes (≤50 lines)





Repo Layout

text.

├─ docs/photos/          # Build pics

├─ hardware/

│   ├─ wiring/           # Pololu MC33926 schematic

│   └─ cad/              # Gripper mount STEP/STL

├─ src/

│   ├─ r2d2\_description/ # URDF (48 cm)

│   ├─ r2d2\_navigation/

│   ├─ r2d2\_perception/

│   └─ r2d2\_llm/         # grok\_fallback.py

├─ docker/

│   └─ Dockerfile.humble

├─ .gitignore

├─ LICENSE               # MIT

└─ README.md



Status (November 15, 2025)





MilestoneStatusRepo + README + BOM✅Photos \& wiring✅URDF base⏳Drive system⏳SLAM (slam\_toolbox)⏳TTS/STT⏳LLM + Grok fallback⏳Pick-and-place⏳



Community

@s\_leuenberger | Switzerland 

MIT License – copy, modify, distribute freely.

