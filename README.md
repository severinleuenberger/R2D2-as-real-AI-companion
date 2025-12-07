# R2D2 as a Real AI Companion

> Transform the iconic 1:2 DeAgostini R2-D2 into a fully autonomous AI companion robot with vision, voice, navigation, and personality—powered by NVIDIA Jetson AGX Orin 64GB and ROS 2 Humble.

![R2D2 Full Robot](docs/photos/20251107_105518.jpg)

**Current Status:** Phase 1 Core System — ~85% complete  
**Next Phase:** Speech & Language (STT + LLM + TTS)  
**Project Timeline:** 4 phases, 280-300 hours total (7 weeks full-time, 3-4 months part-time)

---

## 🎯 Core Objectives

The R2D2 project has 16 fundamental goals that define what the robot must accomplish:

### Conversation & Language
- **Natural Voice Interaction:** Listen to spoken input, convert to text, understand meaning, generate contextual responses, and speak back with natural-sounding voice
- **Multi-Turn Conversation:** Support continuous dialogue without manual resets or repeated wake-word calls
- **Local AI Processing:** All speech-to-text, language model, and text-to-speech run locally without cloud services (privacy + offline capability)

### Perception & Recognition
- **Face Recognition:** Detect and identify people, especially primary user (Severin)
- **Person Tracking:** Track identified individuals and optionally follow them autonomously
- **Object Detection:** Recognize obstacles, furniture, and items using camera and depth sensors
- **Environment Understanding:** Recognize rooms and understand spatial context (kitchen, bedroom, office, etc.)
- **Situation Awareness:** Detect environmental events (person entering room, name being called, obstacles appearing)

### Autonomous Navigation
- **Indoor Navigation:** Move autonomously through multi-room environments without human guidance
- **Mapping & Localization:** Build and maintain maps, know current position, recognize revisited areas
- **Obstacle Avoidance:** Detect and avoid collisions with furniture, people, pets, and other obstacles
- **Safe Movement:** Maintain appropriate speeds and stop distances to prevent accidents
- **Location-Based Commands:** Respond to verbal commands like "go to kitchen" or "come here"

### Expression & Interaction
- **Expressive Audio:** Generate R2-D2-like beep sounds and emotional vocalizations
- **Social Responsiveness:** React to environmental events and social cues in real-time
- **Directional Awareness:** Orient camera and movement toward person speaking or object of interest
- **Multi-Modal Interaction:** Simultaneously perceive, navigate, and converse while maintaining social engagement
- **Heartbeat Signal:** Provide continuous "alive" indicator that system is operational
- **Sound Playback:** Play audio files including R2-D2 sound effects and music

### Verbal Command Interface
- **Direct Commands:** Accept and execute commands like "follow me," "look at this," "go to the living room"
- **Natural Language:** Understand conversational requests, not just rigid command syntax
- **Context Awareness:** Remember conversation history and current physical context

---

---

## 🚀 Features (Current & Planned)

### Phase 1: Core System (Current) ✅
- [x] **Real-time Perception:** 30 FPS RGB camera stream, brightness metrics, face detection (90% accuracy)
- [x] **Face Recognition:** LBPH-based person identification (trained for multiple users)
- [x] **Hardware Integration:** OAK-D Lite depth camera, Jetson AGX Orin compute, ROS 2 infrastructure
- [x] **Professional Codebase:** Clean workspace structure, modular packages, parameter-driven configuration
- [x] **Comprehensive Docs:** Setup guides, integration patterns, operations checklist, architecture diagrams

### Phase 2: Speech & AI (Next) ⏳
- [ ] **Speech-to-Text:** Real-time audio input with wake-word detection ("Hey R2D2")
- [ ] **Language Model:** Local LLM inference (Llama 2 7B) for conversational responses
- [ ] **Text-to-Speech:** Real-time voice synthesis and playback
- [ ] **Context Awareness:** Vision-informed responses (greet by name, reference visual context)

### Phase 3: Navigation (Future) ⏳
- [ ] **SLAM Mapping:** Autonomous room mapping and localization
- [ ] **Autonomous Movement:** Differential drive control for 2-wheel locomotion
- [ ] **Obstacle Avoidance:** Real-time collision detection and path replanning
- [ ] **Room Navigation:** Go to named rooms, return to base, explore autonomously

### Phase 4: Memory & Personality (Future) ⏳
- [ ] **Conversation Memory:** Persistent history, context-aware responses
- [ ] **Learning & Adaptation:** Preference learning, anomaly detection
- [ ] **Expression:** LED animations, motor movements, tone variation
- [ ] **Multi-User Support:** Per-user profiles, personalized interactions

---

## 📖 Documentation

Comprehensive guides organized by audience and use case. **Start here:**

### For First-Time Users
1. **[Quick Start](#quick-start)** (below) — Run the system in 5 minutes
2. **[PROJECT_GOALS.md](PROJECT_GOALS.md)** — Understand the 4-phase roadmap and success metrics
3. **[ARCHITECTURE_OVERVIEW.md](ARCHITECTURE_OVERVIEW.md)** — See how components fit together

### For Daily Operators
- **[OPERATIONS_CHECKLIST.md](OPERATIONS_CHECKLIST.md)** — Startup, monitoring, troubleshooting, recovery procedures
- **[COMPUTE_COST_ANALYSIS.md](COMPUTE_COST_ANALYSIS.md)** — CPU/memory usage, performance baselines, optimization tips

### For Phase 2-4 Developers
- **[INTEGRATION_GUIDE.md](INTEGRATION_GUIDE.md)** — Step-by-step guide to add speech, navigation, memory systems
- **[00_INTERNAL_AGENT_NOTES.md](00_INTERNAL_AGENT_NOTES.md)** — ARM architecture quirks, DepthAI setup, common issues and solutions

### Technical Depth (Phase 1 Subsystems)
| Component | Document | Purpose |
|-----------|----------|---------|
| **System Setup** | [01_BASIC_SETUP_AND_FINDINGS.md](01_R2D2_BASIC_SETUP_AND_FINDINGS.md) | Jetson flashing, ROS 2 installation, workspace setup |
| **Camera Integration** | [02_CAMERA_SETUP_DOCUMENTATION.md](02_CAMERA_SETUP_DOCUMENTATION.md) | OAK-D Lite + DepthAI SDK, ROS 2 camera_node |
| **Image Processing** | [03_PERCEPTION_SETUP_DOCUMENTATION.md](03_PERCEPTION_SETUP_DOCUMENTATION.md) | Brightness metrics, Haar Cascade face detection, pipeline |
| **Face Recognition** | [05_FACE_RECOGNITION_INTEGRATION.md](05_FACE_RECOGNITION_INTEGRATION.md) & [06_FACE_RECOGNITION_TRAINING_AND_STATUS.md](06_FACE_RECOGNITION_TRAINING_AND_STATUS.md) | LBPH training, real-time recognition, model management |
| **Backup & Restore** | [07_BACKUP_AND_RESTORE_SETUP.md](07_BACKUP_AND_RESTORE_SETUP.md) | Full-system backup for reproducible deployments |

---

## ✅ Phase 1 Status

**Completion:** ~85% (core systems operational, documentation polishing)

![R2D2 Empty Body](docs/photos/empty_body.jpg)

### What's Working
- ✅ **Jetson Setup:** JetPack 6.x, ROS 2 Humble, clean workspace structure
- ✅ **Camera:** OAK-D Lite streaming 30 FPS RGB (1920×1080) at `/oak/rgb/image_raw`
- ✅ **Perception:** Brightness metrics + Haar Cascade face detection (13 Hz) + LBPH recognition (6.5 Hz)
- ✅ **Node Architecture:** 4 ROS 2 packages with parameter-driven configuration
- ✅ **Performance:** ~10-15% CPU usage (perception pipeline), excellent thermal stability
- ✅ **Documentation:** 7+ technical guides + architecture diagrams + integration templates

### Remaining Phase 1 Tasks
- ⏳ **README Improvements** (this file — making it more accessible)
- ⏳ **PROJECT_GOALS.md** (complete roadmap + 4-phase vision)
- ⏳ Git commit of final documentation

### Phase 1 Exit Criteria
When all above are done:
- ✅ New developers can start camera stream in 5 minutes
- ✅ Architecture is clear (blocks, data flow, integration points)
- ✅ Existing documentation is audited and organized
- ✅ Path to Phase 2 (Speech) is documented
- **→ Ready to start Phase 2**



---

## 🔧 Hardware

The iconic R2-D2 is being rebuilt with modern robotics hardware and AI compute.

### Current Hardware Stack
| Component | Model | Purpose | Status |
|-----------|-------|---------|--------|
| **Chassis** | DeAgostini R2-D2 1:2 Kit | Main body (48 cm tall) | ✅ Complete |
| **Compute** | NVIDIA Jetson AGX Orin 64GB | AI brain (12-core ARM, 504-GPU cores, 100W TDP) | ✅ Mounted & Running |
| **Camera** | Luxonis OAK-D Lite Auto Focus | Vision (1920×1080 @ 30 FPS, depth + IMU) | ✅ Integrated |
| **Audio Input** | ReSpeaker 2-Mic HAT | Voice capture (for Phase 2 STT) | ⏳ Ordered |
| **Drive** | DeAgostini DC Motors (2×) | Leg & dome motors with encoders | ⏳ Not yet integrated |
| **Motor Control** | Pololu MC33926 (2×) | H-bridge drivers for DC motors | ✅ Assembled |
| **Power** | 4S LiPo 5000 mAh (14.8V) | Main battery system | ✅ Charged & Ready |
| **Power Dist** | Custom DC-DC (14V→12V/5V) | Jetson + ReSpeaker + motors | ⏳ Not yet integrated |
| **Internal** | WS2812B RGB LEDs | Status & personality expression | ⏳ Coming in Phase 4 |

### Inside the Bot
![R2D2 Body With Jetson](docs/photos/body%20with%20jetson.jpg)  
The internal structure houses the Jetson AGX Orin, OAK-D camera, power distribution, and future motor drivers. Careful cable management ensures room for Phase 2-4 additions.

### Bill of Materials (Full Project)
See [BOM_HARDWARE.md](BOM_HARDWARE.md) for detailed part numbers, sourcing links, and cost breakdown (~$3,600 including chassis).



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



## 🚀 Quick Start

### 1. Clone & Setup (5 min)

```bash
# Clone repository
git clone git@github.com:severinleuenberger/R2D2-as-real-AI-companion.git
cd R2D2-as-real-AI-companion

# Set up environment (CRITICAL: order matters!)
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8
source ~/.bashrc
cd ros2_ws
source install/setup.bash
```

### 2. Launch Camera & Perception (1 command)

```bash
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py
```

**Expected Output (first 5 seconds):**
```
[INFO] [camera_node]: OAK-D camera initialized
[INFO] [image_listener]: ImageListener node initialized
[INFO] [image_listener]: Haar Cascade loaded successfully
[INFO] All nodes started successfully
```

### 3. Verify System (In another terminal, same env setup as above)

```bash
# Check camera frame rate (should be ~30 Hz)
ros2 topic hz /oak/rgb/image_raw

# Check perception output (should be ~13 Hz)
ros2 topic hz /r2d2/perception/brightness

# See brightness values (should be ~130-140 in normal light)
watch -n 0.5 'ros2 topic echo /r2d2/perception/brightness -n 1'

# See face detection (0 = no one, 1+ = detected)
ros2 topic echo /r2d2/perception/face_count
```

### 4. Troubleshooting

If things don't work:
1. Check [OPERATIONS_CHECKLIST.md](OPERATIONS_CHECKLIST.md) → Section 5 (Troubleshooting)
2. Verify environment setup: `echo $OPENBLAS_CORETYPE` should print `ARMV8`
3. Check camera: `lsusb | grep Movidius`

**Time to working system:** ~7 seconds from launch command

---

## 📁 Repository Structure

```
r2d2/
├── README.md                              # This file
├── PROJECT_GOALS.md                       # 4-phase roadmap, success metrics, FAQs
├── ARCHITECTURE_OVERVIEW.md               # System design, data flow, integration patterns
├── OPERATIONS_CHECKLIST.md                # Daily startup, monitoring, troubleshooting
├── INTEGRATION_GUIDE.md                   # How to add Phase 2-4 features (template + examples)
├── 00_INTERNAL_AGENT_NOTES.md             # ARM quirks, DepthAI setup, performance baselines
├── 01_R2D2_BASIC_SETUP_AND_FINDINGS.md   # Jetson setup, ROS 2 installation
├── 02_CAMERA_SETUP_DOCUMENTATION.md      # OAK-D camera + DepthAI SDK
├── 03_PERCEPTION_SETUP_DOCUMENTATION.md  # Image processing pipeline
├── 05_FACE_RECOGNITION_INTEGRATION.md    # Face recognition system
├── 06_FACE_RECOGNITION_TRAINING_AND_STATUS.md # Training + status
├── 07_BACKUP_AND_RESTORE_SETUP.md        # Backup/restore procedures
├── COMPUTE_COST_ANALYSIS.md               # Performance profiles
│
├── ros2_ws/                               # ROS 2 Humble workspace
│   ├── src/
│   │   ├── r2d2_camera/                  # Camera node
│   │   ├── r2d2_perception/              # Perception node
│   │   ├── r2d2_hello/                   # Heartbeat + beep
│   │   └── r2d2_bringup/                 # Launch files
│   ├── build/ install/ log/              # (generated, gitignored)
│
├── data/
│   └── face_recognition/models/          # Trained LBPH models
│
├── tests/                                 # Component test scripts
├── docs/photos/                           # Build progress photos
└── scripts/                               # Utility scripts
```

---

## 🔗 Bill of Materials (BOM)

See [PROJECT_GOALS.md](PROJECT_GOALS.md) → "Resource Requirements" section for detailed hardware breakdown.

**Quick Summary:**
- **Total cost (compute + sensors):** ~$2,200
- **With DeAgostini kit:** ~$3,600
- **Key sources:** NVIDIA, Luxonis, HobbyKing, Pololu, Seeed Studio

---

## 🤝 Contributing & Community

**GitHub:** [severinleuenberger/R2D2-as-real-AI-companion](https://github.com/severinleuenberger/R2D2-as-real-AI-companion)

**Discussion:**
- [NVIDIA Developer Forums](https://forums.developer.nvidia.com/) (R2D2 thread)
- [GitHub Discussions](https://github.com/severinleuenberger/R2D2-as-real-AI-companion/discussions)

**How to Contribute:**
- **Found a bug?** → [Create an Issue](https://github.com/severinleuenberger/R2D2-as-real-AI-companion/issues)
- **Have an improvement?** → [Submit a Pull Request](https://github.com/severinleuenberger/R2D2-as-real-AI-companion/pulls)
- **Want to discuss?** → [Start a Discussion](https://github.com/severinleuenberger/R2D2-as-real-AI-companion/discussions)
- **Building your own R2D2?** → Document it and share!

**License:** [MIT](LICENSE) - Free to use, modify, and distribute

**Author:** [@s_leuenberger](https://x.com/s_leuenberger) | Switzerland

---

## 🎯 What's Next?

1. **This week:** Complete Phase 1 documentation
2. **Next week:** Begin Phase 2 (speech-to-text) prototype
3. **Next month:** Full speech + LLM pipeline
4. **Q2 2026:** Navigation and autonomous movement
5. **Q3 2026+:** Memory, personality, and deployment

See [PROJECT_GOALS.md](PROJECT_GOALS.md) for the complete roadmap and timeline.

---

**Happy building! Questions?** Check the [docs](README.md#-documentation) or [open an issue](https://github.com/severinleuenberger/R2D2-as-real-AI-companion/issues).

