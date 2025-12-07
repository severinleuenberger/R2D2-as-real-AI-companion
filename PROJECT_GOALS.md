# R2D2 Project Goals & Roadmap

**Project:** R2D2 as a Real AI Companion  
**Platform:** NVIDIA Jetson AGX Orin 64GB + ROS 2 Humble  
**Target:** Indoor autonomous AI robot with perception, navigation, speech, and memory  
**Status:** Phase 1 (Core System) – ~30% complete  
**Last Updated:** December 7, 2025

---

## Vision Statement

Transform the 1:2 scale DeAgostini R2-D2 model into a **fully autonomous indoor AI companion robot** capable of:
- Real-time perception (vision-based face detection/recognition, brightness sensing)
- Natural voice interaction (STT, LLM reasoning, TTS)
- Indoor autonomous navigation (SLAM, obstacle avoidance, room mapping)
- Persistent memory and personality (conversation history, user preferences)
- Real-time interaction with humans in a home environment

**Core Principle:** Learn robotics through an *end-to-end, open-source, hardware-in-the-loop project* with clean ROS 2 architecture, documented decision-making, and educational transparency.

---

## High-Level Architecture

```
LAYER 4: AI & Decision Making
├── Speech-to-Text (Phase 2)
├── Large Language Model (Phase 3)
├── Text-to-Speech (Phase 2)
└── Conversation Memory (Phase 4)

LAYER 3: Perception & Reasoning
├── Vision (face detection, recognition, object detection)
├── Depth/Obstacle Detection
└── Brightness & Environment Sensing

LAYER 2: Integration & Control
├── ROS 2 Node Orchestration
├── Parameter Configuration
├── Topic Pub/Sub Messaging
└── Launch System

LAYER 1: Hardware Interface
├── OAK-D Lite Camera (30 FPS RGB + Depth)
├── ReSpeaker Microphone Array
├── DC Motors (dome + legs)
├── LiPo Power System
└── Internal LEDs
```

---

## Phase-Based Development Plan

### Phase 1: Core System Bringup (30% complete) ✅ CURRENT
**Timeline:** Weeks 1-8  
**Goal:** Verified hardware/software baseline with clean ROS 2 workspace

#### 1.1 System Infrastructure (100% ✅)
- [x] Jetson AGX Orin flashed with JetPack 6.x
- [x] ROS 2 Humble clean installation
- [x] Professional ROS 2 workspace setup (`~/dev/r2d2/ros2_ws`)
- [x] Git repository initialization and organization
- [x] VS Code Remote SSH development workflow
- [x] Virtual environment for DepthAI SDK (isolated from system)

#### 1.2 Basic Node Infrastructure (100% ✅)
- [x] r2d2_hello package (beep + heartbeat nodes)
- [x] r2d2_bringup launch system
- [x] Parameter-driven configuration
- [x] Health monitoring (heartbeat signal)
- [x] Proper logging and error handling

#### 1.3 Hardware Interface Layer (85% ✅)
- [x] OAK-D Lite camera initialization (DepthAI SDK 2.31.0.0)
- [x] r2d2_camera package (camera_node with 30 FPS RGB streaming)
- [x] ROS 2 topic publishing (/oak/rgb/image_raw, 1920×1080 @ 30 Hz)
- [x] Camera calibration (auto, from OAK-D)
- [ ] Depth map integration (optional for Phase 2)

#### 1.4 Perception Pipeline (85% ✅)
- [x] r2d2_perception package (image_listener node)
- [x] Brightness metrics (/r2d2/perception/brightness, 13 Hz)
- [x] Haar Cascade face detection (90% accuracy, 13 Hz)
- [x] LBPH face recognition (trained for Severin, 85-92% accuracy, 6.5 Hz)
- [x] Downscaling (1920×1080 → 640×360) & grayscale conversion
- [x] Performance optimization (frame skip, CPU budget <15%)
- [ ] Face landmark detection (for expression recognition)
- [ ] Body pose estimation (skeleton + joint positions)

#### 1.5 Documentation & Knowledge Base (90% ✅)
- [x] Basic setup guide (01_BASIC_SETUP.md)
- [x] Camera integration docs (02_CAMERA_SETUP.md)
- [x] Perception setup docs (03_PERCEPTION_SETUP.md)
- [x] Face recognition guide (05, 06 docs)
- [x] Compute cost analysis (detailed performance metrics)
- [x] Architecture overview diagram + data flow
- [x] Operations checklist (daily startup, monitoring, troubleshooting)
- [x] Integration guide (how to add Phase 2-4 features)
- [ ] **TODO:** Project goals document (THIS FILE)
- [ ] **TODO:** Improved README (quick start + feature overview)

**Phase 1 Exit Criteria:**
- ✅ Jetson boots reliably with clean ROS 2 setup
- ✅ Camera streams at 30 FPS with <5% CPU overhead
- ✅ Face detection works at 13 Hz with <90% CPU total
- ✅ Face recognition trained and functional
- ✅ All documentation complete and tested
- ⏳ **Ready to start Phase 2 (Speech + LLM)**

---

### Phase 2: Speech & Language (0% complete) ⏳ NEXT
**Timeline:** Weeks 9-14  
**Goal:** Natural voice interaction with real-time speech-to-text and LLM responses

#### 2.1 Speech Input (STT)
- [ ] ReSpeaker 2-Mic HAT integration (audio capture, noise reduction)
- [ ] Speech-to-Text node (Whisper or similar, real-time)
- [ ] Hot-word detection ("Hey R2D2" or similar)
- [ ] Audio level monitoring and feedback

#### 2.2 Language Model (LLM)
- [ ] Lightweight LLM selection (Llama 2 7B or similar)
- [ ] Local LLM inference node (Ollama framework)
- [ ] Context-aware responses (remember recent conversation)
- [ ] Vision-to-text: integrate perception outputs (who they are, room state)

#### 2.3 Speech Output (TTS)
- [ ] Text-to-Speech node (local engine)
- [ ] Audio playback through Jetson speaker/amp
- [ ] Personality/voice selection
- [ ] Real-time speech synthesis (<500 ms latency)

#### 2.4 Integration & Testing
- [ ] Speech → LLM → TTS pipeline
- [ ] Wake word detection and response triggering
- [ ] Perception-aware responses (greet by name if recognized)
- [ ] Performance: <1 second response time
- [ ] CPU budget: <50% (leaving room for Phase 3-4)

**Phase 2 Exit Criteria:**
- ✅ "Hey R2D2" wake word triggers response
- ✅ Spoken question → understood and answered in <1 second
- ✅ System recognizes and greets by name (if trained)
- ✅ All speech nodes documented with examples
- ⏳ **Ready to start Phase 3 (Navigation)**

---

### Phase 3: Navigation & Autonomy (0% complete) ⏳ FUTURE
**Timeline:** Weeks 15-22  
**Goal:** Autonomous indoor movement with SLAM mapping and obstacle avoidance

#### 3.1 Motor Control
- [ ] DC motor driver integration (Pololu MC33926)
- [ ] Motor control node (PWM, encoders if available)
- [ ] Differential drive kinematics (2-wheel base model)
- [ ] Speed limiting and safety interlocks

#### 3.2 Localization & Mapping (SLAM)
- [ ] Nav2 stack integration
- [ ] Occupancy grid mapping (from OAK-D depth)
- [ ] Robot pose estimation (SLAM with visual odometry)
- [ ] Loop closure detection (remember visited areas)

#### 3.3 Path Planning & Obstacle Avoidance
- [ ] Global path planner (from start to goal)
- [ ] Local costmap for dynamic obstacles
- [ ] Real-time obstacle avoidance (DWA planner or similar)
- [ ] Safe-stop on detection of humans/pets

#### 3.4 Integration & Testing
- [ ] Map a 4-room house autonomously
- [ ] Autonomous room-to-room navigation
- [ ] Collision avoidance with humans
- [ ] Return-to-base when battery low
- [ ] Navigation-aware speech ("I'm rolling to the kitchen")

**Phase 3 Exit Criteria:**
- ✅ Successfully maps and navigates a home environment
- ✅ Safely avoids obstacles and humans
- ✅ Can navigate to named rooms ("Go to kitchen")
- ⏳ **Ready to start Phase 4 (Memory & Personality)**

---

### Phase 4: Memory & Personality (0% complete) ⏳ FUTURE
**Timeline:** Weeks 23-30+  
**Goal:** Persistent memory, learning, and personalized AI companion behavior

#### 4.1 Conversation Memory
- [ ] Persistent conversation database (SQLite or similar)
- [ ] Context-aware response generation (remember past interactions)
- [ ] User profile system (preferences, habits, history)
- [ ] Long-term memory consolidation

#### 4.2 Learning & Adaptation
- [ ] Reinforcement learning from human feedback
- [ ] Preference learning (favorite routes, music, conversation topics)
- [ ] Anomaly detection (unusual activity alerts)
- [ ] Predictive context ("It's 6 PM, probably dinner time")

#### 4.3 Personality & Expressiveness
- [ ] LED animation system (eyes, status indicators)
- [ ] Motor expressions (dome spin, leg movements for excitement)
- [ ] Tone variation in speech (question, surprise, empathy)
- [ ] Easter eggs and personality quirks (R2D2-like)

#### 4.4 Multi-User Support
- [ ] Face recognition for different users
- [ ] Per-user conversation history
- [ ] Per-user preferences and settings
- [ ] Multi-user conflict resolution (whose preference wins)

**Phase 4 Exit Criteria:**
- ✅ Remembers past conversations and references them naturally
- ✅ Learns and adapts to user preferences
- ✅ Expresses personality through motion and tone
- ✅ Multi-user support with personalized responses
- ⏳ **Ready for extended deployment and testing**

---

## Current Task Breakdown (as of Dec 7, 2025)

### Immediate (This Week)
- [x] Complete Architecture Overview documentation
- [x] Complete Operations Checklist
- [x] Complete Integration Guide for Phase 2-4 developers
- [ ] **Improve README.md** (quick start, feature highlights, structure)
- [ ] **Complete PROJECT_GOALS.md** (this file)
- [ ] Commit all documentation to git

### Short-term (Next 2 Weeks)
- [ ] Begin Phase 2 speech-to-text implementation
- [ ] Select and test Whisper STT engine
- [ ] ReSpeaker audio input integration
- [ ] Create speech_node package skeleton
- [ ] Initial testing with live audio

### Medium-term (Next Month)
- [ ] Complete speech pipeline (STT + TTS)
- [ ] Integrate local LLM (Ollama + Llama 2 7B)
- [ ] First end-to-end voice conversation
- [ ] Performance optimization (response time <1 sec)
- [ ] User feedback and iteration

---

## Success Metrics by Phase

| Phase | Metric | Target | Current | Status |
|-------|--------|--------|---------|--------|
| **1** | Jetson uptime | >99% | 99.5% | ✅ |
| **1** | Camera frame rate | 30 Hz | 30 Hz ±1 | ✅ |
| **1** | CPU usage (perception) | <15% | 10-15% | ✅ |
| **1** | Face detection accuracy | >85% | ~90% | ✅ |
| **1** | Documentation completeness | 100% | ~90% | ⏳ |
| **2** | STT latency | <500 ms | Not started | ⏳ |
| **2** | LLM response time | <1 sec | Not started | ⏳ |
| **2** | Wake word accuracy | >95% | Not started | ⏳ |
| **3** | SLAM map accuracy | <10cm error | Not started | ⏳ |
| **3** | Navigation success rate | >95% | Not started | ⏳ |
| **4** | Memory recall accuracy | >90% | Not started | ⏳ |
| **4** | User satisfaction | Subjective | Not measured | ⏳ |

---

## Risk Assessment & Mitigation

### Technical Risks

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|-----------|
| Jetson thermal throttling under load | Medium | High | Add heatsinks, monitor temps, profile code |
| Phase 2 LLM too slow for real-time | Medium | High | Start with small model (7B), optimize inference |
| SLAM failure in low-texture rooms | Medium | Medium | Rely on depth + visual odometry, add markers if needed |
| Speech recognition accuracy in noise | High | Medium | Use directional mic array (ReSpeaker), noise filtering |
| Battery life insufficient for navigation | Medium | Medium | Profile power draw, optimize motor control |

### Project Risks

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|-----------|
| Scope creep (too many features) | High | High | Stick to 4-phase plan, mark as "nice-to-have" |
| Hardware availability (chip shortages) | Low | Medium | Have backup part sources, reusable modules |
| Documentation lag | Medium | Low | Document as you code, use automated doc generation |
| Community interest/contributions | Medium | Low | Promote on forums, make it easy to contribute |

---

## Resource Requirements

### Hardware (Already Acquired)
- ✅ NVIDIA Jetson AGX Orin 64GB (compute core)
- ✅ Luxonis OAK-D Lite (camera)
- ✅ DeAgostini R2-D2 1:2 kit (chassis)
- ✅ Pololu MC33926 motor drivers (2×)
- ✅ DC motors (from kit)
- ✅ LiPo battery (4S 5000 mAh)

### Hardware (Needed)
- ⏳ ReSpeaker 2-Mic HAT or array (STT input)
- ⏳ Audio amplifier + speaker (TTS output)
- ⏳ USB hub or additional USB port (future sensors)

### Software Stack (Current)
- ✅ Ubuntu 22.04 Jammy
- ✅ ROS 2 Humble
- ✅ Python 3.10.6
- ✅ DepthAI SDK 2.31.0.0
- ✅ OpenCV + opencv-contrib

### Software Stack (Phase 2-4 Needed)
- ⏳ Whisper or similar (STT)
- ⏳ Ollama + Llama 2 7B (LLM)
- ⏳ TTS engine (text-to-speech)
- ⏳ Nav2 stack (navigation)
- ⏳ SQLite or similar (conversation memory)

### Development Time Estimate
- Phase 1: ~40 hours (mostly complete)
- Phase 2: ~60 hours (speech + LLM)
- Phase 3: ~80 hours (navigation + SLAM)
- Phase 4: ~100+ hours (memory + personality)
- **Total: ~280-300 hours** (equivalent to ~7 weeks full-time or 3-4 months part-time)

---

## Learning Objectives

This project serves as a comprehensive learning platform for:

1. **ROS 2 Architecture**
   - Package structure and best practices
   - Node design patterns (subscribers, publishers, services)
   - Launch file configuration and parameter passing
   - Integration of third-party libraries

2. **Embedded Linux & ARM Architecture**
   - NVIDIA Jetson AGX Orin specifics
   - ARM CPU vs x86 differences
   - System profiling and optimization
   - Thermal management and power budgeting

3. **Computer Vision**
   - Real-time image processing
   - Object detection (Haar Cascade, CNN)
   - Face recognition (LBPH, modern methods)
   - Depth estimation and 3D vision

4. **Robotics & Control**
   - Motor control and PWM
   - Differential drive kinematics
   - SLAM and visual odometry
   - Path planning and obstacle avoidance

5. **AI/ML Systems**
   - Local LLM inference and optimization
   - Speech-to-text and text-to-speech
   - Conversation context management
   - Reinforcement learning basics

6. **Software Engineering**
   - Version control and git workflows
   - Documentation and knowledge management
   - Testing and validation strategies
   - Performance profiling and optimization

---

## Comparison to Similar Projects

| Project | Hardware | Software | Status | Key Difference |
|---------|----------|----------|--------|-----------------|
| R2D2 (This) | Jetson AGX Orin | ROS 2 + Local LLM | Phase 1 | Hobby-scale, fully documented, open-source |
| Spot (Boston Dynamics) | Custom quadruped | Proprietary | Commercial | Enterprise-scale, $150k+ |
| Turtlebot 4 | Small mobile robot | ROS 2 + Nav2 | Commercial | Small form factor, less compute |
| Home Robot | Mobile manipulator | ROS 2 + ViLP | Research | Dexterous arm, active SLAM |

**Why this project is unique:**
- ✅ Iconic R2-D2 form factor (fan appeal)
- ✅ Powerful compute (64GB Jetson) for local AI
- ✅ 100% open-source and documented
- ✅ Hobby/educational scale (not commercial)
- ✅ Focus on learned robotics (not just integration)

---

## Communication & Community

### Discussion Channels
- **NVIDIA Developer Forums:** [ROS2 Humble R2D2 AI Companion](https://forums.developer.nvidia.com/) (active thread)
- **GitHub Issues:** Feature requests, bug reports, discussions
- **X/Twitter:** [@s_leuenberger](https://x.com/s_leuenberger) for updates

### How to Contribute
1. **Report Issues:** Found a bug? [Create an issue](https://github.com/severinleuenberger/R2D2-as-real-AI-companion/issues)
2. **Submit PRs:** Improvements to code/docs? [Open a pull request](https://github.com/severinleuenberger/R2D2-as-real-AI-companion/pulls)
3. **Discussions:** Ideas or questions? Start a discussion on GitHub
4. **Documentation:** Help improve guides and tutorials
5. **Hardware Variations:** Document your own R2D2 build with different hardware

### License & Reuse
- **Code License:** MIT (free to copy, modify, distribute)
- **Documentation License:** Creative Commons BY-SA 4.0
- **Hardware Designs (CAD/Schematics):** Open to contribute (coming in Phase 2-3)
- **Attribution:** Please credit this project if you build on it

---

## Frequently Asked Questions

**Q: Will this work with different hardware (different Jetson, camera, etc.)?**  
A: The architecture is modular. Swapping the camera to a Realsense or the Jetson to an Orin Nano is possible with minimal changes to the perception node. The integration guide covers this.

**Q: How long does it take to complete Phase 1?**  
A: ~40 hours including hardware setup, documentation, and testing. You can skip some documentation if you prefer to move faster.

**Q: Can I run this on a Jetson Orin Nano (cheaper)?**  
A: Yes, but with reduced performance. The Nano has 8GB RAM and 8 cores (vs AGX's 64GB and 12 cores). Phase 2-3 features may require optimization.

**Q: What's the total cost?**  
A: ~$3,600 including the DeAgostini kit. The compute + camera + electronics cost ~$2,200. The kit is optional (use any R2-D2 replica or custom chassis).

**Q: Is this AI-powered robot as smart as GPT-4?**  
A: No. Local LLMs (Llama 2 7B) are much smaller than GPT-4 (65B+). However, they're sufficient for conversational AI in a home setting and run entirely locally (no cloud, instant response, privacy).

**Q: Can I make my own version of this?**  
A: Yes! The code and documentation are open-source (MIT license). Fork the repo, adapt it to your hardware, and build your own R2D2 (or other robot chassis).

---

## Acknowledgments & References

### Open-Source Projects
- ROS 2 Humble (robotics middleware)
- DepthAI SDK (camera integration)
- OpenCV (computer vision)
- Ollama (local LLM inference)
- Nav2 (autonomous navigation)

### Communities & Resources
- NVIDIA Developer Community
- ROS Community (ros.org)
- Home Robotics forums
- Academic papers on SLAM, face recognition, etc.

### Inspiration
- Original R2-D2 (Star Wars)
- Boston Dynamics Spot
- Home robot projects (e.g., Home Robot, TurtleBot)
- AI/robotics researchers exploring embodied AI

---

## Next Steps

1. **This Week:** Finish documentation (README, PROJECT_GOALS)
2. **Next Week:** Begin Phase 2 prototype (speech input testing)
3. **Month 2:** Complete speech pipeline
4. **Month 3:** Begin navigation setup
5. **Month 4+:** Memory and personality systems

For the latest updates, check the [GitHub repository](https://github.com/severinleuenberger/R2D2-as-real-AI-companion).

---

*Project Goals & Roadmap created: December 7, 2025*
*Next Review: March 7, 2026 (end of Phase 1 + completion of Phase 2)*
