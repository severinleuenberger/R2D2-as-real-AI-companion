# Internal Agent Notes for R2D2 Project

**Purpose:** Quick reference guide for AI agents working on this R2D2 project during development and testing. This is FOR AGENTS, not for end users. Users read the 0XX-numbered docs; agents use THIS file to understand context, architecture, and how to work efficiently.

**Audience:** AI agents, developers who need quick context

**Workflow Stage:** From task start → until feature is tested and working

**After Testing Complete:** Use `000_AGENT_FINALIZATION_GUIDE.md` for verification, documentation, and deployment steps.

---

## 📋 Development Workflow Overview

```
START → Build & Test (this file) → TESTED & WORKING → Finalize & Deploy (000_AGENT_FINALIZATION_GUIDE.md) → DONE
```

**This file covers:**
- ✅ Phases 1-2: Development & Testing, Production Installation
- ✅ System architecture and environment
- ✅ Command references and troubleshooting
- ✅ Documentation standards

**For finalization (after testing), see:** `000_AGENT_FINALIZATION_GUIDE.md`
- ✅ Phases 3-5: Verification, Documentation, Git & Deployment
- ✅ Critical rules for pushing to repository
- ✅ Post-deployment monitoring

---

## 🎓 AI TUTOR SYSTEM (For Severin's Education)

**IMPORTANT:** The user (Severin) is learning robotics/ROS 2 while building this project. He is a senior BI developer but junior in embedded systems. **Explain your work as you do it!**

**Full Documentation:** See `300_AI_TUTOR.md` for comprehensive reference.

### Two Learning Modes

| Mode | Voice Trigger | What It Does |
|------|---------------|--------------|
| **Learning Mode** | "Turn on learning mode" | Starts narrator service - R2D2 speaks `coding_live.md` updates |
| **Tutor Mode** | "Be my tutor" | Activates teaching personality for interactive Q&A |

### Real-Time Narration Protocol (Learning Mode)

**After EVERY significant action, write to `~/dev/r2d2/data/coding_live.md`:**

This file is watched by R2D2's narrator service. When you update it, R2D2 will SPEAK the explanation aloud in real-time!

```markdown
## Current Action
[One line: what you just did]

## What This Means
[2-3 sentences explaining the concept]
[Use BI analogies - see table below]

## BI Analogy
[Direct comparison to a concept Severin knows]

## Category
[Category] > [Subcategory] | Understanding: [1-5]
```

**Example:**
```markdown
## Current Action
Editing audio_notification_node.py - adding jitter tolerance parameter

## What This Means
A STATE MACHINE tracks person recognition status through transitions.
The jitter tolerance prevents false "lost" alerts when you briefly look away.
Think of it like a debounce filter - wait 5 seconds before confirming loss.

## BI Analogy
Like a slowly-changing dimension (SCD Type 2) in a data warehouse -
we track state transitions over time with timestamps.

## Category
Architecture > State Machines | Understanding: 3
```

### When to Update coding_live.md

✅ **DO update** after:
- Editing a file (explain the change)
- Before running a command (explain what it does)
- Encountering an important new concept
- Completing a task (summarize what was learned)

❌ **DON'T update** for:
- Trivial changes (typos, formatting)
- Reading files (no action taken)
- Routine commands (cd, ls)

### BI-to-Robotics Concept Mapping

Use these analogies to explain robotics concepts:

| Robotics Concept | BI Analogy | Example in R2D2 |
|------------------|------------|-----------------|
| **ROS 2 Topic** | Database table / Event stream | `/r2d2/perception/person_id` is like a `person_events` table |
| **ROS 2 Subscriber** | SQL Trigger / ETL source | Face detection callback fires when new frame arrives |
| **ROS 2 Publisher** | INSERT statement / Event producer | Publishing brightness value = inserting a row |
| **ROS 2 Service** | Stored procedure / API call | `start_session` service = calling `sp_StartConversation` |
| **ROS 2 Node** | ETL Job / Microservice | `image_listener` node = an always-running ETL process |
| **Launch File** | Job scheduler / Orchestrator | Like SSIS package that starts multiple jobs |
| **Systemd Service** | SQL Agent Job / Scheduled task | Runs on boot, restarts on failure |
| **State Machine** | Workflow status / SCD Type 2 | Order status: pending→processing→shipped |
| **Callback Function** | Trigger procedure | Code that runs when event fires |
| **Message Queue** | Message broker / Event hub | Topics buffer messages between nodes |
| **Parameter** | Config table / Environment variable | Runtime settings without code changes |
| **Hz (frequency)** | Polling interval / Refresh rate | "13 Hz" = refreshes 13 times per second |
| **Latency** | Query response time | Time from input to output |

### In-Line Explanation Style

When explaining code changes, use this format in your responses:

```
LEARNING MOMENT: [Concept Name]
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

[Explanation using BI analogy]

BI Parallel: [Direct comparison]

Key Insight: [One sentence takeaway]
```

**Example:**
```
LEARNING MOMENT: ROS 2 Subscriber Callback
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

I'm creating a subscriber that listens to /r2d2/perception/person_id.
When a new message arrives, the callback function executes automatically.

BI Parallel: This is exactly like a SQL trigger - when a row is 
inserted into the person_events table, your trigger procedure fires.

Key Insight: Subscribers are event-driven, not polling. The system 
pushes data to you rather than you asking for it repeatedly.
```

### Learning Progress Tracking

Learning progress is tracked in SQLite: `~/dev/r2d2/data/persons.db`

Tables:
- `learning_topics` - What concepts have been encountered
- `learning_sessions` - Coding session summaries

When introducing a NEW concept, log it:
```python
# Agents should conceptually track:
# - Category (ROS 2, Python, Hardware, Architecture)
# - Subcategory (Subscribers, GPIO, State Machines)
# - Understanding level (1-5)
# - BI analogy used
```

### Narrator Modes

The user can control how much R2D2 talks:

| Mode | File Content | Behavior |
|------|--------------|----------|
| `narrator` | R2D2 speaks every update | Active learning |
| `quiet` | Silent, but context saved | Focus mode |
| `milestone` | Speaks on task completion | Less interruption |

Check mode: `cat ~/dev/r2d2/data/narrator_mode.txt`

### Interactive Follow-Up

After R2D2 speaks an explanation, Severin can:
1. Trigger speech (index finger gesture)
2. Ask "Explain that again" or "What's a state machine?"
3. R2D2 responds with more detail using the current context

The speech node reads `coding_live.md` for context, so R2D2 knows what you're working on!

### General Tutor Mode (Interactive Q&A)

Tutor Mode is for interactive teaching conversations (not coding narration):

**Activation:** Say "Be my tutor", "Teach me", or "I want to learn about X"

**What happens:**
- R2D2's personality shifts to patient teacher mode
- Uses BI analogies in explanations
- Checks understanding: "Does that make sense?"
- Builds concepts progressively

**Example:**
```
You: "Teach me about ROS 2 services"
R2D2: "A ROS 2 service is like a stored procedure in SQL. You call it,
       wait for execution, and get a result back. Unlike topics which are
       fire-and-forget, services are request-response. Does that make sense?"
```

**Deactivation:** Say "Normal mode", "Tutor off", or "Stop teaching"

**State file:** `~/dev/r2d2/data/tutor_mode_active.txt` (true/false)

---

## 🚀 BUILD & TEST PHASES (Complete Before Finalization)

### Phase 1: Development & Testing

**Goal:** Implement and verify feature works manually

- [ ] Feature implemented and tested manually
- [ ] All ROS 2 packages built successfully (`colcon build`)
- [ ] No linter errors or Python syntax errors
- [ ] Manual testing confirms functionality (`ros2 launch` or `ros2 run`)
- [ ] Code changes committed to git (but not pushed yet)

**Example:**
```bash
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_your_package
source install/setup.bash
ros2 launch r2d2_your_package your_launch.py  # Test manually
```

### Phase 2: Production Installation (SYSTEMD SERVICES)

**Goal:** Install service so it can run automatically

⚠️ **CRITICAL: DO NOT SKIP THIS PHASE**

A feature is NOT complete until it survives a reboot and works automatically.

**Common Mistake:** Testing with `ros2 launch` manually works, but service was never installed/enabled, so after reboot the system is broken.

**If your feature involves a systemd service, ALL of these are MANDATORY:**

- [ ] Service file created/updated in project root (`~/dev/r2d2/*.service`)
- [ ] **Service file copied to `/etc/systemd/system/`**
  ```bash
  sudo cp ~/dev/r2d2/r2d2-your-service.service /etc/systemd/system/
  ```
- [ ] **`systemctl daemon-reload` executed**
  ```bash
  sudo systemctl daemon-reload
  ```
- [ ] **`systemctl enable <service>` executed**
  ```bash
  sudo systemctl enable r2d2-your-service.service
  ```
- [ ] **`systemctl start <service>` executed**
  ```bash
  sudo systemctl start r2d2-your-service.service
  ```
- [ ] **Verify service is enabled:**
  ```bash
  systemctl is-enabled r2d2-your-service.service  # Must return: enabled
  ```
- [ ] **Verify service is running:**
  ```bash
  systemctl status r2d2-your-service.service  # Must show: active (running)
  ```

**Example:**
```bash
# Install and enable service
sudo cp ~/dev/r2d2/r2d2-your-service.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable r2d2-your-service.service
sudo systemctl start r2d2-your-service.service

# Verify
systemctl is-enabled r2d2-your-service.service  # Must return: enabled
systemctl status r2d2-your-service.service      # Must show: active (running)
```

### When You've Completed Phases 1-2

✅ Feature implemented  
✅ Built successfully  
✅ Tested manually and works  
✅ Service installed and running  

**→ NOW:** Switch to `000_AGENT_FINALIZATION_GUIDE.md` for:
- Phase 3: Verification (reboot testing)
- Phase 4: Documentation
- Phase 5: Git & Deployment

---

## 🖥️ Execution Environment (READ THIS FIRST!)

### WHERE ARE WE?

**YOU ARE EXECUTING DIRECTLY ON THE JETSON AGX ORIN**

- ✅ All commands run **ON the Jetson itself** (not via SSH)
- ✅ The Jetson is **BOTH** the development machine AND deployment target
- ✅ Terminal commands execute in the local Jetson shell
- ✅ `/home/severin/dev/r2d2` is the local workspace on Jetson storage
- ✅ All systemd services run locally on the Jetson

### What This Means For You

**File paths are local:**
- `~/dev/r2d2/` = `/home/severin/dev/r2d2/` on Jetson's NVMe SSD
- No need for `scp`, `ssh`, or remote file transfer
- Direct file access via standard file operations

**Services are local:**
- `systemctl` manages services on this Jetson
- `ros2` commands execute on this Jetson
- `tegrastats` shows THIS machine's stats

**You have full access:**
- Root privileges available via `sudo`
- Direct hardware access (camera, GPIO, I2S audio)
- Real-time sensor data from local devices

### Remote Access Context (For Understanding Only)

**Tailscale VPN is configured but NOT used by agents:**
- Allows Windows laptop to access Jetson for monitoring
- Used for web dashboard access from remote location
- **Agents do NOT connect via Tailscale** - you're already on the Jetson
- Think of it like a security camera - allows viewing, but camera runs independently

**The Jetson operates autonomously:**
- All ROS 2 nodes run locally
- Speech processing happens on local GPU
- Face recognition uses local camera
- No cloud dependency for core functions (except OpenAI API for speech)

### Quick Environment Check

If you're ever unsure, these commands confirm you're on the Jetson:

```bash
# Check hostname (should show: r2d2)
hostname

# Check architecture (should show: aarch64)
uname -m

# Check NVIDIA GPU (should show: Orin)
tegrastats | head -n 1

# Check project location
pwd  # Should be somewhere under /home/severin/
```

---

## Quick Command Reference

### Environment Setup (CORRECT ORDER MATTERS!)
```bash
# ALWAYS IN THIS ORDER - Order matters on ARM!
source ~/depthai_env/bin/activate      # DepthAI first!
source ~/.bashrc                        # Then bash config
source ~/dev/r2d2/ros2_ws/install/setup.bash  # Finally ROS 2
export OPENBLAS_CORETYPE=ARMV8          # Critical for Jetson ARM
```

### Build & Run
```bash
# Standard build (one package)
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select <package_name>

# Clean rebuild (when cache is stale)
rm -rf build install log && colcon build --packages-select <package_name>

# Launch audio notifications
ros2 launch r2d2_audio audio_notification.launch.py

# Launch camera perception
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py

# Check ROS 2 topics
ros2 topic list
ros2 topic echo /topic_name -n 20
ros2 topic hz /topic_name -w 5

# Systemd services (see 005_SYSTEMD_SERVICES_REFERENCE.md for complete reference)
sudo systemctl status r2d2-<service>.service
sudo systemctl restart r2d2-<service>.service
journalctl -u r2d2-<service>.service -f
```

### Debug & Troubleshooting
```bash
# Process stuck?
pkill -9 -f ros2 && sleep 2

# Inspect topic data
ros2 topic echo <topic> -n 1

# Check performance
top                    # CPU/memory
tegrastats             # Jetson thermal/power
```

---

## System Architecture

### Software Stack
```
ROS 2 Humble
    ├── r2d2_audio (audio notifications)
    ├── r2d2_perception (image processing)
    └── r2d2_bringup (camera + perception launch)
         ├── DepthAI Python SDK (OAK-D camera control)
         ├── OpenCV (image processing)
         └── NumPy/SciPy (numerical computing)

Linux ALSA
    └── PAM8403 Amplifier → Speaker
```

### ROS 2 Node Layout
```
r2d2_camera_perception (launches on boot)
  ├── OAK-D Camera → /oak/rgb/image_raw (30 Hz)
  ├── Perception Node → /r2d2/perception/brightness (13 Hz)
  └── Face Recognition Node → /r2d2/perception/person_id (varies)

r2d2-audio-notification.service (systemd service)
  ├── Subscribes: /r2d2/perception/person_id
  ├── Plays: Audio files via ffplay
  └── Publishes: /r2d2/audio/notification_event
```

### Package Structure
```
ros2_ws/src/
├── r2d2_audio/
│   ├── audio_notification_node.py (subscribes to person_id, plays audio)
│   ├── audio_player.py (MP3 playback utility)
│   ├── assets/audio/ (MP3 files: Voicy_R2-D2 - 2.mp3, 5.mp3)
│   └── launch/audio_notification.launch.py
│
├── r2d2_perception/
│   ├── image_listener.py (face recognition, brightness analysis)
│   └── launch/perception.launch.py
│
└── r2d2_bringup/
    └── launch/r2d2_camera_perception.launch.py (main launch file)
```

---

## Platform & Hardware Reference

### Hardware Fixed Constants
| Item | Value | Notes |
|------|-------|-------|
| Platform | NVIDIA Jetson AGX Orin 64GB | ARM64, not x86 |
| OS | Ubuntu 22.04 Jammy | Jetson-specific image |
| ROS 2 | Humble | Required version |
| Camera | OAK-D Lite AF | 1920×1080 @ 30 FPS |
| Camera Serial | 19443010E1D30C7E00 | Specific unit |
| Audio Output | Jetson J511 Pin 9 (HPO_L) via I2S | PAM8403 → 8Ω speaker |
| Python | 3.10.6 | System or venv |
| Project Root | ~/dev/r2d2 | NOT /opt, NOT /home/user |

### Critical ARM Issue
**Symptom:** `Illegal instruction (core dumped)` when running Python  
**Cause:** OpenBLAS optimized for wrong CPU architecture  
**Fix:** `export OPENBLAS_CORETYPE=ARMV8` BEFORE any ROS 2 commands  
**Prevention:** Add to ~/.bashrc

### Project Directory Structure
```
~/dev/r2d2/
├── ros2_ws/                                # ROS 2 workspace
│   ├── src/r2d2_*/                        # Source packages
│   ├── build/, install/, log/             # Build artifacts
│   └── install/setup.bash                 # Source for ROS 2 env
│
├── tests/camera/                          # Test outputs
│   ├── perception_debug.jpg               # RGB capture (1920×1080)
│   └── perception_debug_gray.jpg          # Grayscale (640×360)
│
├── 000_INTERNAL_AGENT_NOTES.md            # This file
├── 001_ARCHITECTURE_OVERVIEW.md           # System design
├── README.md                              # Main overview
└── _ANALYSIS_AND_DOCUMENTATION/           # Analysis & reference
    └── (detailed diagnostic files)
```

---

## Documentation Standards

### What Gets Documented (In 0XX_ to 999_ Files)
✅ All the important main Documentations!
✅ User-facing setup and configuration  
✅ How to run and test the system  
✅ What each parameter does  
✅ Configuration examples  
✅ Troubleshooting guide  

### What Goes in _ANALYSIS_AND_DOCUMENTATION/
✅ Detailed analysis and diagnostics  
✅ Cost breakdowns and research  
✅ Technical deep-dives  
✅ Historical notes and process documents  
✅ (Link these from main docs!)

### What Goes in _TEMP/ (Temporary Scratch Folder - NOT Tracked)
✅ Session logs and build reports  
✅ Debugging artifacts and verification reports  
✅ Draft analysis (work-in-progress)  
✅ Agent working notes during sessions  
✅ **These files are NEVER committed to git (folder is in .gitignore)**  
✅ **AGENTS: Ignore any files in _TEMP/ when reading context or instructions**

### Temporary File Management (Industry Best Practice)
- **_TEMP/ folder:** For temporary scratch work (gitignored)
- **Root-level TEMP_*.md:** Also gitignored (legacy cleanup)
- **Permanent docs:** Use numbered 000-999 files or _ANALYSIS_AND_DOCUMENTATION/

### Documentation Workflow
1. Work in _TEMP/ for session logs, debug reports, build verification
2. Extract key findings to main docs (000-999 numbered files)
3. Move detailed analysis to _ANALYSIS_AND_DOCUMENTATION/ for permanent storage
4. Clean up _TEMP/ files when session is complete (auto-ignored anyway)
5. Commit only permanent content to git

**Rule for Agents:** When reading project context or looking for instructions:
- ✅ READ: Main docs (000-999), _ANALYSIS_AND_DOCUMENTATION/
- ❌ IGNORE: _TEMP/ - these are session artifacts, not source of truth

---

## Documentation Hierarchy & File Organization (UPDATED DEC 2025)

### Documentation Structure Overview

```
/home/severin/dev/r2d2/
├── 000-099: Internal, Setup & Infrastructure
│   ├── 000_INTERNAL_AGENT_NOTES.md (this file - for AI agents)
│   ├── 001_ARCHITECTURE_OVERVIEW.md (complete system architecture)
│   ├── 002_HARDWARE_REFERENCE.md (all hardware specifications)
│   ├── 003-010: Jetson setup, backup/restore, system config
│   └── ...
├── 100-199: Perception & Status System (Phase 1)
│   ├── 100_PERCEPTION_STATUS_REFERENCE.md (complete technical reference)
│   ├── 101-103: Installation, quick start, troubleshooting
│   ├── 110-111: Web dashboard architecture and documentation
│   └── ...
├── 200-299: Speech System (Phase 2)
│   ├── 200_SPEECH_SYSTEM_REFERENCE.md (complete technical reference)
│   ├── 201-204: Installation, quick start, troubleshooting, customization
│   └── ...
├── 300-399: AI Tutor & Learning System
│   ├── 300_AI_TUTOR.md (complete AI tutor reference - coding + general modes)
│   └── (Gesture docs archived to _ARCHIVE/)
├── 400-499: Reserved for Navigation System (Phase 3)
├── 500-599: Reserved for Memory & Personality (Phase 4)
├── README.md (project overview - main entry point)
├── LICENSE
└── r2d2_power_button_simple.py (systemd service script)

_ANALYSIS_AND_DOCUMENTATION/  (≤15 supplementary docs only)
├── README.md (index of supplementary docs - REQUIRED)
├── QUICK_START.md
├── AUDIO_QUICK_REFERENCE.md
├── COMPUTE_COST_ANALYSIS.md
├── INTEGRATION_GUIDE.md
├── OPERATIONS_CHECKLIST.md
└── QUICK_ANSWERS.md

_ARCHIVE/  (Historical documents - NOT for active reference)
├── analysis_history_2025-12-24/ (60+ old Phase 1-2 analysis docs)
├── misc_files_2025-12-24/ (debug images, audio samples)
└── (future dated folders...)

_TEMP/  (Transient work only - gitignored)
├── cleanup_audit.md
├── CONTENT_EXTRACTION_PLAN.md
└── (temporary analysis - DELETE when task complete)

scripts/  (Organized by function)
├── deprecated/phase1-development/ (old fix/install scripts)
├── install/ (installation and setup scripts)
├── start/ (launch scripts for services)
├── test/ (testing scripts)
└── util/ (monitoring and utility scripts)

docs/  (Supporting materials)
├── photos/ (hardware photos for README)
└── tools/ (utility documents like VIEW_DIAGRAMS.md)

ros2_ws/  (ROS 2 workspace - standard structure)
├── src/ (source packages)
├── build/ (generated - gitignored)
├── install/ (generated - gitignored)
└── log/ (generated - gitignored)

data/  (Runtime data and models)
├── persons.db (Person Registry database)
├── face_recognition/models/ (trained LBPH models)
├── gesture_recognition/models/ (trained SVM classifiers)
└── conversations/ (speech conversation logs)
```

### Documentation Rules & Best Practices

#### Numbered Files (000-999) - SINGLE SOURCE OF TRUTH

**Purpose:** All authoritative, permanent documentation

**Rules:**
- ✅ **Primary reference** - All main documentation lives here
- ✅ **Persistent** - Never delete, only update or mark deprecated
- ✅ **Version controlled** - All changes tracked in git
- ✅ **Cross-referenced** - Link only to other numbered docs or README
- ❌ **Do NOT reference** `_ANALYSIS_AND_DOCUMENTATION/` from numbered docs

**Naming Convention:**
- `000-099`: Internal guides, setup, infrastructure
- `100-199`: Phase 1 (Perception & Status)
- `200-299`: Phase 2 (Speech & Conversation)
- `300-399`: AI Tutor & Learning System
- `400-499`: Phase 3 (Navigation - reserved)
- `500-599`: Phase 4 (Memory & Personality - reserved)

#### _ANALYSIS_AND_DOCUMENTATION/ - SUPPLEMENTARY ONLY

**Purpose:** Quick references, checklists, FAQs (NOT primary docs)

**Rules:**
- ✅ **Max 15 files** - Keep lean and focused
- ✅ **README.md required** - Must index all files in folder
- ✅ **Quick reference only** - Cheat sheets, checklists, summaries
- ❌ **Not primary source** - Numbered docs (000-999) are authoritative
- ❌ **Do NOT extract from here** - These supplement, not replace main docs

**Allowed Types:**
- Quick start guides (for experienced users)
- Command cheat sheets (audio, ROS, systemd)
- Operations checklists (daily procedures)
- FAQ documents (common questions)
- Performance benchmarks (detailed analysis)
- Integration templates (for adding features)

#### _ARCHIVE/ - HISTORICAL ONLY

**Purpose:** Historical documents, old analysis, deprecated code

**Rules:**
- ✅ **Dated folders** - Use `YYYY-MM-DD` format
- ✅ **README per folder** - Document what was archived and why
- ❌ **Never reference** - These are archaeology, not current work
- ❌ **Never active use** - If needed again, copy to correct location

**Archive Types:**
- Implementation history (completed feature docs)
- Old analysis documents (superseded by numbered docs)
- Debug/diagnostic files (outdated troubleshooting)
- Deprecated scripts (replaced by systemd services)

#### _TEMP/ - TRANSIENT WORK ONLY

**Purpose:** Work-in-progress, scratch notes, session artifacts

**Rules:**
- ✅ **Gitignored** - Never committed to repository
- ✅ **Delete when done** - Clean up after task completion
- ❌ **Never reference** - These are scratch work, not documentation
- ❌ **Agents ignore** - Do not read for project context

**Usage:**
- Session logs and build reports
- Debugging artifacts during development
- Draft analysis before finalizing
- Cleanup plans and audit reports (temporary)

### File Movement & Cleanup Guidelines

**When to archive a file:**
1. Analysis complete and findings extracted to numbered docs
2. Implementation finished and documented in main reference
3. Feature superseded by newer implementation
4. Diagnostic/troubleshooting no longer relevant

**How to archive:**
```bash
# Create dated folder
mkdir -p _ARCHIVE/description_YYYY-MM-DD/

# Move files
mv <file1> <file2> ... _ARCHIVE/description_YYYY-MM-DD/

# Create README
cat > _ARCHIVE/description_YYYY-MM-DD/README.md << 'EOF'
# Description Archive - YYYY-MM-DD

**Purpose:** <why these were archived>

**Contents:**
- <file1>: <description>
- <file2>: <description>
...

**Extracted to:** <numbered doc references>

**Status:** Archived - historical reference only
EOF
```

**When to delete (not archive):**
- Generated files (build artifacts)
- True temporary files (session logs)
- Duplicate copies of tracked files
- Empty or placeholder files

### Documentation Maintenance Checklist

**Monthly (or after major feature completion):**
- [ ] Review `_ANALYSIS_AND_DOCUMENTATION/` - ensure ≤15 files
- [ ] Check for duplicate information across numbered docs
- [ ] Archive completed implementation/analysis docs
- [ ] Update `_ANALYSIS_AND_DOCUMENTATION/README.md`
- [ ] Clean `_TEMP/` folder (should always be minimal)

**After Each Major Feature:**
- [ ] Extract key findings to numbered docs (000-999)
- [ ] Move implementation notes to `_ARCHIVE/`
- [ ] Update relevant reference docs
- [ ] Delete temporary analysis files

**Before Each Git Commit:**
- [ ] Verify `_TEMP/` is gitignored (should not appear in `git status`)
- [ ] Check no loose files in root (scripts, images, audio)
- [ ] Ensure all numbered docs are properly cross-referenced
- [ ] Update README.md if new numbered docs added

**For Git Best Practices and deployment steps, see:** `000_AGENT_FINALIZATION_GUIDE.md`

---

## Documentation Deduplication Rules (MANDATORY)

### Source of Truth Hierarchy

When documenting configuration values, parameters, or procedures, ALWAYS check the authoritative source first:

| Topic | Authoritative Document |
|-------|----------------------|
| **Config parameters** | Actual YAML config files in `ros2_ws/src/*/config/` |
| **Systemd services** | `005_SYSTEMD_SERVICES_REFERENCE.md` |
| **State machine (RED/GREEN/BLUE)** | `100_PERCEPTION_STATUS_REFERENCE.md` |
| **ROS 2 topics** | `001_ARCHITECTURE_OVERVIEW.md` |
| **Monitoring commands** | `006_SYSTEM_STATUS_AND_MONITORING.md` |
| **Person Registry** | `250_PERSON_MANAGEMENT_SYSTEM_REFERENCE.md` |
| **Speech system** | `200_SPEECH_SYSTEM_REFERENCE.md` |
| **Troubleshooting** | `103_PERCEPTION_STATUS_TROUBLESHOOTING.md` or `203_SPEECH_SYSTEM_TROUBLESHOOTING.md` |

### Before Documenting Parameters/Commands

1. **Check** if already documented in authoritative source (table above)
2. **If yes:** ADD CROSS-REFERENCE, do NOT copy content
3. **If no:** Add to authoritative source first, then cross-reference

### Cross-Reference Formats

**For configurable parameters:**
```markdown
> **Source of Truth:** [`audio_params.yaml`](ros2_ws/src/r2d2_audio/config/audio_params.yaml) - check config file for current values
```

**For procedures/commands:**
```markdown
> **See:** [`005_SYSTEMD_SERVICES_REFERENCE.md`](005_SYSTEMD_SERVICES_REFERENCE.md) for complete service management
```

**For quick-start summaries (allowed to include brief info):**
```markdown
**Quick Reference:** (see [`100_PERCEPTION_STATUS_REFERENCE.md`](100_PERCEPTION_STATUS_REFERENCE.md) for complete details)
- RED = recognized (4 matches in 1.5s window)
- GREEN = unknown person
- BLUE = no person
```

### Why This Matters

On December 2025, documentation audit found:
- Same parameter values documented in 5+ places
- 3 files had OUTDATED values (3 instead of 4, 1.0 instead of 1.5)
- Even the SAME document had BOTH correct and incorrect values

**Result:** Confusion about actual system behavior.

**Prevention:** Always reference source of truth, never copy parameter values.

---

## Service Management & Resource Optimization

### Auto-Start Configuration (Production Deployment)

**Services that auto-start on boot:**
1. ✅ `r2d2-camera-perception.service` (camera + face + gesture recognition)
2. ✅ `r2d2-audio-notification.service` (audio alerts + LED + logger)
3. ✅ `r2d2-gesture-intent.service` (gesture-to-speech control)
4. ✅ `r2d2-speech-node.service` (Fast Mode - OpenAI Realtime API, ~1s latency)
5. ✅ `r2d2-rest-speech-node.service` (Intelligent Mode - REST APIs with o1-preview, ~3-5s latency)
6. ✅ `r2d2-heartbeat.service` (system health monitoring)
7. ✅ `tailscaled.service` (VPN access)
8. ⚠️ `r2d2-powerbutton.service` (optional - if physical button installed)

**Dual-Mode Speech System (December 2025):**
- **Fast Mode** (index finger ☝️): OpenAI Realtime API, chatty personality, ~1s response
- **R2-D2 Mode** (open hand 🖐️): REST APIs (Whisper→gpt-4o→TTS with `echo` voice), terse astromech personality, ~2-3s response
- **Stop** (fist ✊): Stops active conversation

**Services that remain on-demand (manual start):**
1. ❌ `r2d2-rosbridge.service` (web dashboard WebSocket bridge)
2. ❌ `r2d2-web-dashboard.service` (FastAPI REST API + UI)
3. ❌ `r2d2-camera-stream.service` (MJPEG video stream - **device conflict with camera-perception**)

**Resource Usage Summary:**
- **Auto-start services:** 16-26% CPU, ~400 MB RAM (core functionality)
- **All services active:** 26-39% CPU, ~600 MB RAM (includes web dashboard + speech)
- **Available headroom:** 74-84% CPU, ~63.5 GB RAM (for future features)

**⚠️ Critical Rule: camera_stream_node and camera_node are MUTUALLY EXCLUSIVE**
- Both require exclusive camera access (OAK-D Lite via DepthAI SDK)
- NEVER auto-start camera_stream (causes device conflict with core perception)
- camera_stream is ONLY for web dashboard preview (on-demand via API)

### Service Control Commands

> **See:** [`005_SYSTEMD_SERVICES_REFERENCE.md`](005_SYSTEMD_SERVICES_REFERENCE.md) for complete service management documentation.

**Quick reference:**
```bash
# Status/restart/logs for any service
sudo systemctl status r2d2-<service>.service
sudo systemctl restart r2d2-<service>.service
journalctl -u r2d2-<service>.service -f

# Enable/disable auto-start
sudo systemctl enable r2d2-<service>.service
sudo systemctl disable r2d2-<service>.service
```

### Parameter Tuning Guidelines

**Face Recognition (if CPU constrained):**
- Increase `recognition_frame_skip` (default: 2) → 3 or 4 for ~2-3% CPU savings
- Trade-off: Slower recognition response (~150ms slower per frame skip increment)

**Gesture Recognition (if CPU constrained):**
- Increase `gesture_frame_skip` (default: 5) → 7 or 10 for lower CPU usage
- Trade-off: Reduced responsiveness

**Audio Alerts (if silent environment needed):**
- Set `audio_volume:=0.0` to mute all audio alerts
- LED status still works (visual feedback only)

**Speech Auto-Shutdown (cost optimization):**
- Adjust `auto_shutdown_timeout_seconds` (default: 35)
- Longer timeout (60-300s) = fewer API reconnections but higher cost
- Shorter timeout (20-30s) = more aggressive cost savings

**Current settings are already well-optimized. Only adjust if specific needs arise.**

**For detailed optimization analysis, see:** `_ANALYSIS_AND_DOCUMENTATION/006_OPTIMIZATION_ANALYSIS.md` (archived)

---

## For Future Agents Reading This

### Recent Fixes & Important History

**December 27, 2025 - R2-D2 Mode Fully Tested & Working:**
- **Status:** ✅ Full end-to-end testing completed successfully
- **Fixed:** Gesture detection → REST speech service routing → multi-turn conversation loop
- **Config:** `silence_duration: 1.5s` for natural conversation flow
- **Verified:** open_hand (🖐️) triggers R2-D2 mode, fist (✊) stops, continuous turns work

**December 26, 2025 - R2-D2 Personality Mode:**
- **Update:** Renamed "Intelligent Mode" to "R2-D2 Mode" with authentic astromech personality
- **Changes:**
  - `intelligent_model`: `o1-preview` → `gpt-4o` (faster responses, ~2-3s)
  - `tts_voice`: `nova` → `echo` (robotic character)
  - `intelligent_instructions`: New terse, mission-oriented R2-D2 prompt with [beeps], [chirps]
- **Character:** Ultra-short responses, parenthetical sound flavor, occasionally sarcastic
- **Documentation:** `204_SPEECH_SYSTEM_VOICE_CONFIGURATION.md` fully updated

**December 25, 2025 - Dual-Mode Gesture Speech System:**
- **Feature:** Two speech modes triggered by different gestures
- **Fast Mode** (☝️ index finger): OpenAI Realtime API, chatty personality, ~1s latency
- **R2-D2 Mode** (🖐️ open hand): REST APIs (Whisper→gpt-4o→TTS), terse astromech personality, ~2-3s latency
- **New Components:**
  - `r2d2_speech/rest_api/rest_speech_client.py` - Turn-based STT→LLM→TTS pipeline
  - `rest_speech_node.py` - ROS2 lifecycle node for R2-D2 Mode
  - `r2d2-rest-speech-node.service` - Systemd service (auto-start enabled)
- **Updated:** gesture_intent_node routes open_hand to REST speech services
- **Training:** 3-gesture model (index_finger_up, fist, open_hand)
- **Branch:** `feature/dual-mode-gesture-speech`
- **Config:** `speech_params.yaml` has personality configs for both modes

**December 24, 2025 - Speech Service Path Fix:**
- **Issue:** Speech system wouldn't start after index finger gesture (Phase 4 → Phase 5 blocked)
- **Root Cause:** `r2d2-speech-node.service` ExecStart pointed to old path `/home/severin/dev/r2d2/start_speech_node.sh`
- **Solution:** Updated to correct path `/home/severin/dev/r2d2/scripts/start/start_speech_node.sh`
- **Context:** Scripts were moved to `scripts/start/` during December 2025 documentation cleanup, but service files were not all updated
- **Status:** Fixed and working. gesture_intent_node now successfully calls start_session service.
- **Remaining:** 4 other services still need path updates (audio-notification, heartbeat, rosbridge, camera-stream)

**December 24, 2025 - Documentation Reorganization:**
- Moved `SYSTEM_STATUS_AND_MONITORING.md` to `006_SYSTEM_STATUS_AND_MONITORING.md` (official 000-999 series)
- Created `005_SYSTEMD_SERVICES_REFERENCE.md` (complete service documentation)
- Updated `001_ARCHITECTURE_OVERVIEW.md` with System Components Map
- Documented `tools/minimal_monitor.py` (comprehensive one-line monitoring tool)
- All changes on branch: `cleanup/docs-2025-12-24`

**December 2025 - Script Organization:**
- All startup scripts moved to `scripts/start/` directory
- Systemd service files need ExecStart path updates to match
- See `005_SYSTEMD_SERVICES_REFERENCE.md` for complete path audit

### What You Need to Know From This File
- Environment setup order (DepthAI → bashrc → ROS2)
- System architecture (nodes, packages, topics)
- How to build, test, and install services
- Troubleshooting common issues
- Documentation management rules

### When You've Completed Development & Testing
1. ✅ Feature implemented and working
2. ✅ Built successfully (colcon build)
3. ✅ Service installed and enabled
4. ✅ Manual testing complete

**→ NEXT:** Switch to `000_AGENT_FINALIZATION_GUIDE.md` for:
- Verification (reboot testing)
- Documentation updates
- Git commit and push
- Post-deployment monitoring

---

## System Monitoring and Diagnostics

### Freeze Monitor System
**Status:** ✅ Active since December 16, 2025

The Jetson experiences periodic freezes (every 30-60 minutes). A comprehensive monitoring system is now in place:

- **Service:** `freeze-monitor.service` (systemd)
- **Logs:** `/var/log/freeze_logs/` (5 separate log files)
- **Interval:** Every 10 seconds
- **Documentation:** `050_FREEZE_MONITOR_SYSTEM.md`

**After a freeze:**
```bash
# Quick analysis
tail -100 /var/log/freeze_logs/summary.log
tail -50 /var/log/freeze_logs/hardware.log
tail -50 /var/log/freeze_logs/kernel.log
```

**Service management:**
```bash
systemctl status freeze-monitor
journalctl -u freeze-monitor -f
```

---

## File Location Rules & Documentation Hierarchy

### System Scripts in /home/severin/ (DO NOT MOVE)

These scripts MUST remain in `/home/severin/` as they are referenced by systemd services and autostart:

**Freeze Monitor System:**
- `/home/severin/freeze_monitor.py` - Main monitor script (freeze-monitor.service)
- `/home/severin/install_freeze_monitor.sh` - Installer
- `/home/severin/apply_3day_retention.sh` - Log retention config
- `/home/severin/apply_14day_retention.sh` - Log retention config

**Display Configuration:**
- `/home/severin/set-lg-resolution.sh` - Autostart display config (referenced in ~/.config/autostart/)
- `/home/severin/set-3440x1440.sh` - Resolution helper

**GPU Utilities:**
- `/home/severin/r2d2-gpu-run.sh` - GPU container helper (documented in 007)
- `/home/severin/test_gpu_speech.sh` - GPU test script

⚠️ **DO NOT MOVE THESE FILES** - They are active system components with hardcoded references.

### Documentation Hierarchy (STRICTLY ENFORCED)

#### Project Root (dev/r2d2/)
```
dev/r2d2/
├── README.md (EXCEPTION - GitHub standard)
├── 000-999_*.md (Authoritative main documentation)
└── docs/ (Supplementary documentation organized by category)
```

#### Numbered Documentation (000-999)
- **Purpose:** Authoritative, high-level documentation
- **Naming:** `NNN_DESCRIPTIVE_NAME.md` (e.g., `005_SYSTEMD_SERVICES_REFERENCE.md`)
- **Location:** Project root only
- **Rules:**
  - Use lowercase with underscores
  - Keep focused and well-organized
  - Reference supplementary docs in `docs/` when needed

#### Supplementary Documentation (docs/)
```
docs/
├── hardware/       # Wiring diagrams, hardware guides
│   └── led_wiring.md
├── troubleshooting/ # Detailed troubleshooting guides
│   ├── tailscale_vpn.md (consolidated from 5 files)
│   └── freeze_monitor.md
├── setup/          # Detailed setup and installation guides
│   └── gpu_acceleration.md (consolidated from 4 files)
├── reference/      # Best practices, reference materials
│   └── jetson_best_practices.md
├── photos/         # Build photos and images
└── tools/          # Tool-specific documentation
```

#### Scripts Organization
```
scripts/
├── start/          # Service startup scripts (referenced by systemd)
├── install/        # Installation scripts
│   ├── freeze_monitor/
│   └── display/
├── util/           # Helper utilities
└── deprecated/     # Archived/obsolete scripts
    └── troubleshooting/
```

#### Other Folders
- **`_ARCHIVE/`** - Historical documentation, archived files (git-tracked)
- **`_TEMP/`** - Work-in-progress, scratch files (gitignored)
- **`_ANALYSIS_AND_DOCUMENTATION/`** - Quick reference docs (max 15 files)
- **`tools/`** - Active monitoring and utility scripts
- **`ros2_ws/`** - ROS 2 workspace (standard structure)
- **`data/`** - Training data, models

### Rules for AI Agents

#### BEFORE Creating Any .md File:
1. Is it authoritative documentation? → `NNN_NAME.md` in project root
2. Is it hardware/wiring? → `docs/hardware/`
3. Is it troubleshooting? → `docs/troubleshooting/`
4. Is it setup/installation? → `docs/setup/`
5. Is it reference/best practices? → `docs/reference/`
6. Is it implementation history? → `_ARCHIVE/`
7. Is it work-in-progress? → `_TEMP/` (gitignored)

#### BEFORE Creating Any Script:
1. Does it start a service? → `scripts/start/`
2. Is it for installation/setup? → `scripts/install/{category}/`
3. Is it a utility/helper? → `scripts/util/`
4. Is it obsolete/debug only? → `scripts/deprecated/`

#### Consolidation Rules:
- ✅ Consolidate redundant docs (e.g., 4 GPU docs → 1)
- ✅ Use consistent naming (lowercase_underscore.md)
- ✅ Organize by function in subdirectories
- ❌ Never create loose files in `/home/severin/`
- ❌ Never create multiple docs for same topic
- ❌ Never skip proper folder organization

### Verification Checklist

Before committing documentation changes:
- [ ] No project files in `/home/severin/` (except system scripts above)
- [ ] All numbered docs follow `NNN_NAME.md` pattern
- [ ] Supplementary docs organized in `docs/{category}/`
- [ ] Scripts organized in `scripts/{function}/`
- [ ] No redundant documentation (consolidated if needed)
- [ ] Cross-references updated and valid
- [ ] README.md reflects current state

---

## 🔒 Security & Sensitive Data Handling

### What is Security-Sensitive (DO NOT commit to git)

| Category | Examples | Why Sensitive |
|----------|----------|---------------|
| **Real IP addresses** | `100.95.x.x` (Tailscale), `192.168.x.x` (local) | Reveals network topology |
| **Email addresses** | `user@domain.com` | Personal identification |
| **API keys/tokens** | OpenAI API key, Tailscale auth key | Security credentials |
| **SSH keys** | Private keys, key fingerprints | Authentication bypass risk |
| **Hostnames** | Machine-specific names | Network enumeration |

### What is Safe to Commit

- ✅ Placeholder IPs: `100.x.x.x`, `192.168.x.1`
- ✅ Generic usernames: `user@jetson`, `username`
- ✅ Architecture diagrams and procedures
- ✅ Configuration templates (without real values)
- ✅ Scripts using environment variables for secrets
- ✅ Documentation with sanitized examples

### Two-Tier Documentation Strategy

```
┌─────────────────────────────────────────────────────────────┐
│                    GIT REPOSITORY (Public)                  │
│  • Sanitized documentation (placeholder IPs)                │
│  • Generic configuration examples                           │
│  • System architecture and procedures                       │
│  • Scripts (no hardcoded secrets)                          │
└─────────────────────────────────────────────────────────────┘
                              ↕
                    SEPARATE STORAGE
                              ↕
┌─────────────────────────────────────────────────────────────┐
│                    USB BACKUP (Private)                     │
│  • Real ~/.ssh/config with actual IPs                      │
│  • System configs with real addresses                       │
│  • API keys in environment files                           │
│  • Complete working configurations                          │
└─────────────────────────────────────────────────────────────┘
```

### Before Every Git Commit (MANDATORY)

**Security pre-commit checklist:**

```bash
# Check for real Tailscale IPs (100.x.x.x range)
grep -rE "100\.[0-9]+\.[0-9]+\.[0-9]+" ~/dev/r2d2/*.md

# Check for local network IPs (192.168.x.x range)
grep -rE "192\.168\.[0-9]+\.[0-9]+" ~/dev/r2d2/*.md

# Check for email addresses
grep -rE "[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}" ~/dev/r2d2/*.md
```

**If matches found:**
1. Replace real IPs with placeholders (`100.x.x.x`, `192.168.x.1`)
2. Replace emails with `user@example.com`
3. Ensure USB backup has preserved real configs first

### Security Checklist for Agents

- [ ] **Before committing:** Run grep commands above
- [ ] **Real IPs found?** Replace with placeholders
- [ ] **USB backup current?** Run backup to preserve real configs
- [ ] **API keys in code?** Use environment variables instead
- [ ] **SSH configs?** Use generic examples, not real paths

### Recovery Strategy

Real configurations are preserved in USB backups (see `004_BACKUP_AND_RESTORE.md`):
- USB backup contains actual IPs, SSH configs, system files
- After restore from USB, system works with real addresses
- Git documentation provides procedures, USB provides real values

**For detailed backup procedures, see:** `004_BACKUP_AND_RESTORE.md`

---

**This document is a living reference. Update when patterns change or new tools/patterns emerge.**

**Last Updated:** December 29, 2025 - Added AI Tutor System (Learning Mode + Tutor Mode), updated doc hierarchy
