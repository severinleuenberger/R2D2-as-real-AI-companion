# R2D2 AI Tutor System - Complete Reference
## Voice-Activated Learning Assistant

**Date:** December 2025  
**Status:** ✅ OPERATIONAL  
**Platform:** NVIDIA Jetson AGX Orin 64GB + ROS 2 Humble

---

## Executive Summary

The R2D2 AI Tutor System provides two complementary learning modes designed for Severin's robotics education journey:

1. **Coding Tutor Mode** - Real-time narration during Cursor coding sessions
   - R2D2 speaks explanations as the AI agent works
   - Triggered by: "Turn on learning mode"
   - Uses file-watching + espeak TTS
   - Perfect for understanding code changes as they happen

2. **General Tutor Mode** - Interactive teaching conversations
   - R2D2 becomes a patient teacher for any topic
   - Triggered by: "Be my tutor" / "Teach me about X"
   - Uses enhanced teaching personality with BI analogies
   - Perfect for learning concepts through Q&A

Both modes track learning progress in SQLite for long-term skill development.

**Target Audience:** Senior BI developer learning robotics/ROS 2. All explanations use database and ETL analogies.

---

## Quick Start

### Activate Coding Tutor (for Cursor sessions)

```
You: [index finger gesture]
R2D2: "Hello!"
You: "Turn on learning mode"
R2D2: "[chirp] Learning mode activated! I'll explain your coding!"
[R2D2 now narrates what Cursor's AI agent does]
```

### Activate General Tutor (for Q&A learning)

```
You: [index finger gesture]
R2D2: "Hello!"
You: "Be my tutor"
R2D2: "[beep] Tutor mode on. What would you like to learn about?"
You: "Explain ROS 2 topics"
R2D2: "Think of a ROS 2 topic like a database table. Publishers 
       INSERT rows, subscribers have triggers that fire on new data..."
```

### Deactivate Either Mode

```
You: "Learning off" or "Stop learning"     → Stops coding narrator
You: "Tutor off" or "Normal mode"          → Returns to normal R2D2 personality
```

---

## Voice Command Reference

### Coding Tutor Commands

| Phrase | Effect |
|--------|--------|
| "Turn on learning mode" | Start narrator service |
| "Activate learning" | Start narrator service |
| "Explain my code as we go" | Start narrator service |
| "Help me learn while coding" | Start narrator service |
| "Stop learning mode" | Stop narrator service |
| "Learning off" | Stop narrator service |
| "Deactivate learning" | Stop narrator service |

### General Tutor Commands

| Phrase | Effect |
|--------|--------|
| "Tutor mode" | Activate teaching personality |
| "Be my tutor" | Activate teaching personality |
| "Be my teacher" | Activate teaching personality |
| "I want to learn" | Activate teaching personality |
| "Teach me about X" | Activate + start teaching X |
| "Explain X to me" | Activate + start teaching X |
| "Normal mode" | Deactivate teaching personality |
| "Stop teaching" | Deactivate teaching personality |
| "Tutor off" | Deactivate teaching personality |

### Complete Voice Command Cheat Sheet

| Say This | R2D2 Does |
|----------|-----------|
| "Turn on learning mode" | Starts coding narrator |
| "Learning off" | Stops coding narrator |
| "Be my tutor" | Activates teaching mode |
| "Tutor off" | Deactivates teaching mode |
| "Explain that again" | Re-explains last concept |
| "What's a [concept]?" | Teaches the concept |
| "Give me a BI analogy" | Explains using BI terms |

---

## Part 1: Coding Tutor Mode

### How It Works

```
[Cursor AI Agent works on code]
         │
         ▼
[Agent writes to coding_live.md]
         │
         ▼
[Narrator Service detects change]
         │
         ▼
[Extracts "What This Means" section]
         │
         ▼
[espeak speaks explanation aloud]
         │
         ▼
[You hear R2D2 explain what just happened]
```

### The coding_live.md Format

AI agents write explanations in this structured format:

```markdown
## Current Action
[One line: what the agent just did]

## What This Means
[2-3 sentences explaining the concept]
[This section is spoken by R2D2]

## BI Analogy
[Comparison to a Business Intelligence concept]

## Category
[Category] > [Subcategory] | Understanding: [1-5]
```

### Example Entry

```markdown
## Current Action
Creating a ROS 2 subscriber for /r2d2/perception/person_id

## What This Means
A subscriber listens to a topic and executes a callback function 
whenever new data arrives. It's event-driven - the system pushes 
data to you rather than you polling for it.

## BI Analogy
Like a SQL trigger that fires when a row is inserted into a table.
The trigger procedure runs automatically on the INSERT event.

## Category
ROS 2 > Subscribers | Understanding: 3
```

### Narrator Service Architecture

**File:** `scripts/util/coding_narrator.py`

```python
# Core loop (simplified)
while True:
    if file_changed(coding_live.md):
        content = read_file()
        narration = extract_section("## What This Means")
        speak(narration)  # espeak TTS
        update_context(content)  # For follow-up questions
    sleep(0.5)
```

**Systemd Service:** `r2d2-narrator.service`
- Runs as background service
- Started/stopped by voice command
- NOT auto-started on boot (on-demand only)

### Narrator Modes

Control how much R2D2 talks during coding sessions:

| Mode | File Content | Behavior |
|------|--------------|----------|
| `narrator` | Speaks every update | Active learning, full narration |
| `quiet` | Silent | Focus mode, context saved for Q&A |
| `milestone` | Speaks on completions | Less interruption |

**Change mode:**
```bash
echo "quiet" > ~/dev/r2d2/data/narrator_mode.txt
```

### Follow-Up Questions

After R2D2 speaks an explanation, you can ask follow-up questions:

```
R2D2: "A state machine tracks transitions between states..."
You: [gesture] "Explain that more"
R2D2: "Sure! Think of it like an order status in your BI system. 
       An order can be: pending, processing, shipped, delivered. 
       The state machine enforces valid transitions..."
```

The speech node reads `coding_context.json` for context, so R2D2 knows what you're working on.

---

## Part 2: General Tutor Mode

### How It Works

```
[You ask: "Be my tutor"]
         │
         ▼
[R2D2 responds with [TUTOR_ON]]
         │
         ▼
[Speech node detects trigger]
         │
         ▼
[Teaching personality activated]
         │
         ▼
[R2D2 becomes patient, educational]
         │
         ▼
[Interactive Q&A conversation]
```

### Teaching Personality

When tutor mode is active, R2D2's personality shifts:

| Normal R2D2 | Tutor R2D2 |
|-------------|------------|
| Terse, efficient | Patient, thorough |
| Quick answers | Step-by-step explanations |
| Mission-focused | Learning-focused |
| "[beep] Done." | "Let me explain that..." |

### Example Tutor Conversation

```
You: "Teach me about ROS 2 services"

R2D2: "Great topic! A ROS 2 service is like a stored procedure 
       in your SQL database. You call it, wait for it to execute, 
       and get a result back. Unlike topics which are fire-and-forget,
       services are request-response. Does that make sense so far?"

You: "Yes, what's an example?"

R2D2: "In R2D2, the start_session service is like calling 
       sp_StartConversation - you request it, the system processes,
       and returns success or failure. Want me to show you the code?"
```

### Tutor Mode Features

- **Checks understanding**: "Does that make sense?"
- **Uses BI analogies**: Connects to your existing knowledge
- **Builds progressively**: Simple concepts first, then complexity
- **Interactive**: Responds to follow-up questions
- **Encouraging**: "Good question!", "You're getting it!"
- **Patient**: Never rushes, always explains fully

---

## Part 3: BI-to-Robotics Concept Mapping

Reference table for understanding robotics concepts through BI analogies:

| Robotics Concept | BI Analogy | R2D2 Example |
|------------------|------------|--------------|
| **ROS 2 Topic** | Database table / Event stream | `/r2d2/perception/person_id` = `person_events` table |
| **ROS 2 Subscriber** | SQL Trigger / ETL source | Callback fires on new frame = trigger on INSERT |
| **ROS 2 Publisher** | INSERT statement / Event producer | Publishing brightness = inserting a row |
| **ROS 2 Service** | Stored procedure / API call | `start_session` = `sp_StartConversation` |
| **ROS 2 Node** | ETL Job / Microservice | `image_listener` = always-running ETL process |
| **Launch File** | Job scheduler / Orchestrator | Like SSIS package starting multiple jobs |
| **Systemd Service** | SQL Agent Job / Scheduled task | Runs on boot, restarts on failure |
| **State Machine** | Workflow status / SCD Type 2 | Order: pending→processing→shipped |
| **Callback Function** | Trigger procedure | Code that runs when event fires |
| **Message Queue** | Message broker / Event hub | Topics buffer messages between nodes |
| **Parameter** | Config table / Environment variable | Runtime settings without code changes |
| **Hz (frequency)** | Polling interval / Refresh rate | 13 Hz = refreshes 13 times per second |
| **Latency** | Query response time | Time from input to output |
| **Transform (TF)** | Coordinate system / Foreign key | Relationships between reference frames |
| **Action** | Long-running stored procedure | Like sp_ProcessBatch with progress updates |
| **Lifecycle Node** | Job with init/start/stop phases | Configure→Activate→Deactivate→Cleanup |
| **QoS (Quality of Service)** | Transaction isolation level | Reliable = serializable, Best-effort = read uncommitted |
| **Executor** | Thread pool / Job executor | Manages which callbacks run when |
| **Bag file** | Database backup / Log replay | Recording of all topic messages for replay |

### Extended Analogies

#### ROS 2 Topics = Event Streams

```
BI World:
  INSERT INTO person_events (person_id, timestamp) VALUES (123, NOW());
  -- Other processes have triggers that react to this

ROS 2 World:
  publisher.publish(person_id_msg)
  -- Other nodes have subscribers that react to this
```

#### Lifecycle Nodes = ETL Job Phases

```
BI World (SSIS Package):
  1. OnPreValidate  → Configure
  2. OnPreExecute   → Activate
  3. Execute        → Active (running)
  4. OnPostExecute  → Deactivate
  5. OnCleanup      → Cleanup

ROS 2 World (Lifecycle Node):
  1. on_configure() → Configure
  2. on_activate()  → Activate
  3. (running)      → Active
  4. on_deactivate()→ Deactivate
  5. on_cleanup()   → Cleanup
```

---

## Part 4: Learning Progress Tracking

### Database Schema

**Location:** `~/dev/r2d2/data/persons.db`

```sql
-- Topics encountered during learning
CREATE TABLE learning_topics (
    id INTEGER PRIMARY KEY,
    category TEXT NOT NULL,        -- 'ROS 2', 'Python', 'Hardware'
    subcategory TEXT NOT NULL,     -- 'Subscribers', 'GPIO', 'State Machines'
    topic_name TEXT NOT NULL,      -- Specific concept
    first_seen TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    times_encountered INTEGER DEFAULT 1,
    understanding_level INTEGER,   -- 1-5 scale
    bi_analogy_used TEXT,          -- What analogy was used
    notes TEXT
);

-- Learning session summaries
CREATE TABLE learning_sessions (
    id INTEGER PRIMARY KEY,
    session_date TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    duration_minutes INTEGER,
    topics_covered TEXT,           -- JSON array of topic names
    mode TEXT,                     -- 'coding' or 'tutor'
    summary TEXT
);
```

### Understanding Levels

| Level | Meaning | Indicator |
|-------|---------|-----------|
| 1 | First exposure | Just introduced, needs review |
| 2 | Familiar | Seen multiple times, basic grasp |
| 3 | Understands basics | Can explain simply to others |
| 4 | Comfortable | Can apply it in practice |
| 5 | Mastered | Can teach others, deep understanding |

### Tracking APIs

```python
from person_registry import PersonRegistry

registry = PersonRegistry()

# Log a new topic
registry.log_learning_topic(
    category="ROS 2",
    subcategory="Subscribers", 
    topic_name="Callback pattern",
    understanding_level=3,
    bi_analogy="SQL Trigger"
)

# Log a session
registry.log_learning_session(
    duration_minutes=45,
    topics_covered=["Subscribers", "Publishers"],
    mode="coding",
    summary="Learned ROS 2 pub/sub basics"
)

# Query progress
topics = registry.get_learning_topics(category="ROS 2")
sessions = registry.get_learning_sessions(limit=10)
```

### Viewing Your Progress

```bash
# See all topics you've learned
sqlite3 ~/dev/r2d2/data/persons.db "SELECT category, subcategory, topic_name, understanding_level FROM learning_topics ORDER BY category;"

# See recent sessions
sqlite3 ~/dev/r2d2/data/persons.db "SELECT session_date, mode, summary FROM learning_sessions ORDER BY session_date DESC LIMIT 5;"

# Topics that need review (level < 3)
sqlite3 ~/dev/r2d2/data/persons.db "SELECT topic_name, understanding_level FROM learning_topics WHERE understanding_level < 3;"
```

---

## Part 5: System Configuration

### Configuration Files

| File | Location | Purpose |
|------|----------|---------|
| `speech_params.yaml` | `ros2_ws/src/r2d2_speech/config/` | LLM instructions, voice triggers |
| `narrator_mode.txt` | `data/` | Narrator verbosity (narrator/quiet/milestone) |
| `coding_live.md` | `data/` | Agent writes explanations here |
| `coding_context.json` | `data/` | Current context for follow-up Q&A |
| `tutor_mode_active.txt` | `data/` | Tutor mode state flag (true/false) |

### speech_params.yaml Key Settings

```yaml
# Learning Mode triggers
# [LEARNING_ON] / [LEARNING_OFF] → starts/stops narrator service

# Tutor Mode triggers  
# [TUTOR_ON] / [TUTOR_OFF] → activates teaching personality

# The LLM recognizes many variations:
# "learning mode", "teach me", "explain my code" → Learning ON
# "tutor mode", "be my tutor", "I want to learn" → Tutor ON
```

### Sudoers Configuration

For passwordless narrator service control (voice-activated):

```bash
# /etc/sudoers.d/r2d2-narrator
severin ALL=(ALL) NOPASSWD: /bin/systemctl start r2d2-narrator.service
severin ALL=(ALL) NOPASSWD: /bin/systemctl stop r2d2-narrator.service
```

**Setup command:**
```bash
sudo bash ~/dev/r2d2/scripts/setup/setup_narrator_sudoers.sh
```

---

## Part 6: Technical Implementation

### System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     Voice Input (Microphone)                     │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                     Speech Node (ROS 2)                          │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │ OpenAI GPT-4o with custom instructions                      ││
│  │ - Recognizes "learning mode" / "tutor mode" phrases         ││
│  │ - Includes [LEARNING_ON/OFF] or [TUTOR_ON/OFF] in response  ││
│  └─────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                   ros2_bridge.py - Trigger Detection             │
│  ┌──────────────────────┐    ┌──────────────────────┐          │
│  │ [LEARNING_ON/OFF]    │    │ [TUTOR_ON/OFF]       │          │
│  │ → systemctl start/   │    │ → Write true/false   │          │
│  │   stop narrator      │    │   to state file      │          │
│  └──────────────────────┘    └──────────────────────┘          │
└─────────────────────────────────────────────────────────────────┘
                │                              │
                ▼                              ▼
┌───────────────────────────┐    ┌───────────────────────────────┐
│   Narrator Service        │    │   Tutor Mode Active           │
│   (coding_narrator.py)    │    │   (tutor_mode_active.txt)     │
│                           │    │                               │
│   Watches coding_live.md  │    │   R2D2 uses teaching          │
│   Speaks via espeak       │    │   personality in responses    │
└───────────────────────────┘    └───────────────────────────────┘
```

### Trigger Detection Flow

**File:** `ros2_ws/src/r2d2_speech/r2d2_speech_ros/ros2_bridge.py`

```python
def process_learning_mode_triggers(text: str) -> str:
    """Process both learning and tutor mode triggers."""
    
    # Learning mode (narrator service)
    if "[LEARNING_ON]" in text:
        toggle_learning_mode(True)  # systemctl start
        text = text.replace("[LEARNING_ON]", "")
    elif "[LEARNING_OFF]" in text:
        toggle_learning_mode(False)  # systemctl stop
        text = text.replace("[LEARNING_OFF]", "")
    
    # Tutor mode (personality change)
    if "[TUTOR_ON]" in text:
        toggle_tutor_mode(True)  # Write to state file
        text = text.replace("[TUTOR_ON]", "")
    elif "[TUTOR_OFF]" in text:
        toggle_tutor_mode(False)  # Write to state file
        text = text.replace("[TUTOR_OFF]", "")
    
    return text
```

### Component Locations

| Component | File Path |
|-----------|-----------|
| Narrator script | `scripts/util/coding_narrator.py` |
| Narrator service | `scripts/systemd/r2d2-narrator.service` |
| Speech params | `ros2_ws/src/r2d2_speech/config/speech_params.yaml` |
| Bridge logic | `ros2_ws/src/r2d2_speech/r2d2_speech_ros/ros2_bridge.py` |
| REST speech node | `ros2_ws/src/r2d2_speech/r2d2_speech_ros/rest_speech_node.py` |
| Learning DB | `data/persons.db` |
| Live content | `data/coding_live.md` |
| Context file | `data/coding_context.json` |
| Narrator mode | `data/narrator_mode.txt` |
| Tutor state | `data/tutor_mode_active.txt` |

---

## Part 7: Troubleshooting

### Narrator Not Speaking

```bash
# Check service status
sudo systemctl status r2d2-narrator.service

# Check if file is being watched
cat ~/dev/r2d2/data/coding_live.md

# Check espeak is installed
which espeak

# Test espeak manually
espeak "Hello, this is R2D2"

# Check narrator mode
cat ~/dev/r2d2/data/narrator_mode.txt
# Should be: narrator (not quiet or milestone)
```

### Voice Commands Not Working

```bash
# Check speech node is running
sudo systemctl status r2d2-speech-node.service

# Check logs for trigger detection
journalctl -u r2d2-speech-node.service -f

# Verify trigger phrases in config
grep -A10 "LEARNING MODE" ~/dev/r2d2/ros2_ws/src/r2d2_speech/config/speech_params.yaml
grep -A10 "TUTOR MODE" ~/dev/r2d2/ros2_ws/src/r2d2_speech/config/speech_params.yaml
```

### Tutor Mode Not Activating

```bash
# Check tutor state file
cat ~/dev/r2d2/data/tutor_mode_active.txt

# Manually test by setting it
echo "true" > ~/dev/r2d2/data/tutor_mode_active.txt

# Check speech node logs for [TUTOR_ON] detection
journalctl -u r2d2-speech-node.service | grep -i tutor
```

### Learning Not Being Tracked

```bash
# Check database exists
ls -la ~/dev/r2d2/data/persons.db

# Verify tables exist
sqlite3 ~/dev/r2d2/data/persons.db ".tables"
# Should include: learning_topics, learning_sessions

# Query learning tables
sqlite3 ~/dev/r2d2/data/persons.db "SELECT * FROM learning_topics;"
sqlite3 ~/dev/r2d2/data/persons.db "SELECT * FROM learning_sessions;"
```

### Sudoers Permission Denied

```bash
# Check if sudoers file exists
ls -la /etc/sudoers.d/r2d2-narrator

# Re-run setup if missing
sudo bash ~/dev/r2d2/scripts/setup/setup_narrator_sudoers.sh

# Verify syntax
sudo visudo -c
```

---

## Part 8: For AI Agents (Internal Reference)

### When Working in Cursor with Learning Mode Active

1. **After each significant edit**, update `~/dev/r2d2/data/coding_live.md`
2. **Use the standard format** (Current Action, What This Means, BI Analogy, Category)
3. **Choose appropriate BI analogies** from the mapping table in Part 3
4. **Rate understanding level** honestly (1-5)
5. **Skip trivial changes** (typos, formatting, routine commands)

### When to Update coding_live.md

✅ **DO update** after:
- Editing a file (explain the change)
- Before running a significant command (explain what it does)
- Encountering an important new concept
- Completing a task (summarize what was learned)

❌ **DON'T update** for:
- Trivial changes (typos, formatting)
- Reading files (no action taken)
- Routine commands (cd, ls, cat)

### Example Agent Workflow

```
1. Receive task from user
2. Edit file
3. Write to coding_live.md:
   - What you did
   - Why it matters
   - BI analogy
4. R2D2 speaks explanation
5. User understands (or asks follow-up)
6. Continue to next step
7. At task end, log to learning database
```

### In-Line Explanation Style (for chat responses)

When explaining concepts directly in chat, use this format:

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

---

## Appendix A: Installation & Setup

### First-Time Setup

1. **Ensure narrator service file exists:**
   ```bash
   ls ~/dev/r2d2/scripts/systemd/r2d2-narrator.service
   ```

2. **Install and enable the narrator service:**
   ```bash
   sudo cp ~/dev/r2d2/scripts/systemd/r2d2-narrator.service /etc/systemd/system/
   sudo systemctl daemon-reload
   # Don't enable auto-start - it's on-demand only
   ```

3. **Set up passwordless control:**
   ```bash
   sudo bash ~/dev/r2d2/scripts/setup/setup_narrator_sudoers.sh
   ```

4. **Verify espeak is installed:**
   ```bash
   sudo apt-get install espeak
   ```

5. **Initialize data files:**
   ```bash
   mkdir -p ~/dev/r2d2/data
   echo "narrator" > ~/dev/r2d2/data/narrator_mode.txt
   echo "false" > ~/dev/r2d2/data/tutor_mode_active.txt
   echo "{}" > ~/dev/r2d2/data/coding_context.json
   ```

### Verify Installation

```bash
# Test narrator service can start
sudo systemctl start r2d2-narrator.service
sudo systemctl status r2d2-narrator.service
sudo systemctl stop r2d2-narrator.service

# Test espeak
espeak "R2D2 tutor system ready"

# Verify speech node has updated config
grep "TUTOR MODE" ~/dev/r2d2/ros2_ws/src/r2d2_speech/config/speech_params.yaml
```

---

## Appendix B: Comparison with Other Learning Tools

| Feature | R2D2 AI Tutor | Traditional Docs | Video Tutorials |
|---------|---------------|------------------|-----------------|
| Real-time | ✅ Yes | ❌ No | ❌ No |
| Interactive | ✅ Q&A | ❌ Static | ❌ One-way |
| Personalized | ✅ BI analogies | ❌ Generic | ❌ Generic |
| Context-aware | ✅ Knows your code | ❌ No | ❌ No |
| Hands-free | ✅ Voice | ❌ Reading | ❌ Watching |
| Progress tracking | ✅ SQLite | ❌ Manual | ❌ Manual |

---

## Appendix C: Future Enhancements

Potential improvements for the AI Tutor system:

1. **Learning Dashboard** - Web UI showing progress, topics mastered, areas needing review
2. **Spaced Repetition** - R2D2 periodically quizzes you on concepts that need reinforcement
3. **Project-Specific Tutorials** - AI-generated tutorials based on the R2D2 codebase
4. **Voice-Activated Progress Report** - "R2D2, how's my learning going?"
5. **Multi-User Support** - Track learning for different users

---

**Last Updated:** December 2025  
**Maintainer:** R2D2 Project  
**Related Docs:** 
- `200_SPEECH_SYSTEM_REFERENCE.md` - Speech system architecture
- `000_INTERNAL_AGENT_NOTES.md` - Agent instructions for learning mode
- `250_PERSON_MANAGEMENT_SYSTEM_REFERENCE.md` - Database schema

