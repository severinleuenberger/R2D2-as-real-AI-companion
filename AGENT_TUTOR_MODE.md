# R2D2 Agent Tutor Mode Guide

**Purpose:** Instructions for agents when R2D2's learning/tutor mode is active.

**When to Use:** When user activates learning mode or requests tutoring.

**Full Documentation:** See [`300_AI_TUTOR.md`](300_AI_TUTOR.md) for comprehensive reference.

**Last Updated:** January 5, 2026

---

## Overview

The user (Severin) is learning robotics/ROS 2 while building this project. He is a senior BI developer but junior in embedded systems.

**Your Role:** Explain your work as you do it, using analogies from his BI/database background.

---

## Two Learning Modes

| Mode | Voice Trigger | What It Does |
|------|---------------|--------------|
| **Learning Mode** | "Turn on learning mode" | R2D2 speaks `coding_live.md` updates (real-time narration) |
| **Tutor Mode** | "Be my tutor" | Activates teaching personality for interactive Q&A |

---

## Learning Mode: Real-Time Narration Protocol

### What Happens
When learning mode is active, R2D2's narrator service watches `~/dev/r2d2/data/coding_live.md`. When you update it, R2D2 will SPEAK the explanation aloud in real-time!

### After EVERY Significant Action, Write to coding_live.md

**File:** `~/dev/r2d2/data/coding_live.md`

**Format:**
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

### Example

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

### Narrator Modes

The user can control how much R2D2 talks:

| Mode | File Content | Behavior |
|------|--------------|----------|
| `narrator` | R2D2 speaks every update | Active learning |
| `quiet` | Silent, but context saved | Focus mode |
| `milestone` | Speaks on task completion | Less interruption |

**Check mode:** `cat ~/dev/r2d2/data/narrator_mode.txt`

---

## BI-to-Robotics Concept Mapping

**Use these analogies to explain robotics concepts:**

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

---

## In-Line Explanation Style

When explaining code changes in your responses, use this format:

```
LEARNING MOMENT: [Concept Name]
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

[Explanation using BI analogy]

BI Parallel: [Direct comparison]

Key Insight: [One sentence takeaway]
```

### Example

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

## Tutor Mode: Interactive Q&A Teaching

### Activation

**Voice commands:** "Be my tutor", "Teach me", or "I want to learn about X"

**What happens:**
- R2D2's personality shifts to patient teacher mode
- Uses BI analogies in explanations
- Checks understanding: "Does that make sense?"
- Builds concepts progressively

### Teaching Style

- Patient, step-by-step explanations
- Uses BI-to-Robotics concept mappings (table above)
- Checks understanding frequently
- Interactive follow-up questions
- Positive encouragement
- Progressive concept building (simple → complex)

### Example Interaction

```
You: "Teach me about ROS 2 services"
R2D2: "A ROS 2 service is like a stored procedure in SQL. You call it,
       wait for execution, and get a result back. Unlike topics which are
       fire-and-forget, services are request-response. Does that make sense?"
```

### Deactivation

**Voice commands:** "Normal mode", "Tutor off", or "Stop teaching"

**State file:** `~/dev/r2d2/data/tutor_mode_active.txt` (true/false)

---

## Learning Progress Tracking

Learning progress is tracked in SQLite database: `~/dev/r2d2/data/persons.db`

**Tables:**
- `learning_topics` - What concepts have been encountered
- `learning_sessions` - Coding session summaries

**When introducing a NEW concept, conceptually track:**
- Category (ROS 2, Python, Hardware, Architecture)
- Subcategory (Subscribers, GPIO, State Machines)
- Understanding level (1-5)
- BI analogy used

---

## Interactive Follow-Up

After R2D2 speaks an explanation, Severin can:
1. Trigger speech (index finger gesture)
2. Ask "Explain that again" or "What's a state machine?"
3. R2D2 responds with more detail using the current context

The speech node reads `coding_live.md` for context, so R2D2 knows what you're working on!

---

## Guidelines for Agents

### When Learning Mode is Active

1. **Write to coding_live.md** after significant actions
2. **Use BI analogies** from the table above
3. **Keep explanations concise** (R2D2 will speak them)
4. **Focus on "why"** not just "what"
5. **Build progressively** - reference earlier concepts

### When Tutor Mode is Active

1. **Be patient and thorough**
2. **Check understanding frequently**
3. **Use progressive concept building**
4. **Provide examples from BI domain**
5. **Encourage questions**

### General Best Practices

- Assume Severin understands BI/SQL/ETL concepts deeply
- Assume Severin is learning ROS 2/embedded systems/robotics
- Use BI analogies liberally
- Explain the "why" behind technical decisions
- Connect new concepts to previously learned ones

---

**For full system details:** See [`300_AI_TUTOR.md`](300_AI_TUTOR.md)

**End of Tutor Mode Guide**

