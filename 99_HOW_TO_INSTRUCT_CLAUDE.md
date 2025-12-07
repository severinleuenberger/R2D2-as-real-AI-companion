# How to Instruct Claude in a New Chat Window

**Purpose:** Complete instructions for giving Claude new tasks on R2D2 project so they execute correctly every time.

**When to use this:** Starting a new chat window with Claude to work on R2D2 tasks.

---

## Minimal Checklist (For New Chat)

When starting a new Claude conversation for R2D2 work:

1. **Paste this at the START of the new chat:**
   ```
   I'm working on the R2D2 project in /home/severin/dev/r2d2
   
   READ THESE FIRST:
   - /home/severin/dev/r2d2/00_INTERNAL_AGENT_NOTES.md (CRITICAL git rules + setup)
   - /home/severin/dev/r2d2/README.md (project overview)
   
   My new task is:
   [PASTE YOUR TASK HERE]
   ```

2. **Ask Claude explicitly to read the meta-documents**

3. **Give task description with required outputs**

4. **Claude will acknowledge before starting**

---

## Complete Template (Copy-Paste Ready)

Paste this entire section into a new Claude chat, filling in `[TASK DESCRIPTION]`:

```
I'm working on the R2D2 project in /home/severin/dev/r2d2

CRITICAL: Please read these meta-documents FIRST:
1. /home/severin/dev/r2d2/00_INTERNAL_AGENT_NOTES.md
   - Contains CRITICAL git rules (⚠️ ALWAYS USE main BRANCH, NEVER master)
   - Contains hardware constants and environment setup
   - Contains common issues and solutions

2. /home/severin/dev/r2d2/README.md
   - Project overview
   - Architecture (4 ROS 2 packages)
   - Quick start guide

3. Additional docs to scan if relevant to task:
   - ARCHITECTURE_OVERVIEW.md (if building new features)
   - OPERATIONS_CHECKLIST.md (if troubleshooting)
   - INTEGRATION_GUIDE.md (if adding new ROS 2 packages)

MY TASK:
[PASTE YOUR TASK HERE]

REQUIRED OUTPUT:
[SPECIFY WHAT YOU WANT: test script? documentation? ROS 2 package? etc]
```

---

## Key Information to Include

When describing your task, provide:

### 1. **Hardware/Setup Context**
```
Example:
"I have wired a PAM8403 amplifier and 8Ω speaker as follows:
- Jetson pin 2 (5V) → PAM8403 +5V
- Jetson pin 6 (GND) → PAM8403 GND
- Jetson J511 pin 9 (HPO_L) → PAM8403 LIN
- etc."
```

### 2. **Specific Requirements (What must work)**
```
Example:
- Detect the correct ALSA audio device for HPO_L
- Configure ALSA mixer settings
- Create test_speaker.sh script
- Ensure settings survive reboot
```

### 3. **What to Document**
```
Example:
- Which ALSA card/device index is used
- Step-by-step configuration procedure
- Mixer settings (channel names, safe volumes)
- Test script with clear success/error messages
```

### 4. **Success Criteria**
```
Example:
- Audio plays on speaker when test script runs
- No errors in ALSA configuration
- Settings persist after reboot
```

---

## What Claude Will Do (Standard Process)

Once you provide a task:

1. **Read meta-documents** (if new chat or not recent conversation)
2. **Gather context** using tools (grep, read_file, run terminal commands)
3. **Analyze current state** (what exists, what's missing)
4. **Implement the task** (install packages, configure, test)
5. **Document findings** in appropriate file (01_, 02_, etc. or task-specific)
6. **Create any required artifacts** (scripts, configs, etc.)
7. **Verify with testing** (run commands, check output)
8. **Commit to main branch** (with detailed commit message)
9. **Push to GitHub** (after branch verification)

---

## Special Requests (How to Ask For Them)

### If you want documentation in specific file:
```
"Document the audio setup in a new file 07_AUDIO_SETUP_AND_CONFIGURATION.md"
```

### If you want a ROS 2 package:
```
"Create a new ROS 2 package r2d2_audio that publishes audio status and provides test services"
```

### If you want only analysis (no implementation):
```
"Analyze the audio configuration but DON'T implement yet - I want to review your plan first"
```

### If you want to verify before committing:
```
"Do the work but don't commit - I'll review first"
```

### If you want different branch:
```
"Do this on a separate branch audio-setup (but this is NOT normal - user only uses main)"
```

---

## What Meta-Documents Contain

### 00_INTERNAL_AGENT_NOTES.md
- ⚠️ CRITICAL git rules (main branch only, never master)
- Environment sourcing order (depthai → bash → ROS 2)
- Hardware constants (Jetson Orin, OAK-D camera specs)
- Performance baselines (30 FPS camera, 13 Hz perception)
- Common issues and solutions
- Test patterns

### README.md
- Project status (Phase 1: 85% complete)
- 16 functional requirements
- 4-phase roadmap
- Architecture overview
- Quick start (5 minutes)

### ARCHITECTURE_OVERVIEW.md
- Hardware specifications
- Software stack (4-layer diagram)
- Data flow (3 perspectives)
- ROS 2 topics reference
- Node architecture
- Processing pipeline
- Integration patterns for Phase 2-4

### OPERATIONS_CHECKLIST.md
- Startup procedures (environment order critical)
- Daily health checks
- 7-part troubleshooting guide
- Performance monitoring commands
- Quick reference cards

### INTEGRATION_GUIDE.md
- Step-by-step package creation
- ROS 2 node template
- Message type definitions
- Testing strategies
- Common patterns with code examples

---

## Example: Audio Setup Task (What You Just Gave)

Here's how to give your speaker/audio task in a new chat:

```
I'm working on the R2D2 project in /home/severin/dev/r2d2

CRITICAL: Please read these meta-documents FIRST:
1. /home/severin/dev/r2d2/00_INTERNAL_AGENT_NOTES.md
2. /home/severin/dev/r2d2/README.md

MY TASK:
I have wired an external PAM8403 class-D amplifier and 8Ω / 3W speaker:

Hardware connections:
- Jetson 40-pin header (J30):
  - Pin 2 (5V) → PAM8403 +5V
  - Pin 6 (GND) → PAM8403 GND (power)
- Jetson audio panel header (J511):
  - Pin 9 (HPO_L, left headphone out) → PAM8403 LIN
  - Pin 2 (AGND) → PAM8403 GND (audio)
- PAM8403 speaker output:
  - L+ → speaker +
  - L− → speaker −

REQUIRED WORK:
1. Detect correct ALSA device for HPO_L
2. Install audio tools if needed (alsa-utils, pulseaudio-utils, etc)
3. Configure ALSA/PulseAudio/PipeWire for analog output (no HDMI)
4. Set mixer settings: unmute DAC/headphone, safe volume, save on reboot
5. Create test_speaker.sh script with clear success/error messages

DOCUMENTATION:
- Document in new file 07_AUDIO_SETUP_AND_CONFIGURATION.md
- Include ALSA device mapping, mixer channel names, configuration steps
- Include test script

SUCCESS CRITERIA:
- Speaker plays audio when test script runs
- Settings persist after reboot
- No errors in ALSA configuration
```

---

## Common Mistakes to Avoid

### ❌ Mistake 1: Don't assume Claude remembers project details
**Wrong:** "Fix the audio output" (assumes context from old chat)
**Right:** Provide full hardware wiring, reference README.md and meta-docs

### ❌ Mistake 2: Don't skip meta-document references
**Wrong:** "Work on the Jetson audio" (no reference to docs)
**Right:** "Read 00_INTERNAL_AGENT_NOTES.md first, then work on audio"

### ❌ Mistake 3: Don't assume branch knowledge
**Wrong:** "Commit the changes"
**Right:** "Commit to main branch and push" (or explicitly stated in meta-docs)

### ❌ Mistake 4: Don't mix multiple unrelated tasks
**Wrong:** "Fix audio AND fix camera AND optimize perception"
**Right:** Break into separate chats or explicitly mark as separate tasks

### ❌ Mistake 5: Don't forget success criteria
**Wrong:** "Document the audio setup"
**Right:** "Document in 07_AUDIO_SETUP.md with: device mapping, mixer settings, test script"

---

## Testing Your Instructions (Self-Check)

Before hitting "Send" on a new chat, ask yourself:

- [ ] Did I reference the meta-documents (00_ file)?
- [ ] Did I specify required output format (script? doc? package)?
- [ ] Did I provide hardware/setup context if needed?
- [ ] Did I define success criteria (how will we know it works)?
- [ ] Did I mention git branch? (should always be main)
- [ ] Did I ask for documentation in specific file if needed?

---

## For Future Reference

This document itself is in `/home/severin/dev/r2d2/99_HOW_TO_INSTRUCT_CLAUDE.md`

When starting a new chat, you can reference it:
```
"I'm working on R2D2. See /home/severin/dev/r2d2/99_HOW_TO_INSTRUCT_CLAUDE.md for how to instruct me.

[YOUR TASK]
"
```

---

## Version History

| Date | Change |
|------|--------|
| Dec 7, 2025 | Initial version created with git branch rules, meta-doc references, templates |

