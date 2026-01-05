# Agent Instructions Archive - January 5, 2026

**Purpose:** This archive contains the original agent instruction files before the token optimization restructure.

**Date Archived:** January 5, 2026

**Reason:** Token usage optimization - reduced from ~17,000-21,000 tokens per query to ~600-800 tokens (Core Rules only).

---

## What Was Archived

### Files

1. **`000_INTERNAL_AGENT_NOTES_ORIGINAL.md`** (~1,257 lines, ~12,000-15,000 tokens)
   - Original comprehensive agent instructions
   - Contained all development guidance, architecture info, recent history, and tutorials

---

## What Replaced It

### New Structure (Tiered Loading)

**Always Loaded:**
- [`000_AGENT_CORE_RULES.md`](../../000_AGENT_CORE_RULES.md) (~200 lines, ~600-800 tokens)
  - Essential rules only
  - Security guidelines
  - Environment setup
  - Quick references

**Load on Demand:**
- [`AGENT_DEV_WORKFLOW.md`](../../AGENT_DEV_WORKFLOW.md) - Development and testing procedures
- [`AGENT_TUTOR_MODE.md`](../../AGENT_TUTOR_MODE.md) - Learning mode guidance (BI analogies, narration)
- [`000_AGENT_FINALIZATION_GUIDE.md`](../../000_AGENT_FINALIZATION_GUIDE.md) - Deployment procedures (condensed)

---

## Content Migration Map

| Original Content (lines) | New Location |
|--------------------------|--------------|
| UX Decision Protocol (32-100) | Core Rules (condensed) |
| AI Tutor System (103-279) | AGENT_TUTOR_MODE.md |
| Build & Test Phases (282-365) | AGENT_DEV_WORKFLOW.md |
| Execution Environment (368-428) | Core Rules + 103_TROUBLESHOOTING.md |
| Quick Commands (431-480) | AGENT_DEV_WORKFLOW.md |
| System Architecture (483-528) | DELETED (already in 001_ARCHITECTURE_OVERVIEW.md) |
| Hardware Reference (531-568) | DELETED (already in 002_HARDWARE_REFERENCE.md) |
| Doc Standards (571-878) | AGENT_DEV_WORKFLOW.md (condensed) |
| Service Management (880-952) | DELETED (already in 005_SYSTEMD_SERVICES_REFERENCE.md) |
| Recent History (955-1006) | This archive (historical context preserved) |
| File Location Rules (1057-1170) | DELETED (redundant with file system) |
| Security Rules (1172-1252) | Core Rules (condensed) |

---

## Key Changes During Migration

### Discrepancies Fixed (January 5, 2026)

1. **gesture_frame_skip:** Updated 5 docs to reflect code truth (default = 3, not 5)
2. **RED entry parameters:** Updated comments to reflect actual values (4 matches in 1.5s, not 3 in 1.0s)

### Improvements

- **Token Reduction:** 75-90% reduction in tokens per query
- **Established "Code is Truth" principle:** Docs reference code, never duplicate values
- **Modular Loading:** Task-specific guides loaded only when needed
- **Removed Redundancy:** Architecture/hardware info pointed to existing numbered docs

---

## Token Usage Comparison

| Scenario | Old Tokens | New Tokens | Savings |
|----------|------------|------------|---------|
| Development task | ~17,000-21,000 | ~2,000-2,500 | 85% |
| Finalization task | ~17,000-21,000 | ~1,500-2,000 | 90% |
| Learning mode | ~17,000-21,000 | ~1,200-1,500 | 92% |
| Simple query | ~17,000-21,000 | ~600-800 | 96% |

---

## Recent History Preserved Here

### December 27, 2025 - R2-D2 Mode Fully Tested & Working
- Status: ✅ Full end-to-end testing completed successfully
- Fixed: Gesture detection → REST speech service routing → multi-turn conversation loop
- Config: `silence_duration: 1.5s` for natural conversation flow
- Verified: open_hand (🖐️) triggers R2-D2 mode, fist (✊) stops, continuous turns work

### December 26, 2025 - R2-D2 Personality Mode
- Update: Renamed "Intelligent Mode" to "R2-D2 Mode" with authentic astromech personality
- Changes:
  - `intelligent_model`: `o1-preview` → `gpt-4o` (faster responses, ~2-3s)
  - `tts_voice`: `nova` → `echo` (robotic character)
  - `intelligent_instructions`: New terse, mission-oriented R2-D2 prompt with [beeps], [chirps]
- Character: Ultra-short responses, parenthetical sound flavor, occasionally sarcastic
- Documentation: `204_SPEECH_SYSTEM_VOICE_CONFIGURATION.md` fully updated

### December 25, 2025 - Dual-Mode Gesture Speech System
- Feature: Two speech modes triggered by different gestures
- Fast Mode (☝️ index finger): OpenAI Realtime API, chatty personality, ~1s latency
- R2-D2 Mode (🖐️ open hand): REST APIs (Whisper→gpt-4o→TTS), terse astromech personality, ~2-3s latency
- New Components:
  - `r2d2_speech/rest_api/rest_speech_client.py` - Turn-based STT→LLM→TTS pipeline
  - `rest_speech_node.py` - ROS2 lifecycle node for R2-D2 Mode
  - `r2d2-rest-speech-node.service` - Systemd service (auto-start enabled)
- Updated: gesture_intent_node routes open_hand to REST speech services
- Training: 3-gesture model (index_finger_up, fist, open_hand)
- Config: `speech_params.yaml` has personality configs for both modes

### December 24, 2025 - Script Organization
- All startup scripts moved to `scripts/start/` directory
- Systemd service files needed ExecStart path updates to match
- Complete service path audit in `005_SYSTEMD_SERVICES_REFERENCE.md`

---

## Status

**Archived:** Yes - Historical reference only  
**Active Use:** No - Replaced by new tiered structure  
**Recovery:** All content preserved or migrated to active docs

---

**End of Archive README**

