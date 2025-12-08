# How to Instruct Claude on R2D2 Tasks

**Purpose:** Simple directive for giving Claude new tasks.

**Key Rule:** Claude should read the 000-004 meta documents to fully understand the project context, then complete your task.

---

## When Starting a New Chat

Simply tell Claude:

```
I'm working on the R2D2 project in /home/severin/dev/r2d2

Please read these foundation documents first to understand the full context:
- 000_INTERNAL_AGENT_NOTES.md (CRITICAL: git rules, environment setup, hardware constants)
- 001_ARCHITECTURE_OVERVIEW.md (system architecture and design)
- 003_JETSON_FLASHING_AND_DISPLAY_SETUP.md (hardware setup if relevant)
- 004_BACKUP_AND_RESTORE.md (for recovery procedures if needed)

You can also reference any 010-089 implementation documents and START_HERE guides as needed.
For organizational/analysis documents, see _ANALYSIS_AND_DOCUMENTATION/ folder.

My task:
[YOUR TASK HERE]

What I need:
[WHAT YOU WANT AS OUTPUT]
```

---

## What Claude Will Do

Claude will:
1. Read the 000-004 meta documents for context
2. Review relevant 010-089 implementation docs
3. Check START_HERE guides if needed
4. Access _ANALYSIS_AND_DOCUMENTATION/ for detailed background if necessary
5. Complete your task with full project understanding

---

## That's It

Don't overthink it. Just describe your task clearly, and Claude will read the documents to understand the context fully.

The project meta-documents (000-004) provide everything needed:
- **000:** Critical rules and environment
- **001:** System architecture
- **002:** This directive (what you're reading)
- **003:** Hardware setup procedures
- **004:** Backup and restore

Everything else is implementation details that Claude can reference as needed.
