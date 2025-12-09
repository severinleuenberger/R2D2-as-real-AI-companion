# How to Instruct Claude on R2D2 Tasks

**Purpose:** Simple directive for giving Claude new tasks.

**Key Rule:** Claude should read the 000-004 meta documents to fully understand the project context, then complete your task.

---

## When Starting a New Chat

Simply tell Claude:

```
I'm working on the R2D2 project in /home/severin/dev/r2d2

Please read these foundation documents in order to understand the full context:

**Start with the overview:**
1. README.md (understand the overall goal and project vision)

**Then the critical meta-documents:**
2. 000_INTERNAL_AGENT_NOTES.md (CRITICAL: git rules, environment setup, hardware constants)
3. 001_ARCHITECTURE_OVERVIEW.md (system architecture and design)

**Project-specific documentation as relevant:**
4. 003_JETSON_FLASHING_AND_DISPLAY_SETUP.md (if working on hardware setup)
5. 004_BACKUP_AND_RESTORE.md (if working on recovery procedures)

**For deeper context:**
- 010-089 implementation documents (project-specific features)
- START_HERE guides (feature-specific guidance)
- _ANALYSIS_AND_DOCUMENTATION/ folder (organizational/analysis docs)

My task:
[YOUR TASK HERE]

What I need:
[WHAT YOU WANT AS OUTPUT]
```

---

## What Claude Will Do

Claude will:
1. Read README.md for the overall project vision and goals
2. Review 000_INTERNAL_AGENT_NOTES.md for critical rules and environment
3. Review 001_ARCHITECTURE_OVERVIEW.md for system architecture
4. Check project-specific docs (003, 004, etc.) as relevant to the task
5. Reference 010-089 implementation docs and START_HERE guides as needed
6. Access _ANALYSIS_AND_DOCUMENTATION/ for detailed background if necessary
7. Complete your task with full project understanding

---

## That's It

Don't overthink it. Just describe your task clearly, and Claude will read the documents to understand the context fully.

The reading order matters:
1. **README.md:** Project vision and overall goals
2. **000-001:** Critical infrastructure and architecture
3. **Project-specific docs (003-004):** Only as relevant to your task
4. **Implementation details (010-089):** Feature-specific guidance
5. **Analysis docs:** Deep background if needed
