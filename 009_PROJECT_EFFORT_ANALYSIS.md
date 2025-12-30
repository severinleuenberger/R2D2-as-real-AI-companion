# 009 R2D2 Project Effort Analysis

**Date:** December 29, 2025  
**Purpose:** Tools and methodology for analyzing project statistics and development effort  
**Audience:** Developers, project managers, stakeholders  

---

## Overview

This document describes the automated tools for generating project statistics on-demand. Rather than maintaining static snapshots that go stale, use the `project_stats.py` script to generate current metrics anytime.

**What this system provides:**
- Git repository statistics (commits, contributors, history)
- Code metrics (lines of code, file counts by language)
- Documentation metrics (markdown files and lines)
- Derived metrics (velocity, ratios, intensity indicators)

**Use cases:**
- Monthly progress reports
- Stakeholder presentations
- GitHub README updates
- Project milestone tracking
- Comparing effort across phases

---

## Quick Start

```bash
# From project root
cd ~/dev/r2d2
python tools/project_stats.py
```

**Output formats:**
```bash
python tools/project_stats.py              # Terminal output (default)
python tools/project_stats.py --markdown   # Generate markdown
python tools/project_stats.py --json       # Generate JSON
python tools/project_stats.py --all        # All formats (saves to _TEMP/)
python tools/project_stats.py --quiet      # Suppress progress messages
```

---

## Available Metrics

### Git Statistics

| Metric | Description | Example |
|--------|-------------|---------|
| Total Commits | All commits in repository history | 335 |
| First Commit Date | Date of initial commit | 2025-12-04 |
| Last Commit Date | Date of most recent commit | 2025-12-29 |
| Project Duration | Days from first to last commit | 26 days |
| Commits per Day | Average development velocity | 12.9 |
| Contributors | List of contributors with commit counts | See output |
| Total Insertions | Lines added across all commits | 7,954,100 |
| Total Deletions | Lines removed across all commits | 7,353,296 |
| Net Growth | Current codebase size vs empty repo | 578,143 lines |
| Recent Commits | Last 5 commit messages | See output |

### Code Metrics

| Metric | Description | Included Extensions |
|--------|-------------|---------------------|
| Active Code Lines | Non-empty lines in source files | `.py`, `.js`, `.ts`, `.tsx`, `.cpp`, `.c`, `.h`, `.hpp` |
| Code Files | Number of source code files | Same as above |
| By Extension | Breakdown by file type | Lines and files per extension |

### Documentation Metrics

| Metric | Description |
|--------|-------------|
| Documentation Lines | Non-empty lines in markdown files |
| Documentation Files | Number of `.md` files |
| Docs-to-Code Ratio | Percentage of documentation vs code |

### Derived Metrics

| Metric | Description | Interpretation |
|--------|-------------|----------------|
| Iteration Rate | Ratio of insertions to deletions | >1.0 indicates active refinement |
| Development Intensity | Classification based on velocity | Very High, High, Moderate, Low, Minimal |

---

## Output Format Examples

### Terminal Output

```
════════════════════════════════════════════════════
  R2D2 PROJECT STATISTICS
  Generated: 2025-12-29 09:25:48
════════════════════════════════════════════════════

GIT METRICS
────────────────────────────────────────────────────
Total Commits:              335
Project Duration:           26 days
First Commit:               2025-12-04
Last Commit:                2025-12-29
Average Velocity:           12.9 commits/day
Contributors:               3
  - Severin Leuenberger: 313
  - severinleuenberger: 20
  - copilot-swe-agent[bot]: 2

Total Insertions:           7,954,100
Total Deletions:            7,353,296
Net Growth:                 578,143 lines

CODE METRICS
────────────────────────────────────────────────────
Active Code Lines:          21,540
Code Files:                 136

DOCUMENTATION METRICS
────────────────────────────────────────────────────
Documentation Lines:        26,345
Documentation Files:        87
Docs-to-Code Ratio:         122.3%

ANALYSIS
────────────────────────────────────────────────────
High iteration rate (1.08:1 insertions/deletions)
Excellent documentation ratio (122.3%)
Development intensity: Very High (intensive sprint)
```

### Markdown Output

Use `--markdown` flag for GitHub-compatible tables:

```markdown
| Metric | Value |
|--------|-------|
| Total Commits | 335 |
| Project Duration | 26 days |
| Development Velocity | 12.9 commits/day |
| Active Code Lines | 21,540 |
...
```

### JSON Output

Use `--json` flag for machine-readable format:

```json
{
  "generated": "2025-12-29T09:26:45.994756",
  "git": {
    "total_commits": 335,
    "first_commit_date": "2025-12-04",
    ...
  },
  "code": {
    "total_lines": 21540,
    "total_files": 136,
    ...
  },
  ...
}
```

---

## Interpreting Results

### Development Velocity

| Commits/Day | Classification | Typical Scenario |
|-------------|----------------|------------------|
| >= 10 | Very High | Intensive sprint, full-time+ development |
| 5-10 | High | Active full-time development |
| 2-5 | Moderate | Steady part-time progress |
| 0.5-2 | Low | Part-time or periodic development |
| < 0.5 | Minimal | Maintenance mode |

### Documentation Ratio

| Ratio | Assessment | Notes |
|-------|------------|-------|
| >= 10% | Excellent | Well-documented project |
| 5-10% | Good | Adequate documentation |
| 2-5% | Fair | Documentation could improve |
| < 2% | Poor | Needs documentation effort |

### Iteration Rate

| Ratio | Interpretation |
|-------|----------------|
| > 1.5 | Very high churn - rapid prototyping or refactoring |
| 1.0-1.5 | Normal development with refinement |
| 0.7-1.0 | Stable codebase with additions |
| < 0.7 | More deletions than additions - cleanup phase |

---

## Exclusion Patterns

The script excludes certain directories from code/documentation counting to focus on active source code:

### Excluded Directories

| Directory | Reason |
|-----------|--------|
| `_ARCHIVE/` | Historical/deprecated files |
| `_TEMP/` | Temporary work files |
| `_DEBUG/` | Debug artifacts |
| `_ANALYSIS_AND_DOCUMENTATION/` | Supplementary docs (counted separately if needed) |
| `build/`, `install/`, `log/` | ROS 2 build artifacts |
| `node_modules/` | NPM dependencies |
| `__pycache__/` | Python bytecode cache |
| `.git/` | Git metadata |
| `*_env/` | Python virtual environments |

### Why Exclude Build Artifacts?

The `ros2_ws/build/` and `ros2_ws/install/` directories contain **generated code** that is:
- Not written by developers
- Automatically created during `colcon build`
- Duplicated from source files

Including these would inflate code metrics by 10-20x and misrepresent actual development effort.

---

## Manual Commands (Fallback)

If the script is unavailable, use these commands directly:

### Git Statistics

```bash
# Total commits
git log --oneline --all | wc -l

# First commit date
git log --format='%ai' | tail -1

# Last commit date
git log --format='%ai' -1

# Contributors
git log --all --format='%an' | sort | uniq -c | sort -rn

# Total insertions/deletions
git log --all --shortstat --format= | grep -E "insertion|deletion" | \
  awk '{ins+=$4; del+=$6} END {print "Insertions:", ins, "Deletions:", del}'

# Net growth (current state)
git diff --shortstat $(git hash-object -t tree /dev/null) HEAD
```

### Code Counting

```bash
# Count Python/JS/TS lines (with exclusions)
find . -type f \( -name "*.py" -o -name "*.js" -o -name "*.ts" \) \
  -not -path "./_ARCHIVE/*" \
  -not -path "*/build/*" \
  -not -path "*/install/*" \
  -not -path "*/node_modules/*" \
  -not -path "*_env/*" \
  | xargs wc -l | tail -1

# Count files
find . -type f \( -name "*.py" -o -name "*.js" -o -name "*.ts" \) \
  [same exclusions] | wc -l
```

### Documentation Counting

```bash
# Count markdown lines
find . -type f -name "*.md" \
  -not -path "./_ARCHIVE/*" \
  -not -path "./.git/*" \
  | xargs wc -l | tail -1

# Count files
find . -type f -name "*.md" \
  -not -path "./_ARCHIVE/*" \
  -not -path "./.git/*" \
  | wc -l
```

---

## Installation & Dependencies

### Requirements

The script uses only Python standard library - no additional dependencies required.

**Optional (for enhanced output):**
- `rich` - Beautiful terminal formatting (falls back to basic if unavailable)

### Installation

The script is already included in the project at `tools/project_stats.py`:

```bash
# Verify it's executable
ls -la ~/dev/r2d2/tools/project_stats.py

# Run directly
python ~/dev/r2d2/tools/project_stats.py
# or
./tools/project_stats.py  # if executable flag set
```

---

## Best Practices

### When to Generate Statistics

- **Monthly:** For progress tracking and reports
- **Phase milestones:** At completion of each project phase
- **Before presentations:** For stakeholder updates
- **After major refactoring:** To measure impact

### Archiving Snapshots

For historical tracking, save snapshots periodically:

```bash
# Generate and save with timestamp
python tools/project_stats.py --json > _TEMP/stats_$(date +%Y%m%d).json
python tools/project_stats.py --markdown > _TEMP/stats_$(date +%Y%m%d).md
```

Note: `_TEMP/` is gitignored, so these won't be committed. For permanent archival, move to `_ARCHIVE/`.

### Comparing Phases

Generate statistics at phase completion and compare:

| Phase | Code Lines | Doc Lines | Commits | Duration |
|-------|------------|-----------|---------|----------|
| Phase 1 | ~8,000 | ~15,000 | ~150 | 10 days |
| Phase 2 | ~21,000 | ~26,000 | ~335 | 26 days |
| Phase 3 | (TBD) | (TBD) | (TBD) | (TBD) |

---

## Troubleshooting

### "Not a git repository"

**Cause:** Running from outside project directory  
**Solution:** `cd ~/dev/r2d2` before running script

### Statistics seem wrong

**Check exclusions:** Run with verbose mode to see what's being counted:
```bash
# Verify file list
find . -name "*.py" -not -path "*/build/*" -not -path "*/install/*" | head -20
```

### Script takes too long

**Cause:** Large repository with many files  
**Solution:** The script processes sequentially; wait for completion or use `--quiet` to reduce output

### Git statistics incomplete

**Cause:** Detached HEAD or partial clone  
**Solution:** Ensure you're on a branch with full history:
```bash
git fetch --all
git checkout main
```

---

## Script Location

```
~/dev/r2d2/tools/project_stats.py
```

**Related files:**
- `tools/minimal_monitor.py` - System resource monitoring
- `tools/quick_status.sh` - Quick ROS 2 status check

---

## Version History

| Date | Change |
|------|--------|
| 2025-12-29 | Initial creation with terminal, markdown, JSON output |

---

**See also:**
- `001_ARCHITECTURE_OVERVIEW.md` - System architecture
- `000_INTERNAL_AGENT_NOTES.md` - Development guidelines
- `README.md` - Project overview



