#!/usr/bin/env python3
"""
R2D2 Project Statistics Tool

Generates comprehensive project statistics including:
- Git repository metrics (commits, contributors, insertions/deletions)
- Code metrics (lines of code, file counts by language)
- Documentation metrics (markdown lines and files)
- Derived metrics (velocity, duration, ratios)

Usage:
    python tools/project_stats.py              # Terminal output
    python tools/project_stats.py --markdown   # Generate markdown
    python tools/project_stats.py --json       # Generate JSON
    python tools/project_stats.py --all        # All formats

Author: R2D2 Project
Date: December 2025
"""

import argparse
import json
import os
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Tuple, Any

# =============================================================================
# CONFIGURATION
# =============================================================================

# Directories to exclude from code/doc counting
EXCLUDE_DIRS = [
    '_ARCHIVE',
    '_TEMP',
    '_DEBUG',
    '_ANALYSIS_AND_DOCUMENTATION',
    'archive',
    'build',
    'install',
    'log',
    'node_modules',
    '__pycache__',
    '.git',
    'depthai_env',
    'r2d2_speech_env',
]

# File extensions for code files
CODE_EXTENSIONS = ['.py', '.js', '.ts', '.tsx', '.cpp', '.c', '.h', '.hpp']

# File extensions for documentation
DOC_EXTENSIONS = ['.md']


# =============================================================================
# GIT STATISTICS
# =============================================================================

def run_git_command(cmd: List[str], cwd: str = None) -> str:
    """Run a git command and return output."""
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            cwd=cwd,
            timeout=60
        )
        return result.stdout.strip()
    except subprocess.TimeoutExpired:
        return ""
    except Exception as e:
        print(f"Warning: Git command failed: {e}", file=sys.stderr)
        return ""


def get_git_stats(project_root: str) -> Dict[str, Any]:
    """Get comprehensive git statistics."""
    stats = {
        'total_commits': 0,
        'first_commit_date': None,
        'last_commit_date': None,
        'project_duration_days': 0,
        'commits_per_day': 0.0,
        'contributors': [],
        'total_insertions': 0,
        'total_deletions': 0,
        'net_growth': 0,
        'recent_commits': [],
    }
    
    # Total commits
    output = run_git_command(['git', 'log', '--oneline', '--all'], project_root)
    if output:
        stats['total_commits'] = len(output.split('\n'))
    
    # First and last commit dates (use shell piping for correct results)
    last_date = run_git_command(['git', 'log', '--format=%ai', '-1'], project_root)
    
    # Get first commit: all dates piped through tail
    try:
        result = subprocess.run(
            'git log --format=%ai | tail -1',
            shell=True,
            capture_output=True,
            text=True,
            cwd=project_root,
            timeout=60
        )
        first_date = result.stdout.strip()
    except Exception:
        first_date = ""
    
    if last_date:
        stats['last_commit_date'] = last_date.split()[0]
    if first_date:
        stats['first_commit_date'] = first_date.split()[0]
    
    # Calculate project duration
    if stats['first_commit_date'] and stats['last_commit_date']:
        try:
            first = datetime.strptime(stats['first_commit_date'], '%Y-%m-%d')
            last = datetime.strptime(stats['last_commit_date'], '%Y-%m-%d')
            stats['project_duration_days'] = (last - first).days + 1
            if stats['project_duration_days'] > 0:
                stats['commits_per_day'] = round(
                    stats['total_commits'] / stats['project_duration_days'], 1
                )
        except ValueError:
            pass
    
    # Contributors
    output = run_git_command(
        ['git', 'log', '--all', '--format=%an'],
        project_root
    )
    if output:
        contributor_counts = {}
        for name in output.split('\n'):
            name = name.strip()
            if name:
                contributor_counts[name] = contributor_counts.get(name, 0) + 1
        # Sort by commit count descending
        stats['contributors'] = sorted(
            contributor_counts.items(),
            key=lambda x: x[1],
            reverse=True
        )
    
    # Total insertions/deletions across all history
    output = run_git_command(
        ['git', 'log', '--all', '--shortstat', '--format='],
        project_root
    )
    if output:
        for line in output.split('\n'):
            if 'insertion' in line or 'deletion' in line:
                parts = line.split(',')
                for part in parts:
                    part = part.strip()
                    if 'insertion' in part:
                        try:
                            stats['total_insertions'] += int(part.split()[0])
                        except (ValueError, IndexError):
                            pass
                    elif 'deletion' in part:
                        try:
                            stats['total_deletions'] += int(part.split()[0])
                        except (ValueError, IndexError):
                            pass
    
    # Net growth (current state vs empty repo)
    output = run_git_command(
        ['git', 'diff', '--shortstat', '4b825dc642cb6eb9a060e54bf8d69288fbee4904', 'HEAD'],
        project_root
    )
    if output and 'insertion' in output:
        parts = output.split(',')
        for part in parts:
            part = part.strip()
            if 'insertion' in part:
                try:
                    stats['net_growth'] = int(part.split()[0])
                except (ValueError, IndexError):
                    pass
    
    # Recent commits (last 5)
    output = run_git_command(
        ['git', 'log', '--oneline', '-5', '--format=%h %s'],
        project_root
    )
    if output:
        stats['recent_commits'] = output.split('\n')
    
    return stats


# =============================================================================
# CODE AND DOCUMENTATION COUNTING
# =============================================================================

def should_exclude(path: Path, project_root: Path) -> bool:
    """Check if a path should be excluded from counting."""
    rel_path = path.relative_to(project_root)
    path_str = str(rel_path)
    parts = rel_path.parts
    
    for part in parts:
        if part in EXCLUDE_DIRS:
            return True
        # Also exclude virtual environments with _env suffix
        if part.endswith('_env'):
            return True
    
    # Additional pattern checks for nested directories
    exclude_patterns = [
        'ros2_ws/build',
        'ros2_ws/install',
        'ros2_ws/log',
        'web_dashboard/node_modules',
    ]
    for pattern in exclude_patterns:
        if pattern in path_str:
            return True
    
    return False


def count_lines_in_file(filepath: Path) -> int:
    """Count non-empty lines in a file."""
    try:
        with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
            return sum(1 for line in f if line.strip())
    except Exception:
        return 0


def count_code_files(project_root: str) -> Dict[str, Any]:
    """Count code files and lines, organized by extension."""
    root_path = Path(project_root)
    stats = {
        'total_lines': 0,
        'total_files': 0,
        'by_extension': {},
    }
    
    for ext in CODE_EXTENSIONS:
        stats['by_extension'][ext] = {'lines': 0, 'files': 0}
    
    for ext in CODE_EXTENSIONS:
        for filepath in root_path.rglob(f'*{ext}'):
            if should_exclude(filepath, root_path):
                continue
            if filepath.is_file():
                lines = count_lines_in_file(filepath)
                stats['total_lines'] += lines
                stats['total_files'] += 1
                stats['by_extension'][ext]['lines'] += lines
                stats['by_extension'][ext]['files'] += 1
    
    return stats


def count_documentation_files(project_root: str) -> Dict[str, Any]:
    """Count documentation files and lines."""
    root_path = Path(project_root)
    stats = {
        'total_lines': 0,
        'total_files': 0,
    }
    
    for ext in DOC_EXTENSIONS:
        for filepath in root_path.rglob(f'*{ext}'):
            if should_exclude(filepath, root_path):
                continue
            if filepath.is_file():
                lines = count_lines_in_file(filepath)
                stats['total_lines'] += lines
                stats['total_files'] += 1
    
    return stats


# =============================================================================
# DERIVED METRICS
# =============================================================================

def calculate_derived_metrics(
    git_stats: Dict[str, Any],
    code_stats: Dict[str, Any],
    doc_stats: Dict[str, Any]
) -> Dict[str, Any]:
    """Calculate derived metrics from raw stats."""
    derived = {
        'docs_to_code_ratio': 0.0,
        'iteration_rate': 0.0,
        'development_intensity': 'unknown',
    }
    
    # Documentation to code ratio
    if code_stats['total_lines'] > 0:
        derived['docs_to_code_ratio'] = round(
            (doc_stats['total_lines'] / code_stats['total_lines']) * 100, 1
        )
    
    # Iteration rate (insertions / deletions)
    if git_stats['total_deletions'] > 0:
        derived['iteration_rate'] = round(
            git_stats['total_insertions'] / git_stats['total_deletions'], 2
        )
    
    # Development intensity classification
    commits_per_day = git_stats['commits_per_day']
    if commits_per_day >= 10:
        derived['development_intensity'] = 'Very High (intensive sprint)'
    elif commits_per_day >= 5:
        derived['development_intensity'] = 'High (active development)'
    elif commits_per_day >= 2:
        derived['development_intensity'] = 'Moderate (steady progress)'
    elif commits_per_day >= 0.5:
        derived['development_intensity'] = 'Low (part-time development)'
    else:
        derived['development_intensity'] = 'Minimal (maintenance mode)'
    
    return derived


# =============================================================================
# OUTPUT FORMATTERS
# =============================================================================

def format_number(n: int) -> str:
    """Format number with thousands separator."""
    return f"{n:,}"


def format_terminal_output(
    git_stats: Dict[str, Any],
    code_stats: Dict[str, Any],
    doc_stats: Dict[str, Any],
    derived: Dict[str, Any]
) -> str:
    """Generate terminal output with formatting."""
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    
    lines = []
    lines.append("")
    lines.append("═" * 52)
    lines.append("  R2D2 PROJECT STATISTICS")
    lines.append(f"  Generated: {timestamp}")
    lines.append("═" * 52)
    lines.append("")
    
    # Git Metrics
    lines.append("GIT METRICS")
    lines.append("─" * 52)
    lines.append(f"Total Commits:              {format_number(git_stats['total_commits'])}")
    lines.append(f"Project Duration:           {git_stats['project_duration_days']} days")
    lines.append(f"First Commit:               {git_stats['first_commit_date']}")
    lines.append(f"Last Commit:                {git_stats['last_commit_date']}")
    lines.append(f"Average Velocity:           {git_stats['commits_per_day']} commits/day")
    lines.append(f"Contributors:               {len(git_stats['contributors'])}")
    
    for name, count in git_stats['contributors'][:5]:  # Top 5
        lines.append(f"  - {name}: {format_number(count)}")
    
    lines.append("")
    lines.append(f"Total Insertions:           {format_number(git_stats['total_insertions'])}")
    lines.append(f"Total Deletions:            {format_number(git_stats['total_deletions'])}")
    lines.append(f"Net Growth:                 {format_number(git_stats['net_growth'])} lines")
    lines.append("")
    
    # Code Metrics
    lines.append("CODE METRICS")
    lines.append("─" * 52)
    lines.append(f"Active Code Lines:          {format_number(code_stats['total_lines'])}")
    lines.append(f"Code Files:                 {format_number(code_stats['total_files'])}")
    lines.append("")
    lines.append("By Extension:")
    for ext, data in sorted(code_stats['by_extension'].items(), 
                           key=lambda x: x[1]['lines'], reverse=True):
        if data['files'] > 0:
            lines.append(f"  {ext:6} {format_number(data['files']):>6} files, "
                        f"{format_number(data['lines']):>10} lines")
    lines.append("")
    
    # Documentation Metrics
    lines.append("DOCUMENTATION METRICS")
    lines.append("─" * 52)
    lines.append(f"Documentation Lines:        {format_number(doc_stats['total_lines'])}")
    lines.append(f"Documentation Files:        {format_number(doc_stats['total_files'])}")
    lines.append(f"Docs-to-Code Ratio:         {derived['docs_to_code_ratio']}%")
    lines.append("")
    
    # Analysis
    lines.append("ANALYSIS")
    lines.append("─" * 52)
    
    # Iteration rate interpretation
    if derived['iteration_rate'] > 1.0:
        lines.append(f"High iteration rate ({derived['iteration_rate']}:1 insertions/deletions)")
    else:
        lines.append(f"Iteration rate: {derived['iteration_rate']}:1 (insertions/deletions)")
    
    # Documentation quality
    if derived['docs_to_code_ratio'] >= 8:
        lines.append(f"Excellent documentation ratio ({derived['docs_to_code_ratio']}%)")
    elif derived['docs_to_code_ratio'] >= 5:
        lines.append(f"Good documentation ratio ({derived['docs_to_code_ratio']}%)")
    else:
        lines.append(f"Documentation ratio: {derived['docs_to_code_ratio']}%")
    
    lines.append(f"Development intensity: {derived['development_intensity']}")
    lines.append("")
    
    # Recent commits
    lines.append("RECENT COMMITS")
    lines.append("─" * 52)
    for commit in git_stats['recent_commits'][:5]:
        lines.append(f"  {commit}")
    lines.append("")
    
    return '\n'.join(lines)


def format_markdown_output(
    git_stats: Dict[str, Any],
    code_stats: Dict[str, Any],
    doc_stats: Dict[str, Any],
    derived: Dict[str, Any]
) -> str:
    """Generate markdown output."""
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    
    lines = []
    lines.append("# R2D2 Project Statistics")
    lines.append("")
    lines.append(f"**Generated:** {timestamp}")
    lines.append("")
    
    # Summary Table
    lines.append("## Summary")
    lines.append("")
    lines.append("| Metric | Value |")
    lines.append("|--------|-------|")
    lines.append(f"| Total Commits | {format_number(git_stats['total_commits'])} |")
    lines.append(f"| Project Duration | {git_stats['project_duration_days']} days |")
    lines.append(f"| Development Velocity | {git_stats['commits_per_day']} commits/day |")
    lines.append(f"| Active Code Lines | {format_number(code_stats['total_lines'])} |")
    lines.append(f"| Code Files | {format_number(code_stats['total_files'])} |")
    lines.append(f"| Documentation Lines | {format_number(doc_stats['total_lines'])} |")
    lines.append(f"| Documentation Files | {format_number(doc_stats['total_files'])} |")
    lines.append(f"| Docs-to-Code Ratio | {derived['docs_to_code_ratio']}% |")
    lines.append(f"| Total Insertions | {format_number(git_stats['total_insertions'])} |")
    lines.append(f"| Total Deletions | {format_number(git_stats['total_deletions'])} |")
    lines.append(f"| Net Growth | {format_number(git_stats['net_growth'])} lines |")
    lines.append("")
    
    # Git Details
    lines.append("## Git Repository")
    lines.append("")
    lines.append(f"- **First Commit:** {git_stats['first_commit_date']}")
    lines.append(f"- **Last Commit:** {git_stats['last_commit_date']}")
    lines.append(f"- **Iteration Rate:** {derived['iteration_rate']}:1 (insertions/deletions)")
    lines.append(f"- **Development Intensity:** {derived['development_intensity']}")
    lines.append("")
    
    # Contributors
    lines.append("### Contributors")
    lines.append("")
    lines.append("| Contributor | Commits |")
    lines.append("|-------------|---------|")
    for name, count in git_stats['contributors']:
        lines.append(f"| {name} | {format_number(count)} |")
    lines.append("")
    
    # Code by Extension
    lines.append("## Code Breakdown")
    lines.append("")
    lines.append("| Extension | Files | Lines |")
    lines.append("|-----------|-------|-------|")
    for ext, data in sorted(code_stats['by_extension'].items(),
                           key=lambda x: x[1]['lines'], reverse=True):
        if data['files'] > 0:
            lines.append(f"| {ext} | {format_number(data['files'])} | {format_number(data['lines'])} |")
    lines.append("")
    
    # Recent Activity
    lines.append("## Recent Commits")
    lines.append("")
    for commit in git_stats['recent_commits'][:5]:
        lines.append(f"- `{commit}`")
    lines.append("")
    
    return '\n'.join(lines)


def format_json_output(
    git_stats: Dict[str, Any],
    code_stats: Dict[str, Any],
    doc_stats: Dict[str, Any],
    derived: Dict[str, Any]
) -> str:
    """Generate JSON output."""
    data = {
        'generated': datetime.now().isoformat(),
        'git': git_stats,
        'code': code_stats,
        'documentation': doc_stats,
        'derived': derived,
    }
    return json.dumps(data, indent=2, default=str)


# =============================================================================
# MAIN
# =============================================================================

def find_project_root() -> str:
    """Find the project root directory."""
    # Try current directory first
    cwd = os.getcwd()
    
    # Check if we're in the project
    if os.path.exists(os.path.join(cwd, '.git')):
        return cwd
    
    # Check if we're in a subdirectory
    current = Path(cwd)
    while current != current.parent:
        if (current / '.git').exists():
            return str(current)
        current = current.parent
    
    # Default to expected location
    default = os.path.expanduser('~/dev/r2d2')
    if os.path.exists(default):
        return default
    
    print("Error: Could not find project root. Run from project directory.", 
          file=sys.stderr)
    sys.exit(1)


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Generate R2D2 project statistics',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python tools/project_stats.py              # Terminal output
  python tools/project_stats.py --markdown   # Generate markdown
  python tools/project_stats.py --json       # Generate JSON
  python tools/project_stats.py --all        # All formats
        """
    )
    
    parser.add_argument(
        '--markdown', '-m',
        action='store_true',
        help='Output as markdown'
    )
    parser.add_argument(
        '--json', '-j',
        action='store_true',
        help='Output as JSON'
    )
    parser.add_argument(
        '--all', '-a',
        action='store_true',
        help='Output all formats (saves to files)'
    )
    parser.add_argument(
        '--quiet', '-q',
        action='store_true',
        help='Suppress progress messages'
    )
    
    args = parser.parse_args()
    
    # Find project root
    project_root = find_project_root()
    
    if not args.quiet:
        print(f"Analyzing project: {project_root}", file=sys.stderr)
    
    # Gather statistics
    if not args.quiet:
        print("  Gathering git statistics...", file=sys.stderr)
    git_stats = get_git_stats(project_root)
    
    if not args.quiet:
        print("  Counting code files...", file=sys.stderr)
    code_stats = count_code_files(project_root)
    
    if not args.quiet:
        print("  Counting documentation...", file=sys.stderr)
    doc_stats = count_documentation_files(project_root)
    
    if not args.quiet:
        print("  Calculating derived metrics...", file=sys.stderr)
    derived = calculate_derived_metrics(git_stats, code_stats, doc_stats)
    
    if not args.quiet:
        print("", file=sys.stderr)
    
    # Generate output
    if args.all:
        # Save all formats to files
        output_dir = os.path.join(project_root, '_TEMP')
        os.makedirs(output_dir, exist_ok=True)
        
        # Terminal output to console
        print(format_terminal_output(git_stats, code_stats, doc_stats, derived))
        
        # Markdown to file
        md_path = os.path.join(output_dir, 'project_stats.md')
        with open(md_path, 'w') as f:
            f.write(format_markdown_output(git_stats, code_stats, doc_stats, derived))
        print(f"Markdown saved to: {md_path}", file=sys.stderr)
        
        # JSON to file
        json_path = os.path.join(output_dir, 'project_stats.json')
        with open(json_path, 'w') as f:
            f.write(format_json_output(git_stats, code_stats, doc_stats, derived))
        print(f"JSON saved to: {json_path}", file=sys.stderr)
        
    elif args.markdown:
        print(format_markdown_output(git_stats, code_stats, doc_stats, derived))
        
    elif args.json:
        print(format_json_output(git_stats, code_stats, doc_stats, derived))
        
    else:
        # Default: terminal output
        print(format_terminal_output(git_stats, code_stats, doc_stats, derived))


if __name__ == '__main__':
    main()

