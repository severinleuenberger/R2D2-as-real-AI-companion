#!/usr/bin/env python3
"""
R2D2 Coding Narrator - Real-Time Learning Assistant

This script watches ~/dev/r2d2/data/coding_live.md for changes.
When the AI agent updates the file with explanations, R2D2 speaks them aloud!

Usage:
    python3 coding_narrator.py              # Run in foreground
    systemctl start r2d2-narrator.service   # Run as service

Modes (set in ~/dev/r2d2/data/narrator_mode.txt):
    narrator  - Speak every update (default)
    quiet     - Silent, but update speech context
    milestone - Only speak on task completions

Author: R2D2 Project
Date: December 2025
"""

import hashlib
import json
import os
import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

# ============================================================
# Configuration
# ============================================================

HOME = Path.home()
DATA_DIR = HOME / 'dev' / 'r2d2' / 'data'
WATCH_FILE = DATA_DIR / 'coding_live.md'
MODE_FILE = DATA_DIR / 'narrator_mode.txt'
CONTEXT_FILE = DATA_DIR / 'coding_context.json'
LOG_FILE = DATA_DIR / 'narrator.log'

# TTS Configuration
USE_OPENAI_TTS = False  # Set True for high-quality voice (requires API key)
ESPEAK_VOICE = 'en'     # espeak voice
ESPEAK_SPEED = 160      # Words per minute (default 175)

# Polling interval
POLL_INTERVAL = 0.5  # seconds

# ============================================================
# Logging
# ============================================================

def log(message: str, level: str = "INFO"):
    """Log message to file and stdout."""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    log_line = f"[{timestamp}] [{level}] {message}"
    print(log_line)
    
    try:
        with open(LOG_FILE, 'a') as f:
            f.write(log_line + '\n')
    except Exception:
        pass  # Don't fail if can't write log

# ============================================================
# File Watching
# ============================================================

def get_file_hash(filepath: Path) -> Optional[str]:
    """Get MD5 hash of file contents."""
    if not filepath.exists():
        return None
    try:
        return hashlib.md5(filepath.read_bytes()).hexdigest()
    except Exception as e:
        log(f"Error reading {filepath}: {e}", "ERROR")
        return None

def get_narrator_mode() -> str:
    """Get current narrator mode from mode file."""
    if MODE_FILE.exists():
        try:
            mode = MODE_FILE.read_text().strip().lower()
            if mode in ('narrator', 'quiet', 'milestone'):
                return mode
        except Exception:
            pass
    return 'narrator'  # Default mode

# ============================================================
# Content Parsing
# ============================================================

def parse_coding_live(content: str) -> dict:
    """Parse the coding_live.md markdown content."""
    result = {
        'action': '',
        'explanation': '',
        'bi_analogy': '',
        'category': '',
        'is_milestone': False
    }
    
    # Extract sections using regex
    sections = {
        'action': r'## Current Action\s*\n(.*?)(?=\n##|\Z)',
        'explanation': r'## What This Means\s*\n(.*?)(?=\n##|\Z)',
        'bi_analogy': r'## BI Analogy\s*\n(.*?)(?=\n##|\Z)',
        'category': r'## Category\s*\n(.*?)(?=\n##|\Z)',
    }
    
    for key, pattern in sections.items():
        match = re.search(pattern, content, re.DOTALL)
        if match:
            result[key] = match.group(1).strip()
    
    # Check for milestone markers
    result['is_milestone'] = any(marker in content.lower() for marker in [
        'completed', 'finished', 'done', 'milestone', 'task complete'
    ])
    
    return result

def create_narration(parsed: dict) -> str:
    """Create speakable narration from parsed content."""
    parts = []
    
    # Action (brief)
    if parsed['action']:
        parts.append(parsed['action'])
    
    # Main explanation
    if parsed['explanation']:
        parts.append(parsed['explanation'])
    
    # BI analogy (if present and not too long)
    if parsed['bi_analogy'] and len(parsed['bi_analogy']) < 200:
        parts.append(f"In BI terms: {parsed['bi_analogy']}")
    
    narration = ' '.join(parts)
    
    # Clean up for speech
    narration = narration.replace('\n', ' ')
    narration = re.sub(r'\s+', ' ', narration)  # Collapse whitespace
    narration = narration.strip()
    
    # Limit length for speech (avoid very long narrations)
    if len(narration) > 500:
        narration = narration[:500] + '...'
    
    return narration

# ============================================================
# Text-to-Speech
# ============================================================

def speak_espeak(text: str):
    """Speak text using espeak (fast, robotic - fits R2D2!)."""
    try:
        subprocess.run(
            ['espeak', '-v', ESPEAK_VOICE, '-s', str(ESPEAK_SPEED), text],
            check=True,
            capture_output=True
        )
        log(f"Spoke: {text[:50]}...")
    except FileNotFoundError:
        log("espeak not found! Install with: sudo apt install espeak", "ERROR")
    except subprocess.CalledProcessError as e:
        log(f"espeak error: {e}", "ERROR")

def speak_openai(text: str):
    """Speak text using OpenAI TTS (high quality)."""
    try:
        # This would require the openai library and API key
        # For now, fall back to espeak
        log("OpenAI TTS not implemented yet, using espeak", "WARN")
        speak_espeak(text)
    except Exception as e:
        log(f"OpenAI TTS error: {e}", "ERROR")
        speak_espeak(text)

def speak(text: str):
    """Speak text using configured TTS engine."""
    if not text:
        return
    
    if USE_OPENAI_TTS:
        speak_openai(text)
    else:
        speak_espeak(text)

# ============================================================
# Context Management
# ============================================================

def update_speech_context(parsed: dict):
    """Update the context file for R2D2's speech node."""
    context = {
        'current_task': parsed['action'],
        'explanation': parsed['explanation'],
        'bi_analogy': parsed['bi_analogy'],
        'category': parsed['category'],
        'updated_at': datetime.now().isoformat()
    }
    
    try:
        with open(CONTEXT_FILE, 'w') as f:
            json.dump(context, f, indent=2)
        log("Updated speech context")
    except Exception as e:
        log(f"Error updating context: {e}", "ERROR")

# ============================================================
# Main Loop
# ============================================================

def main():
    """Main narrator loop."""
    log("=" * 50)
    log("R2D2 Coding Narrator Starting")
    log(f"Watching: {WATCH_FILE}")
    log(f"Mode file: {MODE_FILE}")
    log("=" * 50)
    
    # Ensure data directory exists
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    
    # Initialize hash tracking
    last_hash = get_file_hash(WATCH_FILE)
    
    # Announce startup
    speak("R2D2 narrator online. Ready to explain your code!")
    
    try:
        while True:
            # Check for file changes
            current_hash = get_file_hash(WATCH_FILE)
            
            if current_hash and current_hash != last_hash:
                log(f"File changed! Processing...")
                
                # Read and parse content
                try:
                    content = WATCH_FILE.read_text()
                    parsed = parse_coding_live(content)
                    
                    # Always update speech context (for follow-up questions)
                    update_speech_context(parsed)
                    
                    # Check narrator mode
                    mode = get_narrator_mode()
                    log(f"Narrator mode: {mode}")
                    
                    # Decide whether to speak
                    should_speak = False
                    if mode == 'narrator':
                        should_speak = True
                    elif mode == 'milestone' and parsed['is_milestone']:
                        should_speak = True
                    # quiet mode: never speak, but context is updated
                    
                    if should_speak:
                        narration = create_narration(parsed)
                        if narration:
                            speak(narration)
                    else:
                        log(f"Mode is '{mode}', not speaking")
                    
                except Exception as e:
                    log(f"Error processing file: {e}", "ERROR")
                
                last_hash = current_hash
            
            # Sleep before next check
            time.sleep(POLL_INTERVAL)
            
    except KeyboardInterrupt:
        log("Narrator stopped by user")
        speak("Narrator going offline. Goodbye!")
    except Exception as e:
        log(f"Fatal error: {e}", "ERROR")
        sys.exit(1)

if __name__ == '__main__':
    main()



