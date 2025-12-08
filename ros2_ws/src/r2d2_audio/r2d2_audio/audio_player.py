#!/usr/bin/env python3
"""
Audio file player for R2D2
Plays MP3 audio files using ffplay or other available audio player
"""

import subprocess
import sys
from pathlib import Path
from typing import Optional


def find_audio_player() -> Optional[str]:
    """
    Find an available audio player on the system.
    
    Returns:
        Path to audio player executable or None if not found
    """
    players = ['ffplay', 'mpv', 'aplay', 'paplay', 'sox']
    
    for player in players:
        try:
            result = subprocess.run(['which', player], capture_output=True, timeout=1)
            if result.returncode == 0:
                return player
        except (subprocess.TimeoutExpired, FileNotFoundError):
            continue
    
    return None


def play_audio(file_path: str, volume: float = 0.5) -> bool:
    """
    Play an audio file.
    
    Args:
        file_path: Path to the audio file to play
        volume: Volume level 0.0-1.0 (0.5 = 50%)
    
    Returns:
        True if playback started successfully, False otherwise
    """
    audio_path = Path(file_path)
    
    if not audio_path.exists():
        print(f"Error: Audio file not found: {file_path}", file=sys.stderr)
        return False
    
    player = find_audio_player()
    
    if player is None:
        print("Error: No audio player found. Install ffplay, mpv, or aplay.", file=sys.stderr)
        return False
    
    try:
        # Prepare command based on player
        if player == 'ffplay':
            # ffplay: -nodisp (no display), -autoexit (exit after playback)
            # Note: ffplay doesn't have direct volume control via CLI, use -af "volume=X"
            volume_db = 20 * (volume - 1)  # Convert 0.0-1.0 to dB (-∞ to 0)
            cmd = [
                'ffplay',
                '-nodisp',
                '-autoexit',
                '-af', f'volume={volume}',
                str(audio_path)
            ]
        elif player == 'mpv':
            # mpv: --no-video (no video), --really-quiet (minimal output), --volume=X (0-100)
            cmd = [
                'mpv',
                '--no-video',
                '--really-quiet',
                f'--volume={int(volume * 100)}',
                str(audio_path)
            ]
        elif player == 'aplay':
            # aplay: simple PCM player, no volume control in CLI
            cmd = ['aplay', str(audio_path)]
        elif player == 'paplay':
            # PulseAudio player
            cmd = ['paplay', str(audio_path)]
        else:
            # sox play command
            cmd = ['play', str(audio_path)]
        
        # Run player without blocking (detach from parent process)
        subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True
        )
        
        return True
        
    except Exception as e:
        print(f"Error playing audio: {e}", file=sys.stderr)
        return False


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 audio_player.py <file_path> [volume]")
        print("  volume: 0.0-1.0 (default 0.5)")
        sys.exit(1)
    
    file_path = sys.argv[1]
    volume = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    
    success = play_audio(file_path, volume)
    sys.exit(0 if success else 1)
