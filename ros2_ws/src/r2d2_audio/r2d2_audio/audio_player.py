#!/usr/bin/env python3
"""
Audio file player for R2D2
Plays MP3 audio files using ffplay or other available audio player
"""

import subprocess
import sys
import os
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


def play_audio(file_path: str, volume: float = 0.5, alsa_device: Optional[str] = None, 
               master_volume: float = 1.0) -> bool:
    """
    Play an audio file.
    
    Args:
        file_path: Path to the audio file to play
        volume: Local volume level 0.0-1.0 (0.5 = 50%)
        alsa_device: ALSA device specification (e.g., "hw:1,0"). If None, uses default or environment variable.
        master_volume: Master volume multiplier 0.0-1.0 from physical volume knob (default: 1.0)
    
    The effective volume is: master_volume * volume
    For example: 0.5 (master) * 0.5 (local) = 0.25 (effective)
    
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
    
    # Calculate effective volume
    effective_volume = max(0.0, min(1.0, master_volume * volume))
    
    # Get ALSA device from parameter or environment variable
    device = alsa_device or os.environ.get('AUDIODEV') or os.environ.get('ALSA_CARD', 'hw:1,0')
    
    try:
        # Prepare command based on player
        if player == 'ffplay':
            # ffplay: -nodisp (no display), -autoexit (exit after playback)
            # Use -af "volume=X" for volume control (X is linear gain, 0.0-1.0)
            # Note: ffplay uses system default audio device (ALSA device selection not supported)
            cmd = [
                'ffplay',
                '-nodisp',
                '-autoexit',
                '-loglevel', 'error',  # Reduce logging
                '-af', f'volume={effective_volume}',
                str(audio_path)
            ]
        elif player == 'mpv':
            # mpv: --no-video (no video), --really-quiet (minimal output), --volume=X (0-100)
            # For ALSA, use --audio-device=alsa/DEVICE
            cmd = [
                'mpv',
                '--no-video',
                '--really-quiet',
                f'--volume={int(effective_volume * 100)}',
            ]
            if device:
                cmd.append(f'--audio-device=alsa/{device}')
            cmd.append(str(audio_path))
        elif player == 'aplay':
            # aplay: simple PCM player, use -D for device (no software volume control)
            cmd = ['aplay', '-D', device, str(audio_path)] if device else ['aplay', str(audio_path)]
        elif player == 'paplay':
            # PulseAudio player (doesn't support ALSA device directly)
            cmd = ['paplay', str(audio_path)]
        else:
            # sox play command
            cmd = ['play', str(audio_path)]
        
        # Prepare environment with ALSA device if needed
        env = os.environ.copy()
        if device and player in ['ffplay', 'mpv']:
            # Set ALSA environment variables as fallback
            env['AUDIODEV'] = device
        
        # Run player without blocking (detach from parent process)
        # Note: Errors from detached process are not captured (acceptable for background playback)
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env=env,
            start_new_session=True
        )
        
        return True
        
    except Exception as e:
        print(f"Error playing audio: {e}", file=sys.stderr)
        return False


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 audio_player.py <file_path> [volume] [alsa_device] [master_volume]")
        print("  volume: 0.0-1.0 (default 0.5) - local/content volume")
        print("  alsa_device: ALSA device (e.g., hw:1,0, default from AUDIODEV env or hw:1,0)")
        print("  master_volume: 0.0-1.0 (default 1.0) - master volume multiplier")
        print("  Effective volume = volume * master_volume")
        sys.exit(1)
    
    file_path = sys.argv[1]
    volume = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    alsa_device = sys.argv[3] if len(sys.argv) > 3 else None
    master_volume = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
    
    success = play_audio(file_path, volume, alsa_device, master_volume)
    sys.exit(0 if success else 1)
