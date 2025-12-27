#!/usr/bin/env python3
"""
R2D2 Audio Volume Baseline Testing Script

Purpose: Test all audio sources at various volume levels to establish
baseline measurements and identify optimal min/max volume settings.

Usage:
    python3 test_volume_baseline.py --mode interactive
    python3 test_volume_baseline.py --mode interactive --output baseline_results.json
    python3 test_volume_baseline.py --mode automatic --output baseline_results.json
    python3 test_volume_baseline.py --verify baseline_results.json

Test Modes:
1. Interactive: User manually evaluates each volume level (recommended for first baseline)
2. Automatic: Plays test tones/files at various volumes, logs to file
3. Verify: Compare current system against saved baseline

Audio Sources Tested:
- Recognition beep (Voicy_R2-D2 - 2.mp3)
- Loss beep (Voicy_R2-D2 - 5.mp3)
- Gesture ack (Voicy_R2-D2 - 12.mp3)
- Session start (Voicy_R2-D2 - 16.mp3)
- Session stop (Voicy_R2-D2 - 20.mp3)

Author: R2D2 Audio System
Date: December 2025
"""

import argparse
import json
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Any


# Audio file paths
AUDIO_ASSETS_DIR = Path(__file__).parent.parent / 'r2d2_audio' / 'assets' / 'audio'

# Audio sources to test
AUDIO_SOURCES = {
    'recognition_beep': {
        'file': 'Voicy_R2-D2 - 2.mp3',
        'description': 'Recognition beep (face detected)',
        'category': 'notification'
    },
    'loss_beep': {
        'file': 'Voicy_R2-D2 - 5.mp3',
        'description': 'Loss beep (person lost)',
        'category': 'notification'
    },
    'gesture_ack': {
        'file': 'Voicy_R2-D2 - 12.mp3',
        'description': 'Gesture acknowledgment',
        'category': 'feedback'
    },
    'session_start': {
        'file': 'Voicy_R2-D2 - 16.mp3',
        'description': 'Speech session started',
        'category': 'feedback'
    },
    'session_stop': {
        'file': 'Voicy_R2-D2 - 20.mp3',
        'description': 'Speech session stopped',
        'category': 'feedback'
    }
}

# Volume levels to test (from very quiet to maximum)
VOLUME_LEVELS = [0.0, 0.01, 0.02, 0.05, 0.1, 0.2, 0.3, 0.5, 0.7, 1.0]

# Rating descriptions
RATING_DESCRIPTIONS = {
    1: "Too quiet / inaudible",
    2: "Quiet but audible",
    3: "Good / acceptable",
    4: "Loud",
    5: "Too loud / distortion"
}


def play_audio(file_path: Path, volume: float) -> bool:
    """
    Play an audio file at the specified volume using ffplay.
    
    Args:
        file_path: Path to the audio file
        volume: Volume level (0.0 - 1.0)
    
    Returns:
        True if playback succeeded, False otherwise
    """
    if not file_path.exists():
        print(f"Error: Audio file not found: {file_path}")
        return False
    
    try:
        cmd = [
            'ffplay',
            '-nodisp',
            '-autoexit',
            '-loglevel', 'error',
            '-af', f'volume={volume}',
            str(file_path)
        ]
        
        result = subprocess.run(cmd, capture_output=True, timeout=10)
        return result.returncode == 0
    except subprocess.TimeoutExpired:
        print("Warning: Audio playback timed out")
        return False
    except FileNotFoundError:
        print("Error: ffplay not found. Please install ffmpeg.")
        return False
    except Exception as e:
        print(f"Error playing audio: {e}")
        return False


def get_user_rating() -> int:
    """
    Get volume rating from user (1-5).
    
    Returns:
        Rating value (1-5)
    """
    print("\nRate this volume (1-5):")
    for rating, desc in RATING_DESCRIPTIONS.items():
        print(f"  {rating} = {desc}")
    
    while True:
        try:
            rating = input("\nYour rating: ").strip()
            if rating.lower() == 'q':
                raise KeyboardInterrupt
            rating_int = int(rating)
            if 1 <= rating_int <= 5:
                return rating_int
            print("Please enter a number between 1 and 5")
        except ValueError:
            print("Invalid input. Please enter a number (1-5) or 'q' to quit")


def get_user_notes() -> str:
    """Get optional notes from user."""
    notes = input("Notes (optional, press Enter to skip): ").strip()
    return notes


def run_interactive_test(output_file: Optional[Path] = None) -> Dict[str, Any]:
    """
    Run interactive baseline test where user rates each volume level.
    
    Args:
        output_file: Optional path to save results
    
    Returns:
        Test results dictionary
    """
    print("\n" + "=" * 60)
    print("R2D2 Audio Volume Baseline Test - Interactive Mode")
    print("=" * 60)
    print("\nThis test will play various audio files at different volume levels.")
    print("You will rate each one on a scale of 1-5.")
    print("Press 'q' at any rating prompt to quit and save progress.")
    print("\nMake sure your speaker is connected and working!")
    input("\nPress Enter to begin...")
    
    results = {
        'test_date': datetime.now().isoformat(),
        'test_mode': 'interactive',
        'hardware': {
            'device': 'hw:1,0',
            'amplifier': 'PAM8403',
            'speaker': '8Ω'
        },
        'baseline_results': {},
        'recommendations': {}
    }
    
    try:
        for source_id, source_info in AUDIO_SOURCES.items():
            print(f"\n{'=' * 60}")
            print(f"Testing: {source_info['description']}")
            print(f"File: {source_info['file']}")
            print(f"Category: {source_info['category']}")
            print('=' * 60)
            
            audio_file = AUDIO_ASSETS_DIR / source_info['file']
            if not audio_file.exists():
                print(f"Warning: Audio file not found, skipping: {audio_file}")
                continue
            
            results['baseline_results'][source_id] = {}
            
            for volume in VOLUME_LEVELS:
                print(f"\n--- Volume: {volume} ({volume * 100:.0f}%) ---")
                print("[Playing audio...]")
                
                success = play_audio(audio_file, volume)
                if not success:
                    print("Playback failed, skipping this volume level")
                    continue
                
                # Wait a moment after playback
                time.sleep(0.3)
                
                rating = get_user_rating()
                notes = get_user_notes()
                
                results['baseline_results'][source_id][str(volume)] = {
                    'rating': rating,
                    'notes': notes,
                    'rating_description': RATING_DESCRIPTIONS[rating]
                }
                
                # If rated as distortion (5), no need to test higher volumes
                if rating == 5:
                    print(f"\nDistortion detected at {volume}, skipping higher volumes for this source.")
                    break
    
    except KeyboardInterrupt:
        print("\n\nTest interrupted. Saving progress...")
    
    # Calculate recommendations
    results['recommendations'] = calculate_recommendations(results['baseline_results'])
    
    # Save results
    if output_file:
        save_results(results, output_file)
    
    # Print summary
    print_summary(results)
    
    return results


def run_automatic_test(output_file: Optional[Path] = None) -> Dict[str, Any]:
    """
    Run automatic baseline test (plays all sounds, logs without user input).
    
    Args:
        output_file: Optional path to save results
    
    Returns:
        Test results dictionary
    """
    print("\n" + "=" * 60)
    print("R2D2 Audio Volume Baseline Test - Automatic Mode")
    print("=" * 60)
    print("\nThis test will play all audio files at various volume levels.")
    print("No user input required - just listen and observe.")
    print("\nMake sure your speaker is connected!")
    input("\nPress Enter to begin...")
    
    results = {
        'test_date': datetime.now().isoformat(),
        'test_mode': 'automatic',
        'hardware': {
            'device': 'hw:1,0',
            'amplifier': 'PAM8403',
            'speaker': '8Ω'
        },
        'baseline_results': {},
        'playback_log': []
    }
    
    for source_id, source_info in AUDIO_SOURCES.items():
        print(f"\n{'=' * 60}")
        print(f"Testing: {source_info['description']}")
        print('=' * 60)
        
        audio_file = AUDIO_ASSETS_DIR / source_info['file']
        if not audio_file.exists():
            print(f"Warning: Audio file not found, skipping: {audio_file}")
            continue
        
        results['baseline_results'][source_id] = {}
        
        for volume in VOLUME_LEVELS:
            print(f"  Volume {volume:.2f} ({volume * 100:.0f}%)... ", end='', flush=True)
            
            start_time = time.time()
            success = play_audio(audio_file, volume)
            elapsed = time.time() - start_time
            
            status = "OK" if success else "FAILED"
            print(status)
            
            results['playback_log'].append({
                'source': source_id,
                'volume': volume,
                'success': success,
                'duration': elapsed
            })
            
            results['baseline_results'][source_id][str(volume)] = {
                'played': success,
                'timestamp': datetime.now().isoformat()
            }
            
            time.sleep(0.5)  # Brief pause between sounds
    
    if output_file:
        save_results(results, output_file)
    
    print("\n" + "=" * 60)
    print("Automatic test complete!")
    print("Review the audio playback and manually note your observations.")
    print("=" * 60)
    
    return results


def run_verification(baseline_file: Path) -> Dict[str, Any]:
    """
    Verify current system against saved baseline.
    
    Args:
        baseline_file: Path to baseline results JSON file
    
    Returns:
        Verification results dictionary
    """
    print("\n" + "=" * 60)
    print("R2D2 Audio Volume Baseline Verification")
    print("=" * 60)
    
    # Load baseline
    try:
        with open(baseline_file, 'r') as f:
            baseline = json.load(f)
    except FileNotFoundError:
        print(f"Error: Baseline file not found: {baseline_file}")
        sys.exit(1)
    except json.JSONDecodeError:
        print(f"Error: Invalid JSON in baseline file: {baseline_file}")
        sys.exit(1)
    
    print(f"\nLoaded baseline from: {baseline_file}")
    print(f"Baseline date: {baseline.get('test_date', 'unknown')}")
    
    if 'recommendations' in baseline:
        rec = baseline['recommendations']
        print(f"\nBaseline recommendations:")
        print(f"  Min volume: {rec.get('min_volume', 'N/A')}")
        print(f"  Max volume: {rec.get('max_volume', 'N/A')}")
        print(f"  Default: {rec.get('default_volume', 'N/A')}")
    
    print("\nRunning verification tests...")
    input("Press Enter to continue...")
    
    verification_results = {
        'verification_date': datetime.now().isoformat(),
        'baseline_file': str(baseline_file),
        'baseline_date': baseline.get('test_date'),
        'tests': [],
        'status': 'PASS'
    }
    
    # Test key volume levels from recommendations
    if 'recommendations' in baseline:
        rec = baseline['recommendations']
        test_volumes = [
            ('min_volume', rec.get('min_volume', 0.01)),
            ('default_volume', rec.get('default_volume', 0.02)),
            ('max_volume', rec.get('max_volume', 0.3))
        ]
        
        for level_name, volume in test_volumes:
            if volume is None:
                continue
                
            print(f"\n--- Testing {level_name}: {volume} ---")
            
            # Test with first audio source
            source_id = list(AUDIO_SOURCES.keys())[0]
            source_info = AUDIO_SOURCES[source_id]
            audio_file = AUDIO_ASSETS_DIR / source_info['file']
            
            if audio_file.exists():
                print(f"Playing {source_info['description']}...")
                success = play_audio(audio_file, volume)
                
                verification_results['tests'].append({
                    'level': level_name,
                    'volume': volume,
                    'success': success
                })
                
                if success:
                    print("✓ Playback successful")
                else:
                    print("✗ Playback failed")
                    verification_results['status'] = 'FAIL'
            
            time.sleep(0.5)
    
    # Summary
    print("\n" + "=" * 60)
    print(f"Verification Status: {verification_results['status']}")
    print("=" * 60)
    
    return verification_results


def calculate_recommendations(baseline_results: Dict[str, Dict]) -> Dict[str, Any]:
    """
    Calculate volume recommendations from baseline results.
    
    Args:
        baseline_results: Dictionary of test results per audio source
    
    Returns:
        Recommendations dictionary
    """
    recommendations = {
        'min_volume': None,
        'default_volume': None,
        'max_volume': None,
        'optimal_range': None
    }
    
    # Collect all ratings
    all_good_volumes = []  # Volumes rated 3 (good)
    all_min_volumes = []   # Lowest audible volumes (rated 2+)
    all_max_volumes = []   # Highest before distortion
    
    for source_id, volumes in baseline_results.items():
        for volume_str, data in volumes.items():
            if 'rating' not in data:
                continue
            
            volume = float(volume_str)
            rating = data['rating']
            
            if rating >= 2:  # Audible
                all_min_volumes.append(volume)
            if rating == 3:  # Good
                all_good_volumes.append(volume)
            if rating == 4:  # Loud but acceptable
                all_max_volumes.append(volume)
            if rating == 5:  # Distortion - mark previous as max
                # Find previous volume level
                idx = VOLUME_LEVELS.index(volume)
                if idx > 0:
                    all_max_volumes.append(VOLUME_LEVELS[idx - 1])
    
    # Calculate recommendations
    if all_min_volumes:
        recommendations['min_volume'] = min(all_min_volumes)
    
    if all_good_volumes:
        recommendations['default_volume'] = min(all_good_volumes)
    
    if all_max_volumes:
        recommendations['max_volume'] = max(all_max_volumes)
    
    if recommendations['min_volume'] and recommendations['max_volume']:
        recommendations['optimal_range'] = f"{recommendations['min_volume']} - {recommendations['max_volume']}"
    
    return recommendations


def save_results(results: Dict[str, Any], output_file: Path):
    """Save results to JSON file."""
    output_file.parent.mkdir(parents=True, exist_ok=True)
    
    with open(output_file, 'w') as f:
        json.dump(results, f, indent=2)
    
    print(f"\nResults saved to: {output_file}")


def print_summary(results: Dict[str, Any]):
    """Print test summary."""
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    
    print(f"\nTest Date: {results.get('test_date', 'unknown')}")
    print(f"Test Mode: {results.get('test_mode', 'unknown')}")
    
    if 'recommendations' in results:
        rec = results['recommendations']
        print("\nRecommendations:")
        print(f"  Minimum usable volume: {rec.get('min_volume', 'N/A')}")
        print(f"  Default volume: {rec.get('default_volume', 'N/A')}")
        print(f"  Maximum safe volume: {rec.get('max_volume', 'N/A')}")
        print(f"  Optimal range: {rec.get('optimal_range', 'N/A')}")
    
    if 'baseline_results' in results:
        print("\nPer-Source Summary:")
        for source_id, volumes in results['baseline_results'].items():
            if not volumes:
                continue
            
            source_info = AUDIO_SOURCES.get(source_id, {})
            print(f"\n  {source_info.get('description', source_id)}:")
            
            # Find optimal volume (rating 3)
            optimal = None
            for vol_str, data in volumes.items():
                if data.get('rating') == 3:
                    optimal = vol_str
                    break
            
            if optimal:
                print(f"    Optimal volume: {optimal}")
    
    print("\n" + "=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description='R2D2 Audio Volume Baseline Testing Script',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  Interactive baseline test:
    python3 test_volume_baseline.py --mode interactive
    
  Save results to file:
    python3 test_volume_baseline.py --mode interactive --output test_results/baseline.json
    
  Automatic test (no user input):
    python3 test_volume_baseline.py --mode automatic --output test_results/auto.json
    
  Verify against baseline:
    python3 test_volume_baseline.py --verify test_results/baseline.json
"""
    )
    
    parser.add_argument(
        '--mode',
        choices=['interactive', 'automatic'],
        help='Test mode: interactive (user rates each volume) or automatic (no input)'
    )
    parser.add_argument(
        '--output', '-o',
        type=Path,
        help='Output file for results (JSON format)'
    )
    parser.add_argument(
        '--verify',
        type=Path,
        metavar='BASELINE_FILE',
        help='Verify current system against saved baseline file'
    )
    parser.add_argument(
        '--list-sources',
        action='store_true',
        help='List available audio sources and exit'
    )
    
    args = parser.parse_args()
    
    # List sources
    if args.list_sources:
        print("\nAvailable Audio Sources:")
        print("-" * 40)
        for source_id, info in AUDIO_SOURCES.items():
            audio_file = AUDIO_ASSETS_DIR / info['file']
            exists = "✓" if audio_file.exists() else "✗"
            print(f"  [{exists}] {source_id}")
            print(f"      File: {info['file']}")
            print(f"      Description: {info['description']}")
            print(f"      Category: {info['category']}")
        return
    
    # Verification mode
    if args.verify:
        run_verification(args.verify)
        return
    
    # Test mode required
    if not args.mode:
        parser.print_help()
        print("\nError: --mode or --verify is required")
        sys.exit(1)
    
    # Run test
    if args.mode == 'interactive':
        run_interactive_test(args.output)
    elif args.mode == 'automatic':
        run_automatic_test(args.output)


if __name__ == '__main__':
    main()

