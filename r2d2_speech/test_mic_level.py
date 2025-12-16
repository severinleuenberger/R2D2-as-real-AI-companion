"""
Simple microphone level test.
Shows real-time audio levels to verify microphone is working.
"""

import numpy as np
import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from utils.audio_stream import AudioCapture, get_device_from_config
from config.config_manager import get_config


def main():
    """Test microphone levels."""
    print("=" * 70)
    print("MICROPHONE LEVEL TEST")
    print("=" * 70)
    print()
    print("⚠ IMPORTANT - Check your HyperX QuadCast S:")
    print()
    print("  1. Tap the TOP of the microphone:")
    print("     - RED light = MUTED (tap again to unmute)")
    print("     - OTHER color = UNMUTED (good!)")
    print()
    print("  2. Turn the GAIN DIAL on the bottom:")
    print("     - Recommend setting to 50-75% for normal use")
    print()
    print("=" * 70)
    input("Press ENTER when ready to start...")
    print()
    
    # Load config and initialize
    config = get_config()
    device_index, device_info = get_device_from_config(config)
    
    print(f"✓ Microphone: {device_info['name']}")
    print(f"✓ Sample rate: {config['mic_native_sample_rate']} Hz")
    print()
    
    capture = AudioCapture(
        device_index=device_index,
        native_sample_rate=config["mic_native_sample_rate"]
    )
    
    print("=" * 70)
    print("🎤 SPEAK NOW! (Will run for 10 seconds)")
    print()
    print("Try saying: \"Testing one two three, hello hello!\"")
    print()
    print("Watch the levels below:")
    print("=" * 70)
    print()
    
    capture.start()
    
    start_time = time.time()
    max_level = 0
    chunk_count = 0
    
    try:
        while time.time() - start_time < 10:
            audio_data = capture.read_chunk()
            if audio_data is None:
                time.sleep(0.01)
                continue
            
            # Calculate level
            avg_level = np.abs(audio_data).mean()
            peak_level = np.abs(audio_data).max()
            max_level = max(max_level, peak_level)
            
            # Show bar graph
            avg_bars = int((avg_level / 32767) * 50)
            peak_bars = int((peak_level / 32767) * 50)
            
            # Color coding
            if peak_level < 500:
                status = "❌ TOO QUIET"
            elif peak_level < 3000:
                status = "⚠️  QUIET    "
            elif peak_level < 20000:
                status = "✅ GOOD     "
            else:
                status = "⚠️  LOUD     "
            
            print(f"{status} | Avg: {avg_level:5.0f} {'█' * avg_bars}{' ' * (50-avg_bars)} | Peak: {peak_level:5.0f}", end='\r')
            
            chunk_count += 1
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n\nInterrupted")
    
    capture.stop()
    
    print("\n")
    print("=" * 70)
    print("RESULTS")
    print("=" * 70)
    print(f"Maximum level detected: {max_level:,.0f} (out of 32,767 max)")
    print()
    
    if max_level < 500:
        print("❌ MICROPHONE NOT WORKING or MUTED")
        print()
        print("Troubleshooting:")
        print("  1. Check the light on top of the microphone:")
        print("     - If RED: TAP the top to unmute")
        print("     - If off: Check USB connection")
        print("  2. Turn the gain dial (bottom) to 50-75%")
        print("  3. Speak DIRECTLY into the microphone (5-10 cm away)")
    elif max_level < 3000:
        print("⚠️  MICROPHONE TOO QUIET")
        print()
        print("Recommendation:")
        print("  - Turn the GAIN DIAL (bottom) higher (towards 75%)")
        print("  - Speak closer to the microphone")
    elif max_level < 20000:
        print("✅ MICROPHONE WORKING PERFECTLY!")
        print()
        print("Your levels are good for the Realtime API.")
    else:
        print("⚠️  MICROPHONE VERY LOUD")
        print()
        print("Recommendation:")
        print("  - Turn the GAIN DIAL (bottom) lower (towards 25-50%)")
        print("  - This prevents clipping and distortion")
    
    print("=" * 70)


if __name__ == "__main__":
    main()

