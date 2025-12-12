#!/usr/bin/env python3
"""
Interactive Guided Face Training Data Capture

This script guides you through the capture process with confirmations
at each stage, ensuring you're ready before capture begins.
"""

import sys
import os
import subprocess

def print_section(title):
    print("\n" + "="*60)
    print(title)
    print("="*60)

def print_stage_info(stage_num, stage_name, description, duration, max_images):
    print(f"\n📸 STAGE {stage_num}/5: {stage_name.upper()}")
    print("-" * 60)
    print(f"Description: {description}")
    print(f"Duration: {duration} seconds")
    print(f"Maximum images: {max_images}")
    print("-" * 60)

def get_confirmation(prompt, default="yes"):
    """Get user confirmation with default option."""
    if default == "yes":
        prompt += " [Y/n]: "
    else:
        prompt += " [y/N]: "
    
    while True:
        try:
            response = input(prompt).strip().lower()
            if not response:
                response = default
            if response in ['y', 'yes']:
                return True
            elif response in ['n', 'no']:
                return False
            else:
                print("Please enter 'y' or 'n'")
        except (EOFError, KeyboardInterrupt):
            print("\n❌ Cancelled by user")
            return False

def main():
    print_section("R2D2 IMPROVED FACE TRAINING CAPTURE - INTERACTIVE GUIDE")
    
    print("""
This guide will walk you through capturing high-quality training images.
The improved system includes:
  ✓ Quality filtering (only saves sharp, well-lit images)
  ✓ Deduplication (avoids redundant images)
  ✓ Smart limits (40-50 images total, not 800+)

You'll be asked to confirm before each stage begins.
You can cancel at any time by pressing Ctrl+C.
""")
    
    # Initial confirmation
    if not get_confirmation("\n🔍 Are you ready to begin? (Make sure you're in front of the camera)", "yes"):
        print("\n❌ Capture cancelled. Run again when ready.")
        return
    
    # Stage information
    stages = [
        {
            "name": "Bright Direct Light",
            "description": "Stand 1 meter from camera, look straight ahead. Move slowly left/right and nod up/down.",
            "duration": 10,
            "max_images": 10
        },
        {
            "name": "Dim Indoor Light",
            "description": "Move to dimly lit area. Stand 1 meter from camera, look straight ahead. Move left/right and nod.",
            "duration": 10,
            "max_images": 10
        },
        {
            "name": "Side Profile 45°",
            "description": "Return to bright area. Stand at 45-degree angle. Move slowly while maintaining angle.",
            "duration": 10,
            "max_images": 10
        },
        {
            "name": "Varied Distance",
            "description": "Stand 1m for 3s, then 2m for 3s, then 3m. Look straight ahead at each distance.",
            "duration": 15,
            "max_images": 10
        },
        {
            "name": "Facial Expressions",
            "description": "Stand 1 meter, front-facing. Show neutral, then smile, then variations.",
            "duration": 5,
            "max_images": 10
        }
    ]
    
    # Confirm each stage
    confirmed_stages = []
    for i, stage in enumerate(stages, 1):
        print_stage_info(i, stage["name"], stage["description"], stage["duration"], stage["max_images"])
        
        if get_confirmation(f"\n✅ Ready for Stage {i}?", "yes"):
            confirmed_stages.append((i, stage))
            print(f"✓ Stage {i} confirmed")
        else:
            print(f"⏭️  Stage {i} skipped")
    
    if not confirmed_stages:
        print("\n❌ No stages confirmed. Capture cancelled.")
        return
    
    # Final summary
    print_section("CAPTURE SUMMARY")
    print(f"Total stages to capture: {len(confirmed_stages)}")
    print(f"Expected images: {len(confirmed_stages) * 10} (40-50 total)")
    print(f"Estimated time: ~{sum(s['duration'] for _, s in confirmed_stages)} seconds")
    
    if not get_confirmation("\n🚀 Start capture now?", "yes"):
        print("\n❌ Capture cancelled.")
        return
    
    # Run the capture script
    print("\n" + "="*60)
    print("STARTING CAPTURE...")
    print("="*60 + "\n")
    
    script_path = os.path.join(os.path.dirname(__file__), "1_capture_training_data.py")
    
    try:
        # Import and run the capture
        os.chdir(os.path.dirname(__file__))
        sys.path.insert(0, os.path.dirname(__file__))
        
        # Set environment
        env = os.environ.copy()
        if 'OPENBLAS_CORETYPE' not in env:
            env['OPENBLAS_CORETYPE'] = 'ARMV8'
        
        # Run the capture script
        result = subprocess.run(
            [sys.executable, script_path],
            env=env,
            cwd=os.path.dirname(__file__)
        )
        
        if result.returncode == 0:
            print_section("CAPTURE COMPLETE!")
            print("\n✅ Training data captured successfully!")
            print("\nNext steps:")
            print("  1. Review images: ls -lh ~/dev/r2d2/data/face_recognition/severin/")
            print("  2. Train model: python3 2_train_recognizer.py")
            print("  3. Test model: python3 3_test_recognizer_demo.py")
        else:
            print("\n❌ Capture failed. Check error messages above.")
            
    except KeyboardInterrupt:
        print("\n\n❌ Capture interrupted by user.")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()

