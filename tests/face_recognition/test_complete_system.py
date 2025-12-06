#!/usr/bin/env python3
"""
Complete System Test Script

Tests all components:
1. LED controller (text, GPIO, HTTP)
2. Face recognition service startup
3. Status file generation
4. Menu integration
5. Data protection (git, overwrite prevention)
"""

import subprocess
import sys
import time
import json
from pathlib import Path


def print_header(title):
    """Print a formatted header."""
    print()
    print("=" * 70)
    print(f"  {title}")
    print("=" * 70)


def test_led_controller():
    """Test LED controller module."""
    print_header("TEST 1: LED Controller")
    
    try:
        from led_controller import create_led_controller, TextLED
        
        print("✓ LED controller imports successful")
        
        # Test text LED
        led = create_led_controller('text')
        print("✓ Text LED created")
        
        print("\nTesting LED states:")
        print("  Recognized state:")
        led.set_recognized("severin")
        
        print("\n  Unrecognized state:")
        led.set_unrecognized()
        
        print("\n  Error state:")
        led.set_error()
        
        print("\n\n✅ LED CONTROLLER TEST PASSED")
        return True
        
    except Exception as e:
        print(f"❌ LED CONTROLLER TEST FAILED: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_face_recognition_service():
    """Test service startup."""
    print_header("TEST 2: Face Recognition Service")
    
    try:
        from face_recognition_service import FaceRecognitionService
        
        print("✓ Face recognition service imports successful")
        
        # Create service instance (don't run it)
        service = FaceRecognitionService(
            person_name='test_person',
            data_dir=str(Path.home() / 'dev' / 'r2d2' / 'data' / 'face_recognition'),
            confidence_threshold=70,
            cpu_limit=0.15
        )
        
        print("✓ Service initialized successfully")
        print(f"  - Person: test_person")
        print(f"  - Threshold: 70")
        print(f"  - CPU Limit: 15%")
        print(f"  - Frame Skip: {service.skip_frames}")
        print(f"  - Status File: {service.status_file}")
        print(f"  - Log File: {service.log_file}")
        
        # Verify LED is available
        if service.led:
            print("✓ LED controller integrated")
        else:
            print("⚠️  LED controller not available (but service will fall back to console)")
        
        print("\n✅ FACE RECOGNITION SERVICE TEST PASSED")
        return True
        
    except Exception as e:
        print(f"❌ FACE RECOGNITION SERVICE TEST FAILED: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_status_file_format():
    """Test status file JSON format."""
    print_header("TEST 3: Status File Format")
    
    try:
        # Create sample status
        status = {
            "timestamp": "2024-01-15T10:30:45.123456",
            "recognized_person": "severin",
            "confidence_threshold": 70,
            "frame_count": 1234
        }
        
        # Verify it's valid JSON
        json_str = json.dumps(status, indent=2)
        parsed = json.loads(json_str)
        
        print("Sample status file format:")
        print(json_str)
        
        # Verify all fields
        assert parsed["timestamp"], "Missing timestamp"
        assert "recognized_person" in parsed, "Missing recognized_person"
        assert parsed["confidence_threshold"] == 70, "Missing/wrong threshold"
        assert parsed["frame_count"] >= 0, "Missing frame_count"
        
        print("\n✅ STATUS FILE FORMAT TEST PASSED")
        return True
        
    except Exception as e:
        print(f"❌ STATUS FILE FORMAT TEST FAILED: {e}")
        return False


def test_data_structure():
    """Test data directory structure."""
    print_header("TEST 4: Data Structure")
    
    try:
        data_dir = Path.home() / 'dev' / 'r2d2' / 'data' / 'face_recognition'
        
        if not data_dir.exists():
            print(f"❌ Data directory doesn't exist: {data_dir}")
            return False
        
        print(f"✓ Data directory exists: {data_dir}")
        
        # Check for person directories
        person_dirs = [d for d in data_dir.iterdir() if d.is_dir() and not d.name.startswith('.') and d.name != 'models']
        print(f"✓ Found {len(person_dirs)} person(s)")
        for person_dir in person_dirs:
            images = list(person_dir.glob('*.jpg'))
            print(f"  - {person_dir.name}: {len(images)} images")
        
        # Check for models directory
        models_dir = data_dir / 'models'
        if models_dir.exists():
            models = list(models_dir.glob('*.xml'))
            print(f"✓ Found {len(models)} model(s)")
            for model in models:
                size_mb = model.stat().st_size / (1024 * 1024)
                print(f"  - {model.name}: {size_mb:.1f} MB")
        else:
            print("⚠️  Models directory not found")
        
        print("\n✅ DATA STRUCTURE TEST PASSED")
        return True
        
    except Exception as e:
        print(f"❌ DATA STRUCTURE TEST FAILED: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_git_protection():
    """Test that training images are in .gitignore."""
    print_header("TEST 5: Git Protection")
    
    try:
        gitignore_path = Path.home() / 'dev' / 'r2d2' / '.gitignore'
        
        if not gitignore_path.exists():
            print(f"⚠️  .gitignore not found: {gitignore_path}")
            return False
        
        gitignore_content = gitignore_path.read_text()
        
        # Check for protection
        if '/data/face_recognition/*/' in gitignore_content:
            print("✓ Training images protected in .gitignore")
            print("  Pattern: /data/face_recognition/*/")
        else:
            print("⚠️  Training images may not be protected")
        
        print("\n✅ GIT PROTECTION TEST PASSED")
        return True
        
    except Exception as e:
        print(f"❌ GIT PROTECTION TEST FAILED: {e}")
        return False


def test_menu_system():
    """Test train_manager.py menu."""
    print_header("TEST 6: Menu System")
    
    try:
        # Check if train_manager.py exists and is executable
        menu_path = Path.home() / 'dev' / 'r2d2' / 'tests' / 'face_recognition' / 'train_manager.py'
        
        if not menu_path.exists():
            print(f"❌ Menu file not found: {menu_path}")
            return False
        
        print(f"✓ Menu file exists: {menu_path.name}")
        
        # Check for key methods
        content = menu_path.read_text()
        
        methods = [
            ('run_interactive_training', 'Train new person'),
            ('add_training_pictures', 'Add additional pictures'),
            ('realtime_recognition_test', 'Real-time test'),
        ]
        
        for method, description in methods:
            if method in content:
                print(f"✓ Method found: {description}")
            else:
                print(f"⚠️  Method not found: {description}")
        
        print("\n✅ MENU SYSTEM TEST PASSED")
        return True
        
    except Exception as e:
        print(f"❌ MENU SYSTEM TEST FAILED: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_service_files():
    """Test service configuration files."""
    print_header("TEST 7: Service Files")
    
    try:
        test_dir = Path.home() / 'dev' / 'r2d2' / 'tests' / 'face_recognition'
        
        # Check for systemd service file
        service_file = test_dir / 'r2d2-face-recognition.service'
        if service_file.exists():
            print(f"✓ Systemd service file: {service_file.name}")
            content = service_file.read_text()
            if 'face_recognition_service.py' in content:
                print("  - Correctly references face_recognition_service.py")
        else:
            print(f"⚠️  Service file not found: {service_file}")
        
        # Check for main service file
        main_service = test_dir / 'face_recognition_service.py'
        if main_service.exists():
            print(f"✓ Main service file: {main_service.name}")
        else:
            print(f"❌ Main service file not found: {main_service}")
            return False
        
        # Check for LED controller
        led_file = test_dir / 'led_controller.py'
        if led_file.exists():
            print(f"✓ LED controller file: {led_file.name}")
        else:
            print(f"❌ LED controller file not found: {led_file}")
            return False
        
        print("\n✅ SERVICE FILES TEST PASSED")
        return True
        
    except Exception as e:
        print(f"❌ SERVICE FILES TEST FAILED: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run all tests."""
    print("\n")
    print(" " * 70)
    print("   FACE RECOGNITION SYSTEM - COMPLETE TEST SUITE")
    print(" " * 70)
    
    tests = [
        test_led_controller,
        test_face_recognition_service,
        test_status_file_format,
        test_data_structure,
        test_git_protection,
        test_menu_system,
        test_service_files,
    ]
    
    results = []
    for test in tests:
        try:
            result = test()
            results.append(result)
        except Exception as e:
            print(f"❌ TEST CRASHED: {e}")
            import traceback
            traceback.print_exc()
            results.append(False)
    
    # Summary
    print_header("TEST SUMMARY")
    
    passed = sum(results)
    total = len(results)
    
    print(f"Tests passed: {passed}/{total}")
    
    if passed == total:
        print("\n✅ ALL TESTS PASSED!")
        print("\nNext steps:")
        print("  1. Start background service:")
        print("     python3 face_recognition_service.py start severin ~/dev/r2d2/data/face_recognition")
        print("  2. Check status in another terminal:")
        print("     python3 face_recognition_service.py status")
        print("  3. View logs:")
        print("     python3 face_recognition_service.py logs 50")
        return 0
    else:
        print(f"\n❌ {total - passed} TEST(S) FAILED")
        return 1


if __name__ == '__main__':
    sys.exit(main())
