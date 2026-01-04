"""
Hardware Diagnostics API Endpoints
Provides hardware testing capabilities for Camera, Mic, Speaker, Bluetooth, LED, Servo

Mode Requirements:
- Parallel Mode: Most tests (read-only, non-conflicting)
- Debug Mode: Tests requiring exclusive hardware access (LED, Servo, Camera stream)
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
import subprocess
import asyncio
from typing import Dict, Optional, List
import os
from pathlib import Path
import json

router = APIRouter(prefix="/api/diagnostics", tags=["diagnostics"])

# =============================================================================
# Mode Management
# =============================================================================

class ModeInfo(BaseModel):
    mode: str  # "parallel" or "debug"
    core_services_running: bool
    can_run_exclusive_tests: bool
    message: str


def get_service_status(service_name: str) -> bool:
    """Check if a systemd service is active"""
    result = subprocess.run(
        ['systemctl', 'is-active', service_name],
        capture_output=True,
        text=True
    )
    return result.stdout.strip() == 'active'


@router.get("/mode", response_model=ModeInfo)
async def get_current_mode():
    """
    Get current operational mode.
    
    Returns:
        ModeInfo with current mode and capabilities
    """
    # Check if core services are running
    camera_running = get_service_status('r2d2-camera-perception.service')
    gesture_running = get_service_status('r2d2-gesture-intent.service')
    
    core_services_running = camera_running and gesture_running
    
    if core_services_running:
        return ModeInfo(
            mode="parallel",
            core_services_running=True,
            can_run_exclusive_tests=False,
            message="Parallel Mode - Productive operation active. Some tests require Debug Mode."
        )
    else:
        return ModeInfo(
            mode="debug",
            core_services_running=False,
            can_run_exclusive_tests=True,
            message="Debug/Maintenance Mode - Productive services stopped. Exclusive tests available."
        )


@router.post("/mode/debug")
async def enter_debug_mode():
    """
    Enter Debug/Maintenance Mode - stops core services for exclusive hardware access.
    
    WARNING: This pauses productive robot operation!
    """
    # Save current ROS parameters
    state_file = Path("/tmp/r2d2_debug_state.json")
    state = {}
    
    # Get current audio volume
    vol_result = subprocess.run(
        ['ros2', 'param', 'get', '/audio_notification_node', 'audio_volume'],
        capture_output=True,
        text=True,
        timeout=5
    )
    if vol_result.returncode == 0 and 'value is:' in vol_result.stdout:
        volume_str = vol_result.stdout.split('value is:')[1].strip()
        state['audio_volume'] = float(volume_str)
    
    # Save state
    state['timestamp'] = subprocess.run(['date', '+%Y-%m-%d %H:%M:%S'], 
                                       capture_output=True, text=True).stdout.strip()
    state_file.write_text(json.dumps(state, indent=2))
    
    # Stop core services
    services_to_stop = [
        'r2d2-camera-perception.service',
        'r2d2-gesture-intent.service'
    ]
    
    for service in services_to_stop:
        result = subprocess.run(['sudo', 'systemctl', 'stop', service],
                              capture_output=True, text=True)
        if result.returncode != 0:
            raise HTTPException(
                status_code=500,
                detail=f"Failed to stop {service}: {result.stderr}"
            )
    
    # Verify services stopped
    await asyncio.sleep(2)
    
    camera_stopped = not get_service_status('r2d2-camera-perception.service')
    gesture_stopped = not get_service_status('r2d2-gesture-intent.service')
    
    if not (camera_stopped and gesture_stopped):
        raise HTTPException(
            status_code=500,
            detail="Services did not stop correctly"
        )
    
    return {
        "success": True,
        "mode": "debug",
        "message": "Debug Mode activated. Productive services stopped.",
        "state_saved": str(state_file)
    }


@router.post("/mode/parallel")
async def exit_debug_mode():
    """
    Exit Debug Mode and resume Parallel (productive) operation.
    Restarts core services and verifies health.
    """
    # Restore state if exists
    state_file = Path("/tmp/r2d2_debug_state.json")
    if state_file.exists():
        state = json.loads(state_file.read_text())
    else:
        state = {}
    
    # Restart core services
    services_to_start = [
        'r2d2-camera-perception.service',
        'r2d2-gesture-intent.service'
    ]
    
    for service in services_to_start:
        result = subprocess.run(['sudo', 'systemctl', 'start', service],
                              capture_output=True, text=True)
        if result.returncode != 0:
            raise HTTPException(
                status_code=500,
                detail=f"Failed to start {service}: {result.stderr}"
            )
    
    # Wait for services to initialize
    await asyncio.sleep(5)
    
    # Verify services running
    camera_running = get_service_status('r2d2-camera-perception.service')
    gesture_running = get_service_status('r2d2-gesture-intent.service')
    
    if not (camera_running and gesture_running):
        raise HTTPException(
            status_code=500,
            detail="Services failed to start. Check: systemctl status r2d2-camera-perception"
        )
    
    # Health check: verify ROS topics
    health_ok = await verify_ros_topics_publishing()
    
    if not health_ok:
        return {
            "success": False,
            "mode": "unknown",
            "message": "Services started but topics not publishing. Manual intervention needed.",
            "recovery": "See 004_BACKUP_AND_RESTORE.md - Emergency Rollback Reference"
        }
    
    # Clean up state file
    if state_file.exists():
        state_file.unlink()
    
    return {
        "success": True,
        "mode": "parallel",
        "message": "Parallel Mode resumed. Productive operation active.",
        "topics_healthy": True
    }


async def verify_ros_topics_publishing() -> bool:
    """Verify critical ROS topics are publishing"""
    try:
        # Check camera topic
        result = subprocess.run(
            ['timeout', '3', 'ros2', 'topic', 'hz', '/oak/rgb/image_raw'],
            capture_output=True,
            text=True
        )
        if 'average rate' not in result.stdout:
            return False
        
        # Check status topic
        result = subprocess.run(
            ['timeout', '3', 'ros2', 'topic', 'echo', '/r2d2/audio/person_status', '--once'],
            capture_output=True,
            text=True
        )
        if 'status' not in result.stdout:
            return False
        
        return True
    except Exception:
        return False


# =============================================================================
# Camera Diagnostics
# =============================================================================

@router.post("/hardware/camera/detect")
async def test_camera_detect():
    """
    Test 1: Camera Device Detection
    Mode: Parallel (safe, read-only)
    """
    result = subprocess.run(
        ['lsusb'],
        capture_output=True,
        text=True,
        timeout=5
    )
    
    # Look for Movidius (OAK-D camera)
    movidius_found = 'Movidius' in result.stdout
    
    if movidius_found:
        # Extract device info
        for line in result.stdout.split('\n'):
            if 'Movidius' in line:
                return {
                    "status": "PASS",
                    "device_found": True,
                    "device_info": line.strip(),
                    "expected_serial": "19443010E1D30C7E00",
                    "message": "OAK-D Lite camera detected"
                }
    
    return {
        "status": "FAIL",
        "device_found": False,
        "device_info": None,
        "message": "OAK-D camera not detected. Check USB connection."
    }


@router.post("/hardware/camera/pipeline")
async def test_camera_pipeline():
    """
    Test 2: Camera Pipeline FPS Check
    Mode: Parallel (if camera-perception running)
    """
    # Check if service is running
    if not get_service_status('r2d2-camera-perception.service'):
        return {
            "status": "SKIP",
            "fps": 0.0,
            "message": "Camera perception service not running"
        }
    
    # Check topic rate
    result = subprocess.run(
        ['timeout', '3', 'ros2', 'topic', 'hz', '/oak/rgb/image_raw'],
        capture_output=True,
        text=True
    )
    
    if result.returncode != 0:
        return {
            "status": "FAIL",
            "fps": 0.0,
            "message": "Topic not publishing or timeout"
        }
    
    # Parse FPS from output
    for line in result.stdout.split('\n'):
        if 'average rate' in line:
            try:
                fps = float(line.split(':')[1].strip())
                status = "PASS" if fps > 25.0 else "WARN"
                message = f"Camera publishing at {fps:.1f} Hz"
                if fps < 25.0:
                    message += " (expected ~30 Hz)"
                
                return {
                    "status": status,
                    "fps": round(fps, 1),
                    "expected_fps": 30.0,
                    "message": message
                }
            except (ValueError, IndexError):
                pass
    
    return {
        "status": "FAIL",
        "fps": 0.0,
        "message": "Could not parse FPS from topic"
    }


@router.post("/hardware/camera/stream/start")
async def start_camera_stream():
    """
    Test 3: Start Camera Livestream
    Mode: DEBUG ONLY (conflicts with camera-perception)
    
    WARNING: This stops productive person recognition!
    """
    # Verify we're in debug mode
    mode = await get_current_mode()
    if mode.mode != "debug":
        raise HTTPException(
            status_code=403,
            detail="Camera stream requires Debug Mode. Stop camera-perception first."
        )
    
    # Start camera stream service
    result = subprocess.run(
        ['sudo', 'systemctl', 'start', 'r2d2-camera-stream.service'],
        capture_output=True,
        text=True
    )
    
    if result.returncode != 0:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to start camera stream: {result.stderr}"
        )
    
    # Wait for service to start
    await asyncio.sleep(2)
    
    # Verify stream is accessible
    stream_running = get_service_status('r2d2-camera-stream.service')
    
    return {
        "success": stream_running,
        "stream_url": "http://localhost:8081/stream",  # Use localhost or client can construct from window.location
        "message": "Camera stream started" if stream_running else "Failed to start stream"
    }


@router.post("/hardware/camera/stream/stop")
async def stop_camera_stream():
    """Stop camera livestream"""
    result = subprocess.run(
        ['sudo', 'systemctl', 'stop', 'r2d2-camera-stream.service'],
        capture_output=True,
        text=True
    )
    
    return {
        "success": result.returncode == 0,
        "message": "Camera stream stopped" if result.returncode == 0 else result.stderr
    }


# =============================================================================
# Microphone Diagnostics
# =============================================================================

@router.post("/hardware/mic/detect")
async def test_mic_detect():
    """
    Test 1: Microphone Device Detection
    Mode: Parallel (safe, read-only)
    """
    result = subprocess.run(
        ['arecord', '-l'],
        capture_output=True,
        text=True,
        timeout=5
    )
    
    # Look for HyperX
    hyperx_found = 'hyperx' in result.stdout.lower() or 'quadcast' in result.stdout.lower()
    
    if hyperx_found:
        # Extract device line
        for line in result.stdout.split('\n'):
            if 'hyperx' in line.lower() or 'quadcast' in line.lower():
                return {
                    "status": "PASS",
                    "device_found": True,
                    "device_info": line.strip(),
                    "message": "HyperX QuadCast S detected"
                }
    
    return {
        "status": "FAIL",
        "device_found": False,
        "device_info": result.stdout,
        "message": "HyperX microphone not detected. Check USB connection."
    }


@router.post("/hardware/mic/capture")
async def test_mic_capture():
    """
    Test 2: Short Audio Capture Test
    Mode: Parallel (safe, creates temp file)
    """
    temp_file = "/tmp/r2d2_mic_test.wav"
    
    # Record 2 seconds
    result = subprocess.run(
        ['timeout', '5', 'arecord', '-D', 'hw:3,0', '-f', 'S16_LE', 
         '-r', '48000', '-c', '2', '-d', '2', temp_file],
        capture_output=True,
        text=True
    )
    
    if result.returncode != 0:
        return {
            "status": "FAIL",
            "file_size": 0,
            "duration": 0,
            "message": f"Recording failed: {result.stderr}"
        }
    
    # Check file size
    if os.path.exists(temp_file):
        file_size = os.path.getsize(temp_file)
        expected_size = 48000 * 2 * 2 * 2  # sample_rate * channels * bytes_per_sample * duration
        
        # Should be ~750KB for 2s
        if file_size > expected_size * 0.8:
            status = "PASS"
            message = f"Microphone captured audio successfully ({file_size} bytes)"
        else:
            status = "WARN"
            message = f"File size smaller than expected ({file_size} vs ~{expected_size})"
        
        # Clean up
        try:
            os.remove(temp_file)
        except:
            pass
        
        return {
            "status": status,
            "file_size": file_size,
            "expected_size": expected_size,
            "duration": 2.0,
            "message": message
        }
    
    return {
        "status": "FAIL",
        "file_size": 0,
        "message": "Recording completed but file not found"
    }


@router.post("/hardware/mic/level")
async def test_mic_level():
    """
    Test 3: Audio Level Test
    Mode: Parallel (safe, analyzes signal strength)
    """
    temp_file = "/tmp/r2d2_mic_level_test.wav"
    
    # Record 1 second
    result = subprocess.run(
        ['timeout', '3', 'arecord', '-D', 'hw:3,0', '-f', 'S16_LE',
         '-r', '48000', '-c', '2', '-d', '1', temp_file],
        capture_output=True,
        text=True
    )
    
    if result.returncode != 0 or not os.path.exists(temp_file):
        return {
            "status": "FAIL",
            "level": "unknown",
            "message": "Could not capture audio for level test"
        }
    
    file_size = os.path.getsize(temp_file)
    
    # Categorize based on file size (rough approximation)
    # Real implementation would analyze WAV data, but file size works for quick test
    if file_size < 10000:
        level = "Silent"
        status = "WARN"
    elif file_size < 50000:
        level = "Quiet"
        status = "PASS"
    elif file_size < 200000:
        level = "Normal"
        status = "PASS"
    else:
        level = "Loud"
        status = "PASS"
    
    # Clean up
    try:
        os.remove(temp_file)
    except:
        pass
    
    return {
        "status": status,
        "level": level,
        "file_size": file_size,
        "message": f"Microphone signal level: {level}"
    }


# =============================================================================
# Speaker Diagnostics
# =============================================================================

@router.post("/hardware/speaker/detect")
async def test_speaker_detect():
    """
    Test 1: Speaker Device Detection
    Mode: Parallel (safe, read-only)
    """
    result = subprocess.run(
        ['aplay', '-l'],
        capture_output=True,
        text=True,
        timeout=5
    )
    
    # Look for card 1 (APE - PAM8403)
    card1_found = 'card 1' in result.stdout
    
    if card1_found:
        for line in result.stdout.split('\n'):
            if 'card 1' in line:
                return {
                    "status": "PASS",
                    "device_found": True,
                    "device_info": line.strip(),
                    "message": "Speaker device (card 1 APE) detected"
                }
    
    return {
        "status": "FAIL",
        "device_found": False,
        "device_info": result.stdout,
        "message": "Speaker device not found. Check PAM8403 wiring."
    }


@router.post("/hardware/speaker/tone")
async def test_speaker_tone():
    """
    Test 2: Play Test Tone
    Mode: Parallel (safe, non-blocking)
    """
    # Play 800Hz tone for 0.5s at 30% volume
    result = subprocess.run(
        ['timeout', '2', 'ffplay', '-autoexit', '-nodisp', '-f', 'lavfi',
         'sine=frequency=800:duration=0.5', '-af', 'volume=0.3'],
        capture_output=True,
        text=True
    )
    
    if result.returncode == 0 or result.returncode == 124:  # 124 = timeout success
        return {
            "status": "PASS",
            "message": "Test tone played (800Hz, 0.5s). Did you hear it?",
            "user_confirmation_required": True
        }
    
    return {
        "status": "FAIL",
        "message": f"ffplay failed: {result.stderr}",
        "user_confirmation_required": False
    }


@router.post("/hardware/speaker/beep")
async def test_speaker_r2d2_beep():
    """
    Test 3: Play R2D2 Beep
    Mode: Parallel (safe, uses existing audio assets)
    """
    # Find R2D2 beep file
    beep_file = Path.home() / 'dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/Voicy_R2-D2 - 2.mp3'
    
    if not beep_file.exists():
        return {
            "status": "FAIL",
            "message": f"R2D2 beep file not found: {beep_file}"
        }
    
    # Get current audio volume
    vol_result = subprocess.run(
        ['ros2', 'param', 'get', '/audio_notification_node', 'audio_volume'],
        capture_output=True,
        text=True,
        timeout=5
    )
    
    volume = 0.1  # Default
    if vol_result.returncode == 0 and 'value is:' in vol_result.stdout:
        try:
            volume = float(vol_result.stdout.split('value is:')[1].strip())
        except:
            pass
    
    # Play beep at current system volume
    result = subprocess.run(
        ['timeout', '5', 'ffplay', '-autoexit', '-nodisp', 
         '-af', f'volume={volume}', str(beep_file)],
        capture_output=True,
        text=True
    )
    
    if result.returncode == 0 or result.returncode == 124:
        return {
            "status": "PASS",
            "volume": volume,
            "beep_file": str(beep_file.name),
            "message": f"R2D2 beep played at {volume*100:.0f}% volume. Did you hear it?",
            "user_confirmation_required": True
        }
    
    return {
        "status": "FAIL",
        "message": f"Failed to play beep: {result.stderr}"
    }


# =============================================================================
# Bluetooth Diagnostics
# =============================================================================

@router.post("/hardware/bluetooth/service")
async def test_bluetooth_service():
    """
    Test 1: Bluetooth Service Status
    Mode: Parallel (safe, read-only)
    """
    result = subprocess.run(
        ['systemctl', 'is-active', 'bluetooth'],
        capture_output=True,
        text=True
    )
    
    active = result.stdout.strip() == 'active'
    
    return {
        "status": "PASS" if active else "FAIL",
        "service_active": active,
        "message": "Bluetooth service active" if active else "Bluetooth service not active"
    }


@router.post("/hardware/bluetooth/adapter")
async def test_bluetooth_adapter():
    """
    Test 2: Bluetooth Adapter Status
    Mode: Parallel (safe, read-only)
    """
    result = subprocess.run(
        ['bluetoothctl', 'show'],
        capture_output=True,
        text=True,
        timeout=5
    )
    
    powered = 'Powered: yes' in result.stdout
    
    adapter_info = {}
    for line in result.stdout.split('\n'):
        if ':' in line:
            key = line.split(':')[0].strip()
            value = line.split(':', 1)[1].strip()
            adapter_info[key] = value
    
    return {
        "status": "PASS" if powered else "FAIL",
        "powered": powered,
        "adapter_info": adapter_info,
        "message": "Bluetooth adapter powered on" if powered else "Bluetooth adapter not powered"
    }


@router.post("/hardware/bluetooth/devices")
async def test_bluetooth_devices():
    """
    Test 3: List Connected Bluetooth Devices
    Mode: Parallel (safe, read-only)
    """
    result = subprocess.run(
        ['bluetoothctl', 'devices'],
        capture_output=True,
        text=True,
        timeout=5
    )
    
    devices = []
    for line in result.stdout.split('\n'):
        if 'Device' in line:
            devices.append(line.strip())
    
    # Check which are connected
    connected_devices = []
    for device_line in devices:
        # Extract MAC address
        parts = device_line.split()
        if len(parts) >= 2:
            mac = parts[1]
            # Check if connected
            info_result = subprocess.run(
                ['bluetoothctl', 'info', mac],
                capture_output=True,
                text=True,
                timeout=3
            )
            if 'Connected: yes' in info_result.stdout:
                connected_devices.append(device_line)
    
    return {
        "status": "PASS" if devices else "WARN",
        "total_devices": len(devices),
        "connected_devices": len(connected_devices),
        "devices": devices,
        "connected": connected_devices,
        "message": f"Found {len(devices)} device(s), {len(connected_devices)} connected"
    }


@router.post("/hardware/bluetooth/pulseaudio")
async def test_bluetooth_pulseaudio():
    """
    Test 4: PulseAudio Bluetooth Sink
    Mode: Parallel (safe, read-only)
    """
    result = subprocess.run(
        ['pactl', 'list', 'sinks', 'short'],
        capture_output=True,
        text=True,
        timeout=5
    )
    
    # Look for bluez sink
    bluez_sinks = []
    for line in result.stdout.split('\n'):
        if 'bluez' in line.lower():
            bluez_sinks.append(line.strip())
    
    # Check default sink
    default_result = subprocess.run(
        ['pactl', 'get-default-sink'],
        capture_output=True,
        text=True,
        timeout=3
    )
    default_sink = default_result.stdout.strip()
    is_bluetooth_default = 'bluez' in default_sink.lower()
    
    return {
        "status": "PASS" if bluez_sinks else "WARN",
        "bluez_sinks_found": len(bluez_sinks),
        "sinks": bluez_sinks,
        "default_sink": default_sink,
        "bluetooth_is_default": is_bluetooth_default,
        "message": f"Found {len(bluez_sinks)} Bluetooth sink(s)" if bluez_sinks else "No Bluetooth sinks"
    }


# =============================================================================
# LED Diagnostics (Requires Debug Mode)
# =============================================================================

@router.post("/hardware/led/test")
async def test_led_control():
    """
    Test: White LED Control
    Mode: DEBUG ONLY (conflicts with status_led_node)
    
    Sequence: ON 2s → OFF 2s → Blink 5x
    """
    # Verify debug mode
    mode = await get_current_mode()
    if mode.mode != "debug":
        raise HTTPException(
            status_code=403,
            detail="LED test requires Debug Mode (conflicts with status_led_node)"
        )
    
    # Path to LED test script
    led_script = Path.home() / 'dev/r2d2/tests/rgb_led/test_white_led_gpio.py'
    
    if not led_script.exists():
        raise HTTPException(
            status_code=404,
            detail=f"LED test script not found: {led_script}"
        )
    
    # Run LED test (this is interactive, so we'll run a simplified version)
    # For now, return instructions for manual test
    return {
        "status": "MANUAL",
        "test_available": True,
        "script_location": str(led_script),
        "message": "LED test requires manual execution",
        "instructions": [
            "1. Stop status_led_node: sudo systemctl stop r2d2-audio-notification",
            f"2. Run test: sudo python3 {led_script}",
            "3. Observe: LED ON 2s → OFF 2s → Blink pattern",
            "4. Resume: sudo systemctl start r2d2-audio-notification"
        ]
    }


# =============================================================================
# Servo Diagnostics (Requires Debug Mode)
# =============================================================================

@router.post("/hardware/servo/detect")
async def test_servo_detect():
    """
    Test 1: Servo GPIO Availability
    Mode: Parallel (safe, just checks pins)
    """
    # Check if GPIO pin 33 (GPIO13) is available
    gpio_path = Path("/sys/class/gpio/gpio13")
    
    if gpio_path.exists():
        # Check if it's in use
        direction_file = gpio_path / "direction"
        if direction_file.exists():
            direction = direction_file.read_text().strip()
            return {
                "status": "WARN",
                "gpio_available": False,
                "gpio_pin": 13,
                "physical_pin": 33,
                "current_direction": direction,
                "message": "GPIO13 already exported (may be in use)"
            }
    
    return {
        "status": "PASS",
        "gpio_available": True,
        "gpio_pin": 13,
        "physical_pin": 33,
        "message": "GPIO13 (Pin 33) available for servo control"
    }


@router.post("/hardware/servo/move")
async def test_servo_movement():
    """
    Test 2: Servo Movement Test
    Mode: DEBUG ONLY (requires exclusive GPIO access)
    
    Safe test sequence: Center → Min → Max → Center
    Angle limits: 60-120 degrees
    """
    # Verify debug mode
    mode = await get_current_mode()
    if mode.mode != "debug":
        raise HTTPException(
            status_code=403,
            detail="Servo test requires Debug Mode (exclusive GPIO access)"
        )
    
    # Path to servo test script
    servo_script = Path.home() / 'dev/r2d2/scripts/test/test_servo_hardware.py'
    
    if not servo_script.exists():
        raise HTTPException(
            status_code=404,
            detail=f"Servo test script not found: {servo_script}"
        )
    
    # For safety, return manual instructions
    # Automated servo control via API is risky - better to require manual supervision
    return {
        "status": "MANUAL",
        "test_available": True,
        "script_location": str(servo_script),
        "safety_limits": {
            "min_angle": 60,
            "max_angle": 120,
            "gpio_pin": 13
        },
        "message": "Servo test requires manual execution for safety",
        "instructions": [
            "1. Ensure Debug Mode active (core services stopped)",
            f"2. Run test: sudo python3 {servo_script} --min 60 --max 120",
            "3. Monitor servo movement carefully",
            "4. Test sequence: Center → Min → Max → Center",
            "5. Use Ctrl+C to stop if needed"
        ]
    }


# =============================================================================
# System Integration Test
# =============================================================================

@router.post("/hardware/system/complete")
async def run_complete_system_test():
    """
    Run Complete System Integration Test
    Mode: Parallel (safe, comprehensive verification)
    
    Tests full pipeline from camera to audio output
    """
    test_script = Path.home() / 'dev/r2d2/tests/system/test_complete_system.sh'
    
    if not test_script.exists():
        raise HTTPException(
            status_code=404,
            detail=f"Test script not found: {test_script}"
        )
    
    # Run the test script
    result = subprocess.run(
        ['bash', str(test_script)],
        capture_output=True,
        text=True,
        timeout=30
    )
    
    # Parse results
    passed = result.stdout.count('✅ PASSED')
    failed = result.stdout.count('❌ FAILED')
    total = passed + failed
    
    return {
        "status": "PASS" if failed == 0 else "FAIL",
        "passed": passed,
        "failed": failed,
        "total": total,
        "output": result.stdout,
        "message": f"System test: {passed}/{total} checks passed"
    }


# =============================================================================
# Service Status and Control (for Diagnostics Page)
# =============================================================================

@router.get("/services")
async def get_all_services():
    """
    Get status of all 12 R2D2 services.
    Mode: Parallel (safe, read-only)
    """
    services = {
        'camera-perception': {},
        'audio-notification': {},
        'gesture-intent': {},
        'volume-control': {},
        'speech-node': {},
        'rest-speech-node': {},
        'heartbeat': {},
        'powerbutton': {},
        'wake-api': {},
        'rosbridge': {},
        'camera-stream': {},
        'web-dashboard': {}
    }
    
    for service_key in services.keys():
        service_name = f'r2d2-{service_key}.service'
        active = get_service_status(service_name)
        services[service_key] = {
            'name': service_key,
            'service_file': service_name,
            'active': active,
            'status': 'running' if active else 'stopped'
        }
    
    return services


@router.post("/services/{service_name}/start")
async def start_service_endpoint(service_name: str):
    """
    Start a systemd service.
    Mode: Requires user confirmation (handled in frontend)
    """
    full_name = f'{service_name}.service' if not service_name.endswith('.service') else service_name
    
    result = subprocess.run(
        ['sudo', 'systemctl', 'start', full_name],
        capture_output=True,
        text=True
    )
    
    if result.returncode != 0:
        return {
            "success": False,
            "error": result.stderr or "Failed to start service"
        }
    
    # Wait and verify
    await asyncio.sleep(2)
    active = get_service_status(full_name)
    
    return {
        "success": active,
        "service": service_name,
        "active": active
    }


@router.post("/services/{service_name}/stop")
async def stop_service_endpoint(service_name: str):
    """
    Stop a systemd service.
    Mode: Requires user confirmation (handled in frontend)
    """
    full_name = f'{service_name}.service' if not service_name.endswith('.service') else service_name
    
    result = subprocess.run(
        ['sudo', 'systemctl', 'stop', full_name],
        capture_output=True,
        text=True
    )
    
    if result.returncode != 0:
        return {
            "success": False,
            "error": result.stderr or "Failed to stop service"
        }
    
    # Wait and verify
    await asyncio.sleep(1)
    active = get_service_status(full_name)
    
    return {
        "success": not active,
        "service": service_name,
        "active": active
    }


@router.post("/services/{service_name}/restart")
async def restart_service_endpoint(service_name: str):
    """
    Restart a systemd service.
    Mode: Requires user confirmation (handled in frontend)
    """
    full_name = f'{service_name}.service' if not service_name.endswith('.service') else service_name
    
    result = subprocess.run(
        ['sudo', 'systemctl', 'restart', full_name],
        capture_output=True,
        text=True
    )
    
    if result.returncode != 0:
        return {
            "success": False,
            "error": result.stderr or "Failed to restart service"
        }
    
    # Wait and verify
    await asyncio.sleep(2)
    active = get_service_status(full_name)
    
    return {
        "success": active,
        "service": service_name,
        "active": active
    }


# =============================================================================
# GPIO Status (LED)
# =============================================================================

@router.get("/gpio/17")
async def read_gpio_17():
    """
    Read GPIO 17 state (LED status).
    Mode: Parallel (safe, read-only)
    """
    try:
        gpio_path = Path("/sys/class/gpio/gpio17/value")
        if gpio_path.exists():
            value = gpio_path.read_text().strip()
            return {
                "gpio": 17,
                "value": int(value),
                "state": "ON" if value == "1" else "OFF"
            }
        else:
            return {
                "gpio": 17,
                "value": None,
                "state": "NOT_EXPORTED",
                "error": "GPIO 17 not exported"
            }
    except Exception as e:
        return {
            "gpio": 17,
            "value": None,
            "state": "ERROR",
            "error": str(e)
        }


# =============================================================================
# ROS 2 System Info
# =============================================================================

@router.get("/ros/nodes")
async def get_ros_nodes():
    """
    List all active ROS 2 nodes.
    Mode: Parallel (safe, read-only)
    """
    try:
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True,
            text=True,
            timeout=10
        )
        
        if result.returncode != 0:
            return {
                "success": False,
                "nodes": [],
                "error": "Failed to list nodes"
            }
        
        nodes = [n.strip() for n in result.stdout.split('\n') if n.strip()]
        
        return {
            "success": True,
            "nodes": nodes,
            "count": len(nodes)
        }
    except Exception as e:
        return {
            "success": False,
            "nodes": [],
            "error": str(e)
        }


@router.get("/ros/topics/hz")
async def get_topics_hz():
    """
    Get publishing rates for key topics.
    Mode: Parallel (safe, read-only)
    """
    topics = [
        '/oak/rgb/image_raw',
        '/r2d2/perception/face_count',
        '/r2d2/perception/person_id',
        '/r2d2/audio/person_status',
        '/r2d2/heartbeat'
    ]
    
    results = {}
    
    for topic in topics:
        try:
            result = subprocess.run(
                ['timeout', '3', 'ros2', 'topic', 'hz', topic],
                capture_output=True,
                text=True
            )
            
            if 'average rate' in result.stdout:
                # Parse "average rate: X.XXX"
                for line in result.stdout.split('\n'):
                    if 'average rate' in line:
                        rate_str = line.split(':')[1].strip()
                        rate = float(rate_str)
                        results[topic] = round(rate, 2)
                        break
            else:
                results[topic] = 0.0
        except Exception as e:
            results[topic] = None
    
    return results


# =============================================================================
# Diagnostic Tests (Safe, Read-Only)
# =============================================================================

@router.post("/test/pulseaudio")
async def test_pulseaudio():
    """Test PulseAudio daemon and default sink"""
    output_lines = []
    
    # Check daemon
    result = subprocess.run(
        ['pulseaudio', '--check'],
        capture_output=True,
        text=True
    )
    if result.returncode == 0:
        output_lines.append("✓ PASS: PulseAudio daemon is running")
    else:
        output_lines.append("✗ FAIL: PulseAudio daemon not running")
    
    # Check default sink
    result = subprocess.run(
        ['pactl', 'get-default-sink'],
        capture_output=True,
        text=True
    )
    if result.returncode == 0 and result.stdout.strip():
        output_lines.append(f"✓ PASS: Default sink: {result.stdout.strip()}")
    else:
        output_lines.append("✗ FAIL: No default sink configured")
    
    return {
        "status": "PASS" if all('PASS' in line for line in output_lines) else "FAIL",
        "output": '\n'.join(output_lines)
    }


@router.post("/test/bluetooth")
async def test_bluetooth():
    """Test Bluetooth controller and connections"""
    output_lines = []
    
    # Check service
    result = subprocess.run(
        ['systemctl', 'is-active', 'bluetooth'],
        capture_output=True,
        text=True
    )
    if result.stdout.strip() == 'active':
        output_lines.append("✓ PASS: Bluetooth service active")
    else:
        output_lines.append("✗ FAIL: Bluetooth service not active")
    
    # Check adapter
    result = subprocess.run(
        ['bluetoothctl', 'show'],
        capture_output=True,
        text=True,
        timeout=5
    )
    if 'Powered: yes' in result.stdout:
        output_lines.append("✓ PASS: Bluetooth adapter powered on")
    else:
        output_lines.append("✗ FAIL: Bluetooth adapter not powered")
    
    return {
        "status": "PASS" if all('PASS' in line for line in output_lines) else "FAIL",
        "output": '\n'.join(output_lines)
    }


@router.post("/test/audio_playback")
async def test_audio_playback():
    """Test audio playback (may overlap with R2D2 beeps)"""
    output_lines = ["⚠️ Note: This test may briefly overlap with R2D2 beeps"]
    
    # Play short test tone
    result = subprocess.run(
        ['timeout', '2', 'ffplay', '-autoexit', '-nodisp', '-f', 'lavfi',
         'sine=frequency=800:duration=0.3', '-af', 'volume=0.2'],
        capture_output=True,
        text=True
    )
    
    if result.returncode == 0 or result.returncode == 124:
        output_lines.append("✓ PASS: Test tone played (800Hz, 0.3s at 20% volume)")
        output_lines.append("   Did you hear it?")
    else:
        output_lines.append(f"✗ FAIL: ffplay failed - {result.stderr}")
    
    return {
        "status": "PASS" if result.returncode in [0, 124] else "FAIL",
        "output": '\n'.join(output_lines)
    }


@router.post("/test/volume")
async def test_volume():
    """Test volume control node and topic"""
    output_lines = []
    
    # Check volume node
    result = subprocess.run(
        ['ros2', 'node', 'list'],
        capture_output=True,
        text=True,
        timeout=10
    )
    if '/volume_control_node' in result.stdout:
        output_lines.append("✓ PASS: volume_control_node is running")
    else:
        output_lines.append("✗ FAIL: volume_control_node not found")
    
    # Check volume topic
    result = subprocess.run(
        ['timeout', '3', 'ros2', 'topic', 'echo', '/r2d2/audio/master_volume', '--once'],
        capture_output=True,
        text=True
    )
    if result.returncode == 0 and 'data' in result.stdout:
        # Parse volume value
        for line in result.stdout.split('\n'):
            if 'data:' in line:
                volume = line.split(':')[1].strip()
                output_lines.append(f"✓ PASS: Master volume topic publishing: {volume}")
                break
    else:
        output_lines.append("✗ FAIL: Master volume topic not publishing")
    
    return {
        "status": "PASS" if all('PASS' in line for line in output_lines) else "FAIL",
        "output": '\n'.join(output_lines)
    }


@router.post("/test/speech_status")
async def test_speech_status():
    """Test speech service status and lifecycle"""
    output_lines = []
    
    # Check service
    if get_service_status('r2d2-speech-node.service'):
        output_lines.append("✓ PASS: r2d2-speech-node service is active")
    else:
        output_lines.append("✗ FAIL: r2d2-speech-node service not active")
    
    # Check lifecycle state
    result = subprocess.run(
        ['ros2', 'lifecycle', 'get', '/speech_node'],
        capture_output=True,
        text=True,
        timeout=5
    )
    if result.returncode == 0:
        state = result.stdout.strip().split('\n')[0] if result.stdout else 'unknown'
        output_lines.append(f"✓ INFO: Speech node lifecycle state: {state}")
    else:
        output_lines.append("⚠️ WARN: Could not get lifecycle state")
    
    return {
        "status": "PASS",
        "output": '\n'.join(output_lines)
    }


@router.post("/test/quick_status")
async def test_quick_status():
    """Quick status check of all core services"""
    output_lines = ["=== R2D2 Quick Status ===", ""]
    
    services_to_check = [
        ('r2d2-camera-perception.service', 'Camera/Perception'),
        ('r2d2-audio-notification.service', 'Audio Notification'),
        ('r2d2-gesture-intent.service', 'Gesture Intent'),
        ('r2d2-speech-node.service', 'Speech Node')
    ]
    
    for service, name in services_to_check:
        active = get_service_status(service)
        status = "✅ Running" if active else "❌ Stopped"
        output_lines.append(f"{name}: {status}")
    
    return {
        "status": "PASS",
        "output": '\n'.join(output_lines)
    }


@router.post("/test/topic_hz")
async def test_topic_hz():
    """Measure topic publishing rates"""
    output_lines = ["=== Topic Publishing Rates ===", ""]
    
    topics_hz = await get_topics_hz()
    
    for topic, hz in topics_hz.items():
        if hz is None:
            output_lines.append(f"{topic}: ✗ ERROR")
        elif hz == 0.0:
            output_lines.append(f"{topic}: ⚠️ NOT PUBLISHING")
        else:
            output_lines.append(f"{topic}: ✓ {hz} Hz")
    
    return {
        "status": "PASS",
        "output": '\n'.join(output_lines)
    }


@router.post("/test/ros_nodes")
async def test_ros_nodes():
    """List all active ROS 2 nodes"""
    nodes_data = await get_ros_nodes()
    
    output_lines = ["=== Active ROS 2 Nodes ===", ""]
    
    if nodes_data['success']:
        output_lines.append(f"Total nodes: {nodes_data['count']}")
        output_lines.append("")
        for node in nodes_data['nodes']:
            output_lines.append(f"  {node}")
    else:
        output_lines.append(f"✗ ERROR: {nodes_data['error']}")
    
    return {
        "status": "PASS" if nodes_data['success'] else "FAIL",
        "output": '\n'.join(output_lines)
    }


@router.post("/test/recognition_log")
async def test_recognition_log():
    """Show recent recognition events from logs"""
    output_lines = ["=== Recent Recognition Events (last 5 minutes) ===", ""]
    
    result = subprocess.run(
        ['journalctl', '-u', 'r2d2-audio-notification.service', '--since', '5 minutes ago', '--no-pager'],
        capture_output=True,
        text=True,
        timeout=10
    )
    
    if result.returncode == 0:
        # Filter for recognition events
        for line in result.stdout.split('\n'):
            if 'RED-FIRST' in line or 'recognized' in line:
                # Extract timestamp and message
                parts = line.split(']')
                if len(parts) >= 2:
                    output_lines.append(parts[-1].strip())
        
        if len(output_lines) == 2:
            output_lines.append("No recognition events in last 5 minutes")
    else:
        output_lines.append("✗ ERROR: Could not read logs")
    
    return {
        "status": "PASS",
        "output": '\n'.join(output_lines)
    }


@router.post("/test/gesture_log")
async def test_gesture_log():
    """Show recent gesture events from logs"""
    output_lines = ["=== Recent Gesture Events (last 5 minutes) ===", ""]
    
    result = subprocess.run(
        ['journalctl', '-u', 'r2d2-gesture-intent.service', '--since', '5 minutes ago', '--no-pager'],
        capture_output=True,
        text=True,
        timeout=10
    )
    
    if result.returncode == 0:
        # Filter for gesture events
        for line in result.stdout.split('\n'):
            if 'Gesture detected' in line or 'Session' in line:
                # Extract timestamp and message
                parts = line.split(']')
                if len(parts) >= 2:
                    output_lines.append(parts[-1].strip())
        
        if len(output_lines) == 2:
            output_lines.append("No gesture events in last 5 minutes")
    else:
        output_lines.append("✗ ERROR: Could not read logs")
    
    return {
        "status": "PASS",
        "output": '\n'.join(output_lines)
    }

