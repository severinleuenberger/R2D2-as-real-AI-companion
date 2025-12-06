#!/usr/bin/env python3
"""
Background Face Recognition Service

Runs continuously as a background service:
- Uses 10-15% CPU (adjustable)
- Recognizes faces in real-time
- Maintains recognition status (5-second timeout)
- Architecture supports LED control (future feature)
- Status display (text for now, can add LED support)

Usage:
  python3 face_recognition_service.py start     # Start service
  python3 face_recognition_service.py stop      # Stop service
  python3 face_recognition_service.py status    # Show current status
  python3 face_recognition_service.py logs      # Show logs
"""

import depthai as dai
import cv2
import os
import sys
import time
import threading
import json
from pathlib import Path
from datetime import datetime, timedelta
import logging
import traceback

# Import LED controller from same directory
try:
    import importlib.util
    led_spec = importlib.util.spec_from_file_location("led_controller", 
                                                        Path(__file__).parent / "led_controller.py")
    led_module = importlib.util.module_from_spec(led_spec)
    led_spec.loader.exec_module(led_module)
    LED_AVAILABLE = True
except Exception as e:
    LED_AVAILABLE = False
    print(f"Warning: LED controller not available: {e}")


class FaceRecognitionService:
    """Background face recognition service."""
    
    def __init__(self, person_name, data_dir, confidence_threshold=70, cpu_limit=0.15):
        """
        Initialize service.
        
        Args:
            person_name: Name of person to recognize
            data_dir: Base data directory path
            confidence_threshold: Confidence threshold (lower = stricter)
            cpu_limit: CPU usage limit (0.0-1.0), controls frame skip rate
        """
        self.person_name = person_name
        self.data_dir = Path(data_dir)
        self.model_path = self.data_dir / 'models' / f'{person_name}_lbph.xml'
        self.confidence_threshold = confidence_threshold
        self.cpu_limit = cpu_limit
        
        # Status state
        self.recognized_person = None
        self.last_recognition_time = None
        self.last_lost_time = None  # When recognition was last lost
        self.recognition_timeout = 5  # 5 seconds to confirm loss (only applies when NOT recognized)
        self.status_lock = threading.Lock()
        
        # Service control
        self.running = False
        self.frame_count = 0
        self.skip_frames = int(1 / cpu_limit)  # Skip frames to limit CPU
        
        # Status file for inter-process communication
        self.status_file = Path.home() / '.r2d2_face_recognition_status.json'
        
        # Log file
        self.log_file = Path.home() / '.r2d2_face_recognition.log'
        
        # LED controller (text-based by default, can be swapped for GPIO/HTTP)
        self.led = None
        if LED_AVAILABLE:
            try:
                self.led = led_module.create_led_controller('text')
            except Exception as e:
                self.log(f"Warning: Could not initialize LED controller: {e}")
        
        self.log(f"Service initialized for '{person_name}' (threshold={confidence_threshold}, cpu_limit={cpu_limit*100:.0f}%)")
        
    def log(self, message):
        """Log a message."""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        print(log_entry)
        
        try:
            with open(self.log_file, 'a') as f:
                f.write(log_entry + '\n')
        except:
            pass
    
    def update_status(self, recognized_person=None):
        """
        Update the recognition status.
        
        - If face detected: IMMEDIATELY set status to recognized (no delay)
        - If face NOT detected: Keep last state for 5 seconds, then reset
        - Result: Responsive recognition, stable display when person leaves
        """
        with self.status_lock:
            current_time = datetime.now()
            
            if recognized_person:
                # Face detected - IMMEDIATELY recognize (no delay)
                self.recognized_person = recognized_person
                self.last_recognition_time = current_time
                self.last_lost_time = None
            else:
                # Face not detected
                # Mark when we lost detection (only first time)
                if self.last_lost_time is None:
                    self.last_lost_time = current_time
                
                # Only reset status after 5 seconds of no detection
                if self.last_recognition_time:
                    elapsed = (current_time - self.last_recognition_time).total_seconds()
                    if elapsed > self.recognition_timeout:
                        self.recognized_person = None
            
            # Write status to file for other processes to read
            status = {
                'timestamp': current_time.isoformat(),
                'recognized_person': self.recognized_person,
                'confidence_threshold': self.confidence_threshold,
                'frame_count': self.frame_count,
            }
            
            try:
                with open(self.status_file, 'w') as f:
                    json.dump(status, f)
            except:
                pass
    
    def get_status(self):
        """Get current recognition status (already computed by update_status)."""
        with self.status_lock:
            return self.recognized_person
    
    def initialize_camera(self):
        """Initialize camera pipeline."""
        self.pipeline = dai.Pipeline()
        
        # Camera
        self.cam = self.pipeline.createColorCamera()
        self.cam.setBoardSocket(dai.CameraBoardSocket.RGB)
        self.cam.setVideoSize(1280, 720)  # Lower resolution to reduce CPU
        self.cam.setPreviewSize(1280, 720)
        self.cam.setFps(15)  # Lower FPS to reduce CPU
        self.cam.setInterleaved(False)
        
        # Output
        self.out = self.pipeline.createXLinkOut()
        self.out.setStreamName('rgb')
        self.cam.preview.link(self.out.input)
        
        # Start device
        self.device = dai.Device(self.pipeline)
        self.queue = self.device.getOutputQueue(name='rgb', maxSize=4, blocking=False)
        
        self.log(f"✓ Camera initialized (1280x720, 15 FPS)")
    
    def initialize_face_detector(self):
        """Load face detector."""
        cascade_path = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'
        if not os.path.exists(cascade_path):
            cascade_path = '/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml'
        
        self.face_detector = cv2.CascadeClassifier(cascade_path)
        if self.face_detector.empty():
            raise Exception('Failed to load face detector')
        
        self.log(f"✓ Face detector loaded")
    
    def initialize_recognizer(self):
        """Load trained recognizer."""
        if not self.model_path.exists():
            raise FileNotFoundError(f'Model not found: {self.model_path}')
        
        self.recognizer = cv2.face.LBPHFaceRecognizer_create()
        self.recognizer.read(str(self.model_path))
        
        self.log(f"✓ Model loaded: {self.model_path.name}")
    
    def run(self):
        """Run the recognition service."""
        try:
            self.log("="*70)
            self.log(f"FACE RECOGNITION SERVICE STARTING")
            self.log("="*70)
            
            # Initialize components
            self.initialize_camera()
            self.initialize_face_detector()
            self.initialize_recognizer()
            
            self.log(f"Recognizing person: {self.person_name.upper()}")
            self.log(f"Confidence threshold: {self.confidence_threshold}")
            self.log(f"Recognition timeout: {self.recognition_timeout}s (5-sec persistence when person leaves)")
            self.log(f"CPU limit: {self.cpu_limit*100:.0f}% (frame skip: {self.skip_frames})")
            self.log("")
            self.log("Service running. Press Ctrl+C to stop.")
            self.log("="*70)
            
            self.running = True
            last_status_display = None
            last_displayed_status = None
            
            while self.running:
                # Get frame
                in_frame = self.queue.get()
                if in_frame is None:
                    continue
                
                self.frame_count += 1
                
                # Skip frames to reduce CPU usage
                if self.frame_count % self.skip_frames != 0:
                    continue
                
                frame = in_frame.getCvFrame()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                
                # Detect faces
                faces = self.face_detector.detectMultiScale(gray, 1.3, 5)
                
                recognized = None
                
                if len(faces) > 0:
                    # Try to recognize each detected face
                    for (x, y, w, h) in faces:
                        face_roi = gray[y:y+h, x:x+w]
                        label, confidence = self.recognizer.predict(face_roi)
                        
                        # Check if confidence is good enough
                        if confidence < self.confidence_threshold:
                            recognized = self.person_name
                            break
                
                # Update status with smart timeout
                self.update_status(recognized)
                
                # Update LED display every 500ms (only if status changed)
                current_time = time.time()
                if last_status_display is None or current_time - last_status_display > 0.5:
                    status = self.get_status()
                    
                    # Only print if status actually changed
                    if status != last_displayed_status:
                        if self.led:
                            if status:
                                self.led.set_recognized(status)
                            else:
                                self.led.set_unrecognized()
                        else:
                            # Fallback to console display if LED not available
                            if status:
                                print(f"\r✅ RECOGNIZED: {status.upper()}", end='', flush=True)
                            else:
                                print(f"\r❌ No one recognized", end='', flush=True)
                        
                        last_displayed_status = status
                    
                    last_status_display = current_time
                
                # Small sleep to prevent busy-waiting
                time.sleep(0.01)
        
        except KeyboardInterrupt:
            self.log("")
            self.log("Service interrupted by user")
        except Exception as e:
            self.log(f"❌ Error: {e}")
            import traceback
            self.log(traceback.format_exc())
        finally:
            self.stop()
    
    def stop(self):
        """Stop the service."""
        self.running = False
        
        if hasattr(self, 'device'):
            self.device.close()
        
        self.log("")
        self.log("="*70)
        self.log(f"FACE RECOGNITION SERVICE STOPPED")
        self.log(f"Total frames processed: {self.frame_count}")
        self.log("="*70)
    
    @staticmethod
    def get_current_status():
        """Get current recognition status from status file."""
        status_file = Path.home() / '.r2d2_face_recognition_status.json'
        
        try:
            with open(status_file, 'r') as f:
                data = json.load(f)
                return data
        except:
            return None
    
    @staticmethod
    def show_logs(lines=50):
        """Show recent logs."""
        log_file = Path.home() / '.r2d2_face_recognition.log'
        
        try:
            with open(log_file, 'r') as f:
                all_lines = f.readlines()
                recent_lines = all_lines[-lines:]
                print(''.join(recent_lines))
        except:
            print("No logs found")


def main():
    """Main entry point."""
    if len(sys.argv) < 2:
        print("Usage: python3 face_recognition_service.py <command> [person_name] [data_dir]")
        print()
        print("Commands:")
        print("  start [person_name] [data_dir]  - Start service")
        print("  stop                            - Stop service")
        print("  status                          - Show current status")
        print("  logs [lines]                    - Show recent logs")
        print()
        print("Example:")
        print("  python3 face_recognition_service.py start severin ~/dev/r2d2/data/face_recognition")
        print("  python3 face_recognition_service.py status")
        print("  python3 face_recognition_service.py logs 20")
        sys.exit(1)
    
    command = sys.argv[1]
    
    if command == 'start':
        if len(sys.argv) < 4:
            print("Usage: python3 face_recognition_service.py start <person_name> <data_dir>")
            sys.exit(1)
        
        person_name = sys.argv[2]
        data_dir = sys.argv[3]
        
        try:
            service = FaceRecognitionService(person_name, data_dir)
            service.run()
        except Exception as e:
            print(f"❌ Error: {e}")
            sys.exit(1)
    
    elif command == 'status':
        status = FaceRecognitionService.get_current_status()
        if status:
            print("="*70)
            print("FACE RECOGNITION SERVICE STATUS")
            print("="*70)
            print(f"Timestamp: {status.get('timestamp', 'N/A')}")
            person = status.get('recognized_person')
            if person:
                print(f"✅ RECOGNIZED: {person.upper()}")
            else:
                print(f"❌ No one recognized")
            print(f"Frames: {status.get('frame_count', 0)}")
            print(f"Threshold: {status.get('confidence_threshold', 'N/A')}")
            print("="*70)
        else:
            print("Service not running or no status available")
    
    elif command == 'logs':
        lines = 50
        if len(sys.argv) > 2:
            try:
                lines = int(sys.argv[2])
            except:
                pass
        FaceRecognitionService.show_logs(lines)
    
    else:
        print(f"Unknown command: {command}")
        sys.exit(1)


if __name__ == "__main__":
    main()
