#!/usr/bin/env python3
"""
Modular Face Training Data Capture Script

Purpose:
  Captures diverse training images for a specific person.
  Called by train_manager.py with person name and data directory.
  Provides clear, step-by-step instructions with pauses between stages.

Usage (via manager):
  python3 train_manager.py  (select "Capture training images")

Usage (direct):
  python3 _capture_module.py <person_name> <data_base_dir>

Author: R2D2 Perception Pipeline
Date: December 6, 2025
"""

import depthai as dai
import cv2
import os
import sys
import time
from datetime import datetime
from pathlib import Path


class CaptureModule:
    """Captures training images for face recognition."""
    
    def __init__(self, person_name, data_dir):
        """
        Initialize capture module.
        
        Args:
            person_name: Name of person being trained
            data_dir: Base data directory path
        """
        self.person_name = person_name
        self.output_dir = Path(data_dir) / person_name
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        self.frame_count = 0
        self.total_saved = 0
        
        # Initialize OAK-D camera
        print('\n[Initializing] Camera startup...')
        self.pipeline = dai.Pipeline()
        self.cam = self.pipeline.createColorCamera()
        self.cam.setBoardSocket(dai.CameraBoardSocket.RGB)
        self.cam.setResolution(dai.ColorCameraProperties.SensorSize.THE_1080_P)
        self.cam.setFps(30)
        
        self.out = self.pipeline.createXLinkOut()
        self.out.setStreamName('rgb')
        self.cam.video.link(self.out.input)
        
        self.device = dai.Device(self.pipeline)
        self.queue = self.device.getOutputQueue(name='rgb', maxSize=4, blocking=False)
        
        print('✓ OAK-D Lite camera initialized')
        
        # Load face cascade
        cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        if not os.path.exists(cascade_path):
            cascade_path = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'
        
        self.face_cascade = cv2.CascadeClassifier(cascade_path)
        if self.face_cascade.empty():
            raise RuntimeError(f'Failed to load Haar Cascade')
        
        print(f'✓ Haar Cascade loaded: {cascade_path}')
        print(f'✓ Output directory: {self.output_dir}')
    
    def show_instruction(self, title, instruction_text):
        """Display clear instruction with visual separation."""
        print('\n' + '='*70)
        print(f'STAGE: {title.upper()}')
        print('='*70)
        print(f'\n{instruction_text}\n')
        print('='*70)
        input('Press ENTER when ready to capture...')
        print('Capturing... (10 seconds)\n')
    
    def capture_stage(self, stage_name, title, instruction_text, duration=10):
        """
        Capture images for one stage with clear feedback.
        
        Args:
            stage_name: Internal identifier (for filenames)
            title: Display title
            instruction_text: Clear instruction to user
            duration: Seconds to capture
        """
        self.show_instruction(title, instruction_text)
        
        start_time = time.time()
        stage_saved = 0
        stage_frames = 0
        faces_detected = 0
        
        print('Capturing...', end='', flush=True)
        
        while True:
            in_frame = self.queue.get()
            frame = in_frame.getCvFrame()
            stage_frames += 1
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(
                gray,
                scaleFactor=1.05,
                minNeighbors=5,
                minSize=(50, 50),
                maxSize=(500, 500)
            )
            
            if len(faces) > 0:
                faces_detected += 1
                
                for (x, y, w, h) in faces:
                    face_roi = frame[y:y+h, x:x+w]
                    face_resized = cv2.resize(face_roi, (100, 100))
                    face_gray = cv2.cvtColor(face_resized, cv2.COLOR_BGR2GRAY)
                    
                    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                    filename = f'{timestamp}_{stage_name}_{stage_saved:03d}.jpg'
                    filepath = self.output_dir / filename
                    
                    cv2.imwrite(str(filepath), face_gray)
                    stage_saved += 1
                    self.total_saved += 1
            
            elapsed = time.time() - start_time
            
            # Progress indicator
            if int(elapsed) % 2 == 0 and int(elapsed) > 0:
                print('.', end='', flush=True)
            
            if elapsed > duration:
                break
        
        print(f' Done!\n')
        print(f'Stage Results:')
        print(f'  ✓ Frames captured: {stage_frames}')
        print(f'  ✓ Faces detected: {faces_detected}')
        print(f'  ✓ Images saved: {stage_saved}')
        print(f'  ✓ Total so far: {self.total_saved}')
    
    def run(self):
        """Run the complete capture sequence."""
        print('\n' + '='*70)
        print(f'TRAINING DATA CAPTURE FOR: {self.person_name.upper()}')
        print('='*70)
        print('\nYou will be guided through 4 capture stages.')
        print('Each stage: 10 seconds of video capture.')
        print('Face detection is automatic - stand naturally and move.')
        print('\nAt each stage, you\'ll be asked to press ENTER before starting.')
        print('This gives you time to position yourself correctly.\n')
        
        input('Press ENTER to begin first stage...')
        
        try:
            # Stage 1: Bright direct lighting
            self.capture_stage(
                'bright_direct',
                'Bright Direct Light',
                'Position yourself in bright, direct light (e.g., sunny window).\n'
                'Stand 1 meter from camera, facing forward.\n'
                'Move slowly: left/right, up/down to vary angles.\n'
                'Try different expressions: neutral, smiling, surprised.'
            )
            
            # Stage 2: Dim indoor lighting
            self.capture_stage(
                'dim_indoor',
                'Dim Indoor Light',
                'Move to a dimly lit area (no direct sunlight).\n'
                'Stand 1 meter from camera, facing forward.\n'
                'Move slowly: left/right, up/down to vary angles.\n'
                'This trains the model to recognize you in normal indoor light.'
            )
            
            # Stage 3: Side profile (45 degrees)
            self.capture_stage(
                'side_45deg',
                'Side Profile - 45 Degrees',
                'Return to bright area, stand at 45-degree angle to camera.\n'
                'This teaches the model your side profile.\n'
                'Move slowly: forward/back to vary distance.\n'
                'Rotate between 45° left and 45° right profiles.'
            )
            
            # Stage 4: Varied distance
            self.capture_stage(
                'varied_distance',
                'Varied Distance (1m → 3m)',
                'Stand 1 meter away for first 5 seconds.\n'
                'Then slowly walk back to 2 meters for middle 5 seconds.\n'
                'Finally, stand at 3 meters for last few seconds.\n'
                'Move your head: side-to-side and up-down at each distance.'
            )
            
            # Summary
            print('\n' + '='*70)
            print('CAPTURE COMPLETE')
            print('='*70)
            print(f'\nTotal images captured: {self.total_saved}')
            print(f'Output directory: {self.output_dir}')
            print(f'\nRecommendation:')
            if self.total_saved < 50:
                print(f'  ⚠️  You captured {self.total_saved} images.')
                print(f'  → Consider running capture again (add more images)')
                print(f'  → Better results with 75-100+ images')
            elif self.total_saved < 75:
                print(f'  ✓ {self.total_saved} images is good.')
                print(f'  → You can proceed to training')
                print(f'  → Consider adding more for even better accuracy')
            else:
                print(f'  ✓ {self.total_saved} images is excellent!')
                print(f'  → Ready for training')
            
            print(f'\nNext step: Train model from captured images.')
            print('='*70 + '\n')
            
            return True
            
        except KeyboardInterrupt:
            print('\n\n⚠️  Capture interrupted by user.')
            return False
        finally:
            self.device.close()


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('Usage: python3 _capture_module.py <person_name> <data_dir>')
        sys.exit(1)
    
    try:
        person_name = sys.argv[1]
        data_dir = sys.argv[2]
        
        capture = CaptureModule(person_name, data_dir)
        success = capture.run()
        sys.exit(0 if success else 1)
    except Exception as e:
        print(f'\n❌ Error: {e}')
        sys.exit(1)
