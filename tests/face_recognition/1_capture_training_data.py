#!/usr/bin/env python3
"""
Face Training Data Capture Script for R2D2

Purpose:
  Captures diverse face images of "Severin" from the OAK-D camera
  for training the LBPH face recognizer. Guides user through
  multiple lighting conditions and angles.

Usage:
  source ~/depthai_env/bin/activate
  export OPENBLAS_CORETYPE=ARMV8
  python3 1_capture_training_data.py

Output:
  Saves images to ~/dev/r2d2/data/face_recognition/severin/
  Each image is 100x100 pixels, extracted from detected faces.

Author: R2D2 Perception Pipeline
Date: December 6, 2025
"""

import depthai as dai
import cv2
import os
import time
from datetime import datetime

# Use non-GUI backend
cv2.setNumThreads(2)  # ARM optimization


class TrainingDataCapture:
    """Captures diverse face training images from OAK-D camera."""
    
    def __init__(self):
        """Initialize camera pipeline and face cascade."""
        self.output_dir = os.path.expanduser('~/dev/r2d2/data/face_recognition/severin')
        self.frame_count = 0
        self.total_saved = 0
        
        # Create output directory if it doesn't exist
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Load face cascade for detection
        cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        if not os.path.exists(cascade_path):
            cascade_path = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'
        
        self.face_cascade = cv2.CascadeClassifier(cascade_path)
        if self.face_cascade.empty():
            raise RuntimeError(f'Failed to load Haar Cascade from {cascade_path}')
        
        # Initialize OAK-D camera
        print('\n[Camera] Initializing OAK-D Lite...')
        self.pipeline = dai.Pipeline()
        self.cam = self.pipeline.createColorCamera()
        self.cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        self.cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        self.cam.setFps(30)
        
        # Create output node for frames
        self.out = self.pipeline.createXLinkOut()
        self.out.setStreamName('rgb')
        self.cam.video.link(self.out.input)
        
        # Connect to device
        self.device = dai.Device(self.pipeline)
        self.queue = self.device.getOutputQueue(name='rgb', maxSize=4, blocking=False)
        
        print(f'✓ OAK-D Lite connected successfully')
        print(f'✓ Output directory: {self.output_dir}')
        print(f'✓ Haar Cascade loaded: {cascade_path}')
    
    def capture_stage(self, stage_name, description, duration_seconds=10):
        """
        Capture images for one stage (lighting/angle condition).
        
        Args:
            stage_name: Identifier (e.g., "bright_direct")
            description: User-facing instruction text
            duration_seconds: How long to capture
        """
        print(f'\n' + '='*60)
        print(f'STAGE: {stage_name.upper()}')
        print('='*60)
        print(f'{description}')
        print(f'Capturing for {duration_seconds} seconds...\n')
        
        start_time = time.time()
        stage_saved = 0
        frame_count = 0
        
        while True:
            # Get frame from camera
            in_frame = self.queue.get()
            frame = in_frame.getCvFrame()
            frame_count += 1
            
            # Detect faces in frame
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(
                gray,
                scaleFactor=1.05,
                minNeighbors=5,
                minSize=(50, 50),
                maxSize=(500, 500)
            )
            
            # Process each detected face
            for (x, y, w, h) in faces:
                # Extract face ROI and resize to 100x100 (standard size for LBPH)
                face_roi = frame[y:y+h, x:x+w]
                face_resized = cv2.resize(face_roi, (100, 100))
                
                # Save with metadata in filename
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                filename = f'{timestamp}_{stage_name}_{stage_saved:03d}.jpg'
                filepath = os.path.join(self.output_dir, filename)
                
                # Save grayscale (LBPH prefers grayscale)
                face_gray = cv2.cvtColor(face_resized, cv2.COLOR_BGR2GRAY)
                cv2.imwrite(filepath, face_gray)
                
                stage_saved += 1
                self.total_saved += 1
            
            # Draw faces on frame for live feedback
            display_frame = frame.copy()
            for (x, y, w, h) in faces:
                cv2.rectangle(display_frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            
            # Display frame count and face count
            elapsed = time.time() - start_time
            face_count = len(faces)
            cv2.putText(
                display_frame,
                f'Faces: {face_count} | Saved this stage: {stage_saved}',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2
            )
            cv2.putText(
                display_frame,
                f'Time: {elapsed:.1f}s / {duration_seconds}s',
                (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2
            )
            
            # Log progress to console instead of displaying (headless environment)
            if frame_count % 30 == 0:  # Log every 30 frames (~1 second at 30 fps)
                print(f'  [{elapsed:.1f}s] {stage_saved} faces captured...')
            
            # Check for timeout (no cv2.waitKey in headless mode)
            if elapsed > duration_seconds:
                break
        
        print(f'✓ Stage complete: Saved {stage_saved} face images')
        return True
    
    def run(self):
        """Run the complete training data capture sequence."""
        print('\n' + '='*60)
        print('R2D2 FACE TRAINING DATA CAPTURE')
        print('='*60)
        print('This script will guide you through capturing training images.')
        print('You will be instructed to position yourself at different angles')
        print('and under different lighting conditions.')
        print('\nPress Ctrl+C to stop at any time.')
        print('Press \'q\' to skip to next stage.')
        print('='*60)
        
        input('\nPress ENTER to begin...')
        
        try:
            # Bright, direct lighting
            self.capture_stage(
                'bright_direct',
                'STAGE 1/4: Bright Direct Light (e.g., sunny window or lamp)\n'
                'Stand 1 meter from camera, look straight ahead.\n'
                'Move slowly left/right and nod up/down to vary angles.',
                duration_seconds=10
            )
            
            # Dim indoor lighting
            self.capture_stage(
                'dim_indoor',
                'STAGE 2/4: Dim Indoor Light\n'
                'Move to a dimly lit area (e.g., corner without direct light).\n'
                'Stand 1 meter from camera, look straight ahead.\n'
                'Move left/right and nod to vary angles.',
                duration_seconds=10
            )
            
            # Side angle (45 degrees)
            self.capture_stage(
                'side_45deg',
                'STAGE 3/4: Side Profile (45 degrees)\n'
                'Return to bright area, stand at 45-degree angle.\n'
                'Move slowly left/right while maintaining 45-degree angle.\n'
                'Nod up/down to vary vertical angle.',
                duration_seconds=10
            )
            
            # Multiple distances (vary distance from camera)
            self.capture_stage(
                'varied_distance',
                'STAGE 4/4: Varied Distance (1m, 2m, 3m)\n'
                'Stand 1 meter from camera for 3 seconds, then move back.\n'
                'Move to 2 meters away for 3 seconds.\n'
                'Move to 3 meters away for the rest of the stage.\n'
                'Look straight ahead and move side-to-side at each distance.',
                duration_seconds=15
            )
            
            # Summary
            print('\n' + '='*60)
            print('TRAINING DATA CAPTURE COMPLETE')
            print('='*60)
            print(f'Total images captured: {self.total_saved}')
            print(f'Output directory: {self.output_dir}')
            print('\nNext step: Run 2_train_recognizer.py to train the model.')
            print('='*60 + '\n')
            
        except KeyboardInterrupt:
            print('\n\nCapture interrupted by user.')
        finally:
            cv2.destroyAllWindows()
            self.device.close()


if __name__ == '__main__':
    try:
        capture = TrainingDataCapture()
        capture.run()
    except Exception as e:
        print(f'\n❌ Error: {e}')
        exit(1)
