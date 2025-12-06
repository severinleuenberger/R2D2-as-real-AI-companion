#!/usr/bin/env python3
"""
Face Recognition Demo Script for R2D2

Purpose:
  Demonstrates live face recognition at multiple distances.
  Guides user through interactive testing stages (1m, 2m, 3m, 5m).
  Uses trained LBPH model to recognize "Severin".

Usage:
  source ~/depthai_env/bin/activate
  export OPENBLAS_CORETYPE=ARMV8
  python3 3_test_recognizer_demo.py

Prerequisites:
  - Training images captured with 1_capture_training_data.py
  - LBPH model trained with 2_train_recognizer.py

Output:
  - Console: Recognition results for each frame
  - Summary: Recognition rate and average confidence per distance stage

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


class FaceRecognitionDemo:
    """Runs interactive face recognition demo at multiple distances."""
    
    def __init__(self):
        """Initialize camera pipeline and LBPH recognizer."""
        self.model_path = os.path.expanduser('~/dev/r2d2/data/face_recognition/models/severin_lbph.xml')
        self.recognition_threshold = 70.0  # Confidence threshold for "Severin"
        
        # Verify model exists
        if not os.path.exists(self.model_path):
            raise RuntimeError(
                f'LBPH model not found at {self.model_path}\n'
                'Please run 2_train_recognizer.py first.'
            )
        
        # Load face cascade for detection
        cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        if not os.path.exists(cascade_path):
            cascade_path = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'
        
        self.face_cascade = cv2.CascadeClassifier(cascade_path)
        if self.face_cascade.empty():
            raise RuntimeError(f'Failed to load Haar Cascade')
        
        # Load LBPH recognizer and read trained model
        self.recognizer = cv2.face.LBPHFaceRecognizer_create()
        self.recognizer.read(self.model_path)
        
        # Initialize OAK-D camera
        print('[Camera] Initializing OAK-D Lite...')
        self.pipeline = dai.Pipeline()
        self.cam = self.pipeline.createColorCamera()
        self.cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        self.cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        self.cam.setFps(30)
        
        self.out = self.pipeline.createXLinkOut()
        self.out.setStreamName('rgb')
        self.cam.video.link(self.out.input)
        
        self.device = dai.Device(self.pipeline)
        self.queue = self.device.getOutputQueue(name='rgb', maxSize=4, blocking=False)
        
        print('✓ OAK-D Lite connected')
        print(f'✓ LBPH model loaded: {self.model_path}')
        print(f'✓ Recognition threshold: {self.recognition_threshold}')
    
    def recognize_stage(self, stage_name, distance_meters, description, duration_seconds=10):
        """
        Run recognition test for one distance stage.
        
        Args:
            stage_name: Identifier (e.g., "1m")
            distance_meters: Distance in meters
            description: User instruction
            duration_seconds: How long to test
        
        Returns:
            dict: Recognition statistics for this stage
        """
        print(f'\n{"="*70}')
        print(f'DISTANCE STAGE: {distance_meters}m')
        print('='*70)
        print(description)
        print(f'Testing for {duration_seconds} seconds...\n')
        
        start_time = time.time()
        frame_count = 0
        severin_recognized = 0
        severin_confidences = []
        unknown_confidences = []
        frames_with_faces = 0
        
        while True:
            # Get frame from camera
            in_frame = self.queue.get()
            frame = in_frame.getCvFrame()
            frame_count += 1
            
            # Detect faces
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
                frames_with_faces += 1
                
                # Extract face ROI and resize to 100x100 (for LBPH)
                face_roi = gray[y:y+h, x:x+w]
                face_resized = cv2.resize(face_roi, (100, 100))
                
                # Recognize face using LBPH
                label, confidence = self.recognizer.predict(face_resized)
                
                # Interpret result
                # label=0 is Severin (from training)
                # confidence: lower is better (distance in LBPH space)
                is_severin = (confidence < self.recognition_threshold)
                
                if is_severin:
                    severin_recognized += 1
                    severin_confidences.append(confidence)
                    result_str = f'Severin recognized (confidence={confidence:.1f})'
                else:
                    unknown_confidences.append(confidence)
                    result_str = f'Unknown (confidence={confidence:.1f})'
                
                # Log this recognition
                print(f'Frame {frame_count:4d} | {result_str}')
            
            # Check for timeout (no interactive cv2.waitKey in headless mode)
            elapsed = time.time() - start_time
            if elapsed > duration_seconds:
                break
        
        # Calculate statistics
        stats = {
            'stage': distance_meters,
            'total_frames': frame_count,
            'frames_with_faces': frames_with_faces,
            'severin_recognized': severin_recognized,
            'recognition_rate': (severin_recognized / frames_with_faces * 100) if frames_with_faces > 0 else 0,
            'avg_severin_confidence': sum(severin_confidences) / len(severin_confidences) if severin_confidences else None,
            'avg_unknown_confidence': sum(unknown_confidences) / len(unknown_confidences) if unknown_confidences else None,
        }
        
        # Print stage summary
        print(f'\n--- Stage Summary (at {distance_meters}m) ---')
        print(f'Frames captured: {frame_count}')
        print(f'Frames with faces: {frames_with_faces}')
        print(f'Severin recognized: {severin_recognized}/{frames_with_faces}')
        print(f'Recognition rate: {stats["recognition_rate"]:.1f}%')
        if severin_confidences:
            print(f'Avg Severin confidence: {stats["avg_severin_confidence"]:.1f}')
        if unknown_confidences:
            print(f'Avg Unknown confidence: {stats["avg_unknown_confidence"]:.1f}')
        
        return stats
    
    def run(self):
        """Run the complete recognition test sequence."""
        print('\n' + '='*70)
        print('R2D2 FACE RECOGNITION DEMO - INTERACTIVE DISTANCE TEST')
        print('='*70)
        print('This script tests face recognition at multiple distances.')
        print('You will be instructed to position yourself at 1m, 2m, 3m, and 5m.')
        print('\nPress Ctrl+C to stop at any time.')
        print('Press \'q\' to skip to next distance.')
        print('='*70)
        
        input('\nPress ENTER to begin...')
        
        all_stats = []
        
        try:
            # 1 meter
            stats = self.recognize_stage(
                '1m',
                1,
                'DISTANCE: 1 METER\n'
                'Stand exactly 1 meter from camera.\n'
                'Look straight ahead. Move slowly side-to-side.\n'
                'Nod up/down to vary vertical angle.',
                duration_seconds=10
            )
            if stats:
                all_stats.append(stats)
            
            # 2 meters
            stats = self.recognize_stage(
                '2m',
                2,
                'DISTANCE: 2 METERS\n'
                'Move back to 2 meters from camera.\n'
                'Look straight ahead. Move side-to-side.\n'
                'Nod up/down to vary angle.',
                duration_seconds=10
            )
            if stats:
                all_stats.append(stats)
            
            # 3 meters
            stats = self.recognize_stage(
                '3m',
                3,
                'DISTANCE: 3 METERS\n'
                'Move back to 3 meters from camera.\n'
                'Look straight ahead. Move side-to-side.\n'
                'Nod up/down to vary angle.',
                duration_seconds=10
            )
            if stats:
                all_stats.append(stats)
            
            # 5 meters
            stats = self.recognize_stage(
                '5m',
                5,
                'DISTANCE: 5 METERS\n'
                'Move back to 5 meters from camera.\n'
                'Look straight ahead. Move side-to-side.\n'
                'Nod up/down to vary angle.',
                duration_seconds=10
            )
            if stats:
                all_stats.append(stats)
            
            # Final summary
            print('\n' + '='*70)
            print('RECOGNITION TEST COMPLETE - SUMMARY')
            print('='*70)
            
            for stats in all_stats:
                print(f'\n{stats["stage"]}m stage:')
                print(f'  Recognition rate: {stats["recognition_rate"]:.1f}%')
                print(f'  Frames with faces: {stats["frames_with_faces"]}')
                if stats["avg_severin_confidence"] is not None:
                    print(f'  Avg Severin confidence: {stats["avg_severin_confidence"]:.1f}')
            
            print('\n' + '='*70)
            print('Recommendations based on results:')
            print('  - If recognition < 70% at 1m: Retrain with more diverse images')
            print('  - If recognition < 50% at 2m: Check lighting, add more dim/bright images')
            print('  - Recognition at 5m depends on camera resolution and face detection')
            print('='*70 + '\n')
            
        except KeyboardInterrupt:
            print('\n\nDemo interrupted by user.')
        finally:
            cv2.destroyAllWindows()
            self.device.close()


if __name__ == '__main__':
    try:
        demo = FaceRecognitionDemo()
        demo.run()
    except Exception as e:
        print(f'\n❌ Error: {e}')
        exit(1)
