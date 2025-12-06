#!/usr/bin/env python3
"""
Modular Face Recognition Testing Script

Purpose:
  Tests trained LBPH face recognizer at different distances and conditions.
  Called by train_manager.py with person name and data directory.
  Provides real-time feedback on recognition accuracy.

Usage (via manager):
  python3 train_manager.py  (select "Test recognizer")

Usage (direct):
  python3 _test_module.py <person_name> <data_dir>

Author: R2D2 Perception Pipeline
Date: December 6, 2025
"""

import depthai as dai
import cv2
import os
import sys
import time
from pathlib import Path


class TestingModule:
    """Tests trained LBPH face recognizer."""
    
    def __init__(self, person_name, data_dir):
        """
        Initialize testing module.
        
        Args:
            person_name: Name of trained person
            data_dir: Base data directory path
        """
        self.person_name = person_name
        self.data_dir = Path(data_dir)
        self.model_path = self.data_dir / 'models' / f'{person_name}_lbph.xml'
        
        if not self.model_path.exists():
            raise FileNotFoundError(f'Model not found: {self.model_path}')
        
        print('\n[Initializing] Camera and model...')
        
        # Initialize OAK-D camera
        self.pipeline = dai.Pipeline()
        self.cam = self.pipeline.createColorCamera()
        self.cam.setBoardSocket(dai.CameraBoardSocket.RGB)
        self.cam.setVideoSize(1920, 1080)
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
            raise RuntimeError('Failed to load Haar Cascade')
        
        print(f'✓ Haar Cascade loaded')
        
        # Load trained recognizer
        try:
            self.recognizer = cv2.face.LBPHFaceRecognizer_create()
            self.recognizer.read(str(self.model_path))
            print(f'✓ LBPH model loaded: {self.person_name}_lbph.xml')
        except Exception as e:
            raise RuntimeError(f'Failed to load model: {e}')
        
        self.confidence_threshold = 70  # 0-255 scale, higher = stricter
    
    def show_instruction(self, distance_m, title, instruction_text):
        """Display test instruction."""
        print('\n' + '='*70)
        print(f'DISTANCE TEST: {distance_m}m - {title.upper()}')
        print('='*70)
        print(f'\n{instruction_text}\n')
        print('='*70)
        input('Press ENTER to start 10-second test...')
        print('Testing... (10 seconds)\n')
    
    def test_distance(self, distance_m, title, instruction_text):
        """
        Test recognition at specific distance.
        
        Args:
            distance_m: Distance in meters
            title: Display title
            instruction_text: Instructions for user
        """
        self.show_instruction(distance_m, title, instruction_text)
        
        start_time = time.time()
        frames_processed = 0
        faces_detected = 0
        recognitions = 0
        correct_recognitions = 0
        confidence_sum = 0
        
        print('Testing...', end='', flush=True)
        
        while True:
            in_frame = self.queue.get()
            if in_frame is None:
                continue
            
            frame = in_frame.getCvFrame()
            frames_processed += 1
            
            # Downscale for faster processing
            frame_small = cv2.resize(frame, (640, 360))
            gray = cv2.cvtColor(frame_small, cv2.COLOR_BGR2GRAY)
            
            # Detect faces
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
                    # Extract ROI from original frame (scale coordinates back)
                    x, y, w, h = int(x*1.5), int(y*1.5), int(w*1.5), int(h*1.5)
                    x = max(0, x)
                    y = max(0, y)
                    w = min(frame.shape[1] - x, w)
                    h = min(frame.shape[0] - y, h)
                    
                    if w < 50 or h < 50:
                        continue
                    
                    face_roi = frame[y:y+h, x:x+w]
                    face_resized = cv2.resize(face_roi, (100, 100))
                    face_gray = cv2.cvtColor(face_resized, cv2.COLOR_BGR2GRAY)
                    
                    # Recognize
                    try:
                        label, confidence = self.recognizer.predict(face_gray)
                        recognitions += 1
                        confidence_sum += confidence
                        
                        if confidence < self.confidence_threshold:
                            correct_recognitions += 1
                    except Exception as e:
                        pass
            
            elapsed = time.time() - start_time
            
            if int(elapsed) % 2 == 0 and int(elapsed) > 0:
                print('.', end='', flush=True)
            
            if elapsed > 10:
                break
        
        print(' Done!\n')
        
        # Calculate statistics
        detection_rate = (faces_detected / frames_processed * 100) if frames_processed > 0 else 0
        recognition_rate = (correct_recognitions / recognitions * 100) if recognitions > 0 else 0
        avg_confidence = (confidence_sum / recognitions) if recognitions > 0 else 0
        
        print(f'Results at {distance_m}m:')
        print(f'  Frames processed: {frames_processed}')
        print(f'  Faces detected: {faces_detected} ({detection_rate:.1f}%)')
        print(f'  Recognitions: {recognitions}')
        print(f'  Correct recognitions: {correct_recognitions} ({recognition_rate:.1f}%)')
        print(f'  Avg confidence: {avg_confidence:.1f} (lower=better)')
        
        return {
            'distance': distance_m,
            'frames': frames_processed,
            'faces': faces_detected,
            'recognitions': recognitions,
            'correct': correct_recognitions,
            'accuracy': recognition_rate,
            'confidence': avg_confidence
        }
    
    def run(self):
        """Run complete testing sequence at multiple distances."""
        print('\n' + '='*70)
        print(f'TESTING FACE RECOGNIZER FOR: {self.person_name.upper()}')
        print('='*70)
        
        print('\nThis will test recognition accuracy at 4 different distances.')
        print('You\'ll position yourself and the recognizer will measure accuracy.')
        print()
        input('Press ENTER to start tests...')
        
        results = []
        
        try:
            # 1 meter test
            results.append(self.test_distance(
                1,
                'Close Range',
                'Stand exactly 1 meter from camera.\n'
                'Face the camera directly.\n'
                'Move slightly: nod head, turn left/right.\n'
                'This tests recognition at typical interaction distance.'
            ))
            
            # 2 meter test
            results.append(self.test_distance(
                2,
                'Medium Range',
                'Stand 2 meters from camera.\n'
                'Face the camera directly.\n'
                'Move slightly: nod head, turn left/right.\n'
                'This tests recognition at room distance.'
            ))
            
            # 3 meter test
            results.append(self.test_distance(
                3,
                'Far Range',
                'Stand 3 meters from camera.\n'
                'Face the camera directly.\n'
                'Move slightly: nod head, turn left/right.\n'
                'This tests recognition at maximum useful distance.'
            ))
            
            # Summary
            print('\n' + '='*70)
            print('TEST SUMMARY')
            print('='*70)
            
            for result in results:
                dist = result['distance']
                acc = result['accuracy']
                conf = result['confidence']
                
                if acc >= 50:
                    status = '✓ GOOD'
                elif acc >= 25:
                    status = '⚠️  FAIR'
                else:
                    status = '❌ POOR'
                
                print(f'\n{dist}m: {acc:.1f}% accuracy [{status}]')
                print(f'    Confidence: {conf:.1f} | Recognitions: {result["recognitions"]}')
            
            print('\n' + '='*70)
            print('RECOMMENDATIONS:')
            print('='*70)
            
            best_distance = max(results, key=lambda r: r['accuracy'])
            print(f'\nBest performance: {best_distance["distance"]}m ({best_distance["accuracy"]:.1f}%)')
            
            if any(r['accuracy'] >= 50 for r in results):
                print('\n✓ Recognition working well at some distances.')
                print('  Model ready for deployment in ROS 2.')
            else:
                print('\n⚠️  Recognition accuracy is low across all distances.')
                print('  Recommendations:')
                print('    • Capture more training images (add 50+ more)')
                print('    • Vary lighting conditions during capture')
                print('    • Include more profile/angle views')
                print('    • Retrain the model with expanded dataset')
            
            print('='*70 + '\n')
            
            return True
        
        except KeyboardInterrupt:
            print('\n\n⚠️  Testing interrupted by user.')
            return False
        finally:
            self.device.close()


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('Usage: python3 _test_module.py <person_name> <data_dir>')
        sys.exit(1)
    
    try:
        person_name = sys.argv[1]
        data_dir = sys.argv[2]
        
        testing = TestingModule(person_name, data_dir)
        success = testing.run()
        sys.exit(0 if success else 1)
    except Exception as e:
        print(f'\n❌ Error: {e}')
        sys.exit(1)
