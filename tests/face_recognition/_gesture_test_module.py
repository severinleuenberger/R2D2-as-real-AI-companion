#!/usr/bin/env python3
"""
Modular Gesture Recognition Testing Script (Person-Specific)

Purpose:
  Tests trained gesture classifier in real-time.
  Called by train_manager.py with person name and data directory.
  Provides real-time feedback on gesture recognition accuracy.

Usage (via manager):
  python3 train_manager.py  (select "Test gesture classifier")

Usage (direct):
  python3 _gesture_test_module.py <person_name> <gesture_recognition_data_dir>

Author: R2D2 Perception Pipeline
Date: December 17, 2025
"""

import depthai as dai
import cv2
import os
import sys
import time
import numpy as np
from pathlib import Path
import pickle

try:
    import mediapipe as mp
except ImportError:
    print('\n❌ Error: MediaPipe not installed.')
    print('Install with: pip install mediapipe')
    sys.exit(1)


class GestureTestingModule:
    """Tests trained gesture classifier."""
    
    def __init__(self, person_name, data_dir):
        """
        Initialize gesture testing module.
        
        Args:
            person_name: Name of trained person
            data_dir: Base gesture recognition data directory
        """
        self.person_name = person_name
        self.data_dir = Path(data_dir)
        self.model_path = self.data_dir / 'models' / f'{person_name}_gesture_classifier.pkl'
        
        if not self.model_path.exists():
            raise FileNotFoundError(f'Model not found: {self.model_path}')
        
        print('\n[Initializing] Camera, MediaPipe, and model...')
        
        # Initialize MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        print('✓ MediaPipe Hands initialized')
        
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
        
        # Load trained model
        try:
            with open(self.model_path, 'rb') as f:
                model_data = pickle.load(f)
            
            self.classifier = model_data['classifier']
            self.scaler = model_data['scaler']
            self.label_to_gesture = model_data['label_to_gesture']
            self.gestures = model_data['gestures']
            
            print(f'✓ Gesture model loaded: {self.person_name}_gesture_classifier.pkl')
            print(f'  Gestures: {", ".join(self.gestures)}')
            
        except Exception as e:
            raise RuntimeError(f'Failed to load model: {e}')
        
        self.confidence_threshold = 0.6  # Minimum probability for confident prediction
    
    def extract_hand_landmarks(self, image):
        """Extract hand landmarks from image using MediaPipe."""
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image_rgb)
        
        if not results.multi_hand_landmarks:
            return None
        
        hand_landmarks = results.multi_hand_landmarks[0]
        landmarks = []
        for landmark in hand_landmarks.landmark:
            landmarks.extend([landmark.x, landmark.y, landmark.z])
        
        return np.array(landmarks)
    
    def normalize_landmarks(self, landmarks):
        """Normalize landmarks to be scale and position invariant."""
        landmarks_reshaped = landmarks.reshape(21, 3)
        wrist = landmarks_reshaped[0]
        landmarks_centered = landmarks_reshaped - wrist
        
        middle_mcp = landmarks_centered[9]
        hand_size = np.linalg.norm(middle_mcp)
        
        if hand_size > 0.001:
            landmarks_normalized = landmarks_centered / hand_size
        else:
            landmarks_normalized = landmarks_centered
        
        return landmarks_normalized.flatten()
    
    def predict_gesture(self, image):
        """
        Predict gesture from image.
        
        Returns:
            (gesture_name, confidence) or (None, 0.0)
        """
        # Extract landmarks
        landmarks = self.extract_hand_landmarks(image)
        
        if landmarks is None:
            return None, 0.0
        
        # Normalize
        landmarks_normalized = self.normalize_landmarks(landmarks)
        
        # Scale features
        features_scaled = self.scaler.transform([landmarks_normalized])
        
        # Predict
        prediction = self.classifier.predict(features_scaled)[0]
        probabilities = self.classifier.predict_proba(features_scaled)[0]
        
        gesture_name = self.label_to_gesture[prediction]
        confidence = max(probabilities)
        
        return gesture_name, confidence
    
    def show_instruction(self, test_duration):
        """Display test instruction."""
        print('\n' + '='*70)
        print(f'GESTURE RECOGNITION TEST - {test_duration} SECONDS')
        print('='*70)
        print('\nInstructions:')
        print('  1. Hold your hand in front of the camera')
        print('  2. Make the trained gestures:')
        print('     • Index finger up (pointing upward)')
        print('     • Fist (all fingers closed)')
        print('  3. Switch between gestures during the test')
        print('  4. The system will show detected gesture and confidence')
        print()
        print('='*70)
        input('Press ENTER to start test...')
        print(f'Testing for {test_duration} seconds...\n')
    
    def test_realtime(self, duration=30):
        """
        Test gesture recognition in real-time.
        
        Args:
            duration: Test duration in seconds
        """
        self.show_instruction(duration)
        
        start_time = time.time()
        frames_processed = 0
        hands_detected = 0
        gestures_detected = 0
        
        # Track gesture counts
        gesture_counts = {gesture: 0 for gesture in self.gestures}
        gesture_counts['no_hand'] = 0
        
        last_print_time = start_time
        
        print('Real-time gesture detection:')
        print('='*70)
        
        while True:
            in_frame = self.queue.get()
            if in_frame is None:
                continue
            
            frame = in_frame.getCvFrame()
            frames_processed += 1
            
            # Predict gesture
            gesture_name, confidence = self.predict_gesture(frame)
            
            current_time = time.time()
            
            if gesture_name is not None:
                hands_detected += 1
                
                if confidence >= self.confidence_threshold:
                    gestures_detected += 1
                    gesture_counts[gesture_name] += 1
                    
                    # Print update every 0.5 seconds
                    if current_time - last_print_time >= 0.5:
                        print(f'  [{current_time - start_time:5.1f}s] Detected: {gesture_name.replace("_", " ").upper()} (confidence: {confidence:.2f})')
                        last_print_time = current_time
            else:
                gesture_counts['no_hand'] += 1
            
            elapsed = current_time - start_time
            
            if elapsed > duration:
                break
        
        print('='*70)
        print('\nTest complete!\n')
        
        # Calculate statistics
        detection_rate = (hands_detected / frames_processed * 100) if frames_processed > 0 else 0
        recognition_rate = (gestures_detected / hands_detected * 100) if hands_detected > 0 else 0
        
        print('Test Results:')
        print('='*70)
        print(f'  Duration: {duration} seconds')
        print(f'  Frames processed: {frames_processed}')
        print(f'  Hands detected: {hands_detected} ({detection_rate:.1f}%)')
        print(f'  Gestures recognized: {gestures_detected} ({recognition_rate:.1f}%)')
        print()
        print('Gesture distribution:')
        for gesture, count in sorted(gesture_counts.items()):
            if gesture == 'no_hand':
                continue
            percentage = (count / gestures_detected * 100) if gestures_detected > 0 else 0
            print(f'  • {gesture.replace("_", " ")}: {count} times ({percentage:.1f}%)')
        print(f'  • no hand detected: {gesture_counts["no_hand"]} frames')
        
        return {
            'frames': frames_processed,
            'hands_detected': hands_detected,
            'gestures_recognized': gestures_detected,
            'detection_rate': detection_rate,
            'recognition_rate': recognition_rate,
            'gesture_counts': gesture_counts
        }
    
    def run(self):
        """Run complete testing sequence."""
        print('\n' + '='*70)
        print(f'TESTING GESTURE CLASSIFIER FOR: {self.person_name.upper()}')
        print('='*70)
        
        print('\nThis will test real-time gesture recognition.')
        print('Duration: 30 seconds')
        print()
        input('Press ENTER to start test...')
        
        try:
            results = self.test_realtime(duration=30)
            
            # Summary
            print('\n' + '='*70)
            print('TEST SUMMARY')
            print('='*70)
            
            if results['recognition_rate'] >= 70:
                status = '✓ EXCELLENT'
            elif results['recognition_rate'] >= 50:
                status = '✓ GOOD'
            elif results['recognition_rate'] >= 30:
                status = '⚠️  FAIR'
            else:
                status = '❌ POOR'
            
            print(f'\nOverall recognition rate: {results["recognition_rate"]:.1f}% [{status}]')
            print(f'Hand detection rate: {results["detection_rate"]:.1f}%')
            print(f'Total gestures recognized: {results["gestures_recognized"]}')
            
            print('\n' + '='*70)
            print('RECOMMENDATIONS:')
            print('='*70)
            
            if results['recognition_rate'] >= 50:
                print('\n✓ Gesture recognition working well.')
                print('  Model ready for deployment in ROS 2 perception node.')
            else:
                print('\n⚠️  Gesture recognition accuracy could be improved.')
                print('  Recommendations:')
                print('    • Capture more training images (add 20-30+ more per gesture)')
                print('    • Ensure clear gestures during capture')
                print('    • Vary hand positions and angles during capture')
                print('    • Retrain the model with expanded dataset')
            
            print('='*70 + '\n')
            
            return True
        
        except KeyboardInterrupt:
            print('\n\n⚠️  Testing interrupted by user.')
            return False
        finally:
            self.hands.close()
            self.device.close()


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('Usage: python3 _gesture_test_module.py <person_name> <gesture_recognition_data_dir>')
        sys.exit(1)
    
    try:
        person_name = sys.argv[1]
        data_dir = sys.argv[2]
        
        testing = GestureTestingModule(person_name, data_dir)
        success = testing.run()
        sys.exit(0 if success else 1)
    except Exception as e:
        print(f'\n❌ Error: {e}')
        import traceback
        traceback.print_exc()
        sys.exit(1)


