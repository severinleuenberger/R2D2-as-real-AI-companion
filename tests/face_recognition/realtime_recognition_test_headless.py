#!/usr/bin/env python3
"""
Real-Time Face Recognition Test (No Display)
Continuously tries to recognize faces and provides instant terminal feedback.

Usage:
  python3 realtime_recognition_test_headless.py <person_name> <data_dir>

Example:
  python3 realtime_recognition_test_headless.py severin ~/dev/r2d2/data/face_recognition
"""

import depthai as dai
import cv2
import os
import sys
import time
from pathlib import Path


class RealtimeRecognitionTestHeadless:
    """Real-time face recognition with continuous terminal feedback."""
    
    def __init__(self, person_name, data_dir):
        self.person_name = person_name
        self.data_dir = Path(data_dir)
        self.model_path = self.data_dir / 'models' / f'{person_name}_lbph.xml'
        
        if not self.model_path.exists():
            raise FileNotFoundError(f'Model not found: {self.model_path}')
        
        print(f'[Initializing] Real-time recognition test for: {person_name}')
        
        # Initialize camera
        self.pipeline = dai.Pipeline()
        self.cam = self.pipeline.createColorCamera()
        self.cam.setBoardSocket(dai.CameraBoardSocket.RGB)
        self.cam.setVideoSize(1920, 1080)
        self.cam.setPreviewSize(1920, 1080)
        self.cam.setFps(30)
        self.cam.setInterleaved(False)
        
        self.out = self.pipeline.createXLinkOut()
        self.out.setStreamName('rgb')
        self.cam.preview.link(self.out.input)
        
        self.device = dai.Device(self.pipeline)
        self.queue = self.device.getOutputQueue(name='rgb', maxSize=4, blocking=False)
        
        print('✓ OAK-D Lite camera initialized')
        
        # Load face detector
        cascade_path = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'
        if not os.path.exists(cascade_path):
            cascade_path = '/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml'
        
        self.face_detector = cv2.CascadeClassifier(cascade_path)
        if self.face_detector.empty():
            raise Exception('Failed to load cascade classifier')
        
        print('✓ Haar Cascade loaded')
        
        # Load recognizer
        self.recognizer = cv2.face.LBPHFaceRecognizer_create()
        self.recognizer.read(str(self.model_path))
        
        print(f'✓ LBPH model loaded: {self.model_path.name}')
        print()
    
    def close(self):
        """Clean up resources."""
        if self.device:
            self.device.close()
    
    def run(self, duration=30, confidence_threshold=50):
        """
        Run real-time recognition test.
        
        Args:
            duration: How long to run in seconds (default 30)
            confidence_threshold: Confidence score threshold (lower = stricter, default 50)
        """
        print('=' * 70)
        print('REAL-TIME FACE RECOGNITION TEST (HEADLESS)')
        print('=' * 70)
        print()
        print(f'Testing recognition for: {self.person_name.upper()}')
        print(f'Confidence threshold: {confidence_threshold} (lower = better match)')
        print(f'Duration: {duration} seconds')
        print()
        print('Continuous feedback:')
        print('-' * 70)
        print()
        
        frame_count = 0
        recognized_count = 0
        unrecognized_count = 0
        no_face_count = 0
        
        start_time = time.time()
        last_feedback_time = 0
        last_feedback = ""
        
        try:
            while time.time() - start_time < duration:
                # Get frame
                in_frame = self.queue.get()
                if in_frame is None:
                    continue
                
                frame = in_frame.getCvFrame()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                
                frame_count += 1
                
                # Detect faces
                faces = self.face_detector.detectMultiScale(gray, 1.3, 5)
                
                current_feedback = None
                
                if len(faces) == 0:
                    no_face_count += 1
                    current_feedback = "⌛ No face detected"
                else:
                    # Try to recognize each face
                    for (x, y, fw, fh) in faces:
                        face_roi = gray[y:y+fh, x:x+fw]
                        
                        # Predict
                        label, confidence = self.recognizer.predict(face_roi)
                        
                        # Check if it's the trained person (confidence < threshold)
                        if confidence < confidence_threshold:  # Better match
                            recognized_count += 1
                            current_feedback = f"✅ {self.person_name.upper()} RECOGNIZED! (confidence: {confidence:.0f})"
                        else:
                            unrecognized_count += 1
                            current_feedback = f"❌ Unknown person (confidence: {confidence:.0f})"
                
                # Print feedback every ~0.5 seconds
                current_time = time.time()
                if current_feedback and current_time - last_feedback_time > 0.5:
                    if current_feedback != last_feedback:
                        elapsed = current_time - start_time
                        print(f"[{elapsed:5.1f}s] {current_feedback}")
                        last_feedback = current_feedback
                        last_feedback_time = current_time
        
        except KeyboardInterrupt:
            print()
            print('-' * 70)
        
        finally:
            self.close()
        
        # Summary
        print()
        print('-' * 70)
        print()
        print('=' * 70)
        print('REAL-TIME TEST SUMMARY')
        print('=' * 70)
        print()
        print(f'Duration: {time.time() - start_time:.1f} seconds')
        print(f'Total frames processed: {frame_count}')
        print()
        print(f'✅ Recognized {self.person_name.upper()}: {recognized_count} times')
        print(f'❌ Unknown person detected: {unrecognized_count} times')
        print(f'⌛ No face detected: {no_face_count} times')
        print()
        
        if frame_count > 0:
            recognized_or_unknown = recognized_count + unrecognized_count
            
            if recognized_or_unknown > 0:
                recognition_rate = (recognized_count / recognized_or_unknown) * 100
                face_detection_rate = (recognized_or_unknown / frame_count) * 100
                
                print(f'Face detection rate: {face_detection_rate:.1f}%')
                print(f'Recognition accuracy: {recognition_rate:.1f}%')
                print()
                
                if recognition_rate > 80:
                    print("✅ EXCELLENT recognition! Ready for deployment!")
                elif recognition_rate > 50:
                    print("⚠️  MODERATE recognition. Consider capturing more images.")
                else:
                    print("❌ POOR recognition. Need more training images or better conditions.")
            else:
                print("⚠️  No faces detected during test.")
        
        print()


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 realtime_recognition_test_headless.py <person_name> <data_dir>")
        print()
        print("Example:")
        print("  python3 realtime_recognition_test_headless.py severin ~/dev/r2d2/data/face_recognition")
        sys.exit(1)
    
    person_name = sys.argv[1]
    data_dir = sys.argv[2]
    
    # Optional arguments
    duration = 30  # Default to 30 seconds
    confidence_threshold = 50
    
    if len(sys.argv) > 3:
        try:
            duration = int(sys.argv[3])
        except ValueError:
            duration = 30
    
    if len(sys.argv) > 4:
        try:
            confidence_threshold = int(sys.argv[4])
        except ValueError:
            confidence_threshold = 50
    
    try:
        test = RealtimeRecognitionTestHeadless(person_name, data_dir)
        test.run(duration=duration, confidence_threshold=confidence_threshold)
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)
