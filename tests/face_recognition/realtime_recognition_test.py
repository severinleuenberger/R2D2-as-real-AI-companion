#!/usr/bin/env python3
"""
Real-Time Face Recognition Test
Continuously tries to recognize faces and provides instant feedback.

Usage:
  python3 realtime_recognition_test.py <person_name> <data_dir>

Example:
  python3 realtime_recognition_test.py severin ~/dev/r2d2/data/face_recognition
"""

import depthai as dai
import cv2
import os
import sys
from pathlib import Path
from datetime import datetime


class RealtimeRecognitionTest:
    """Real-time face recognition with continuous feedback."""
    
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
    
    def run(self):
        """Run real-time recognition test."""
        print('=' * 70)
        print('REAL-TIME FACE RECOGNITION TEST')
        print('=' * 70)
        print()
        print(f'Testing recognition for: {self.person_name.upper()}')
        print('Confidence threshold: 50 (lower = better match)')
        print()
        print('Press "q" to quit')
        print('Press "s" to save screenshot')
        print()
        print('-' * 70)
        print()
        
        frame_count = 0
        recognized_count = 0
        unrecognized_count = 0
        no_face_count = 0
        
        last_feedback = ""
        feedback_frames = 0
        
        try:
            while True:
                # Get frame
                in_frame = self.queue.get()
                if in_frame is None:
                    continue
                
                frame = in_frame.getCvFrame()
                h, w = frame.shape[:2]
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                
                frame_count += 1
                
                # Detect faces
                faces = self.face_detector.detectMultiScale(gray, 1.3, 5)
                
                current_feedback = None
                
                if len(faces) == 0:
                    no_face_count += 1
                    current_feedback = "⌛ No face detected"
                    feedback_color = (200, 200, 200)  # Gray
                else:
                    # Try to recognize each face
                    for (x, y, fw, fh) in faces:
                        face_roi = gray[y:y+fh, x:x+fw]
                        
                        # Predict
                        label, confidence = self.recognizer.predict(face_roi)
                        
                        # Check if it's the trained person (label 0 or confidence < 50)
                        if confidence < 50:  # Better match
                            recognized_count += 1
                            current_feedback = f"✅ {self.person_name.upper()} RECOGNIZED! (confidence: {confidence:.0f})"
                            feedback_color = (0, 255, 0)  # Green
                        else:
                            unrecognized_count += 1
                            current_feedback = f"❌ Unknown person (confidence: {confidence:.0f})"
                            feedback_color = (0, 0, 255)  # Red
                        
                        # Draw bounding box
                        cv2.rectangle(frame, (x, y), (x+fw, y+fh), feedback_color, 3)
                        
                        # Add label
                        cv2.putText(
                            frame,
                            current_feedback,
                            (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            feedback_color,
                            2
                        )
                
                # Display continuous feedback at top
                if current_feedback and feedback_frames < 30:  # Show for ~1 second
                    feedback_frames += 1
                    last_feedback = current_feedback
                else:
                    feedback_frames = 0
                    if last_feedback:
                        print(last_feedback)
                        last_feedback = current_feedback if current_feedback else ""
                
                # Display stats at bottom
                stats_text = f'Frame: {frame_count} | Recognized: {recognized_count} | Unknown: {unrecognized_count} | No face: {no_face_count}'
                cv2.putText(
                    frame,
                    stats_text,
                    (10, h - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2
                )
                
                # Show frame
                cv2.imshow('Real-Time Face Recognition', frame)
                
                # Handle keyboard
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print()
                    print('-' * 70)
                    break
                elif key == ord('s'):
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    filename = f'screenshot_{timestamp}.jpg'
                    cv2.imwrite(filename, frame)
                    print(f"✓ Screenshot saved: {filename}")
        
        except KeyboardInterrupt:
            print()
            print('-' * 70)
        
        finally:
            cv2.destroyAllWindows()
            self.close()
        
        # Summary
        print()
        print('=' * 70)
        print('REAL-TIME TEST SUMMARY')
        print('=' * 70)
        print()
        print(f'Total frames: {frame_count}')
        print(f'✅ Recognized {self.person_name.upper()}: {recognized_count} times')
        print(f'❌ Unknown person detected: {unrecognized_count} times')
        print(f'⌛ No face detected: {no_face_count} times')
        print()
        
        if frame_count > 0:
            recognition_rate = (recognized_count / (recognized_count + unrecognized_count + no_face_count)) * 100 if (recognized_count + unrecognized_count + no_face_count) > 0 else 0
            face_detection_rate = ((recognized_count + unrecognized_count) / frame_count) * 100
            
            print(f'Face detection rate: {face_detection_rate:.1f}%')
            print(f'Recognition rate: {recognition_rate:.1f}%')
            print()
            
            if recognition_rate > 80:
                print("✅ EXCELLENT recognition! Ready for deployment!")
            elif recognition_rate > 50:
                print("⚠️  MODERATE recognition. Consider capturing more images.")
            else:
                print("❌ POOR recognition. Need more training images or better conditions.")
        
        print()


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 realtime_recognition_test.py <person_name> <data_dir>")
        print()
        print("Example:")
        print("  python3 realtime_recognition_test.py severin ~/dev/r2d2/data/face_recognition")
        sys.exit(1)
    
    person_name = sys.argv[1]
    data_dir = sys.argv[2]
    
    try:
        test = RealtimeRecognitionTest(person_name, data_dir)
        test.run()
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)
