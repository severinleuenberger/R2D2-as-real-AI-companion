#!/usr/bin/env python3
"""
Find the optimal confidence threshold for recognition.
Tests different thresholds to find the best one.
"""

import depthai as dai
import cv2
import os
import sys
from pathlib import Path
from collections import defaultdict

def find_optimal_threshold(person_name, data_dir, test_duration=30):
    """Test different confidence thresholds and find the best one."""
    
    data_dir = Path(data_dir)
    model_path = data_dir / 'models' / f'{person_name}_lbph.xml'
    
    if not model_path.exists():
        print(f"❌ Model not found: {model_path}")
        return
    
    print('=' * 70)
    print('FINDING OPTIMAL CONFIDENCE THRESHOLD')
    print('=' * 70)
    print()
    print(f'Testing with person: {person_name.upper()}')
    print(f'Duration: {test_duration} seconds per threshold')
    print()
    
    # Initialize camera
    pipeline = dai.Pipeline()
    cam = pipeline.createColorCamera()
    cam.setBoardSocket(dai.CameraBoardSocket.RGB)
    cam.setVideoSize(1920, 1080)
    cam.setPreviewSize(1920, 1080)
    cam.setFps(30)
    cam.setInterleaved(False)
    
    out = pipeline.createXLinkOut()
    out.setStreamName('rgb')
    cam.preview.link(out.input)
    
    device = dai.Device(pipeline)
    queue = device.getOutputQueue(name='rgb', maxSize=4, blocking=False)
    
    # Load face detector
    cascade_path = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'
    if not os.path.exists(cascade_path):
        cascade_path = '/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml'
    
    face_detector = cv2.CascadeClassifier(cascade_path)
    if face_detector.empty():
        device.close()
        raise Exception('Failed to load cascade classifier')
    
    # Load recognizer
    recognizer = cv2.face.LBPHFaceRecognizer_create()
    recognizer.read(str(model_path))
    
    print('[Camera initialized]')
    print('[Model loaded]')
    print()
    
    # Test thresholds: 40, 50, 60, 70, 75, 80, 85, 90, 100
    thresholds = [40, 50, 60, 70, 75, 80, 85, 90, 100]
    results = {}
    
    import time
    
    for threshold in thresholds:
        print(f'Testing threshold: {threshold}')
        print('  Collecting data... ', end='', flush=True)
        
        recognized = 0
        unrecognized = 0
        no_face = 0
        confidence_scores = []
        
        start_time = time.time()
        frame_count = 0
        
        while time.time() - start_time < test_duration:
            in_frame = queue.get()
            if in_frame is None:
                continue
            
            frame = in_frame.getCvFrame()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame_count += 1
            
            faces = face_detector.detectMultiScale(gray, 1.3, 5)
            
            if len(faces) == 0:
                no_face += 1
            else:
                for (x, y, fw, fh) in faces:
                    face_roi = gray[y:y+fh, x:x+fw]
                    label, confidence = recognizer.predict(face_roi)
                    confidence_scores.append(confidence)
                    
                    if confidence < threshold:
                        recognized += 1
                    else:
                        unrecognized += 1
        
        face_detections = recognized + unrecognized
        
        if face_detections > 0:
            recognition_rate = (recognized / face_detections) * 100
            avg_confidence = sum(confidence_scores) / len(confidence_scores) if confidence_scores else 0
        else:
            recognition_rate = 0
            avg_confidence = 0
        
        results[threshold] = {
            'recognized': recognized,
            'unrecognized': unrecognized,
            'no_face': no_face,
            'recognition_rate': recognition_rate,
            'avg_confidence': avg_confidence,
            'frames': frame_count
        }
        
        print(f'Done! ({frame_count} frames)')
        print(f'    Recognition: {recognized}/{face_detections} ({recognition_rate:.1f}%)')
        print()
    
    device.close()
    
    # Summary
    print()
    print('=' * 70)
    print('THRESHOLD TEST RESULTS')
    print('=' * 70)
    print()
    print(f'{"Threshold":>10} {"Recognized":>12} {"Rate":>8} {"Avg Conf":>10}')
    print('-' * 70)
    
    best_threshold = None
    best_rate = -1
    
    for threshold in sorted(results.keys()):
        r = results[threshold]
        print(f'{threshold:>10} {r["recognized"]:>3}/{r["recognized"]+r["unrecognized"]:<8} {r["recognition_rate"]:>6.1f}% {r["avg_confidence"]:>10.1f}')
        
        # Find best (highest recognition rate)
        if r['recognition_rate'] > best_rate:
            best_rate = r['recognition_rate']
            best_threshold = threshold
    
    print()
    print('=' * 70)
    print('RECOMMENDATION')
    print('=' * 70)
    print()
    
    if best_rate > 80:
        print(f'✅ EXCELLENT! Use threshold: {best_threshold}')
        print(f'   Recognition rate: {best_rate:.1f}%')
        print()
        print('This threshold should work well for real-world use!')
    elif best_rate > 50:
        print(f'⚠️  MODERATE. Best threshold: {best_threshold}')
        print(f'   Recognition rate: {best_rate:.1f}%')
        print()
        print('Consider improving with more training images.')
    else:
        print(f'❌ POOR across all thresholds')
        print()
        print('You need more training images or better conditions.')
    
    print()
    print(f'Use this command for real-time testing with optimal threshold:')
    print(f'  python3 realtime_recognition_test_headless.py {person_name} {data_dir} 30 {best_threshold}')
    print()

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 find_optimal_threshold.py <person_name> <data_dir>")
        print()
        print("Example:")
        print("  python3 find_optimal_threshold.py severin ~/dev/r2d2/data/face_recognition")
        sys.exit(1)
    
    person_name = sys.argv[1]
    data_dir = sys.argv[2]
    
    try:
        find_optimal_threshold(person_name, data_dir)
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
