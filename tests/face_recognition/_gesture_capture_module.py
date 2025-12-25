#!/usr/bin/env python3
"""
Modular Gesture Training Data Capture Script (Person-Specific)

Purpose:
  Captures gesture training images for a specific person and gesture type.
  Called by train_manager.py with person_name and data directory.
  Uses MediaPipe Hands for gesture validation during capture.

Usage (via manager):
  python3 train_manager.py  (select "Train gestures for person")

Usage (direct):
  python3 _gesture_capture_module.py <person_name> <gesture_recognition_data_dir>

Author: R2D2 Perception Pipeline
Date: December 17, 2025
"""

import depthai as dai
import cv2
import os
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    import mediapipe as mp
except ImportError:
    print('\n❌ Error: MediaPipe not installed.')
    print('Install with: pip install mediapipe')
    sys.exit(1)


class GestureCaptureModule:
    """Captures training images for gesture recognition."""
    
    def __init__(self, person_name, data_dir):
        """
        Initialize gesture capture module.
        
        Args:
            person_name: Name of person being trained (target person)
            data_dir: Base gesture recognition data directory
        """
        self.person_name = person_name
        self.base_output_dir = Path(data_dir) / person_name
        self.base_output_dir.mkdir(parents=True, exist_ok=True)
        
        # Gesture classes to capture
        self.gestures = ['index_finger_up', 'fist', 'open_hand']
        
        self.frame_count = 0
        self.total_saved = 0
        
        # Initialize MediaPipe Hands
        print('\n[Initializing] MediaPipe Hands...')
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        print('✓ MediaPipe Hands initialized')
        
        # Initialize OAK-D camera
        print('[Initializing] Camera startup...')
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
        
        print('✓ OAK-D Lite camera initialized')
        print(f'✓ Output directory: {self.base_output_dir}')
    
    def validate_index_finger_up(self, hand_landmarks):
        """
        Check if gesture is index finger raised.
        
        Args:
            hand_landmarks: MediaPipe hand landmarks
            
        Returns:
            bool: True if index finger is raised, other fingers down
        """
        # Index finger tip (8) should be above MCP (5)
        # Other finger tips should be below their MCPs
        index_tip = hand_landmarks.landmark[8]
        index_mcp = hand_landmarks.landmark[5]
        
        middle_tip = hand_landmarks.landmark[12]
        middle_mcp = hand_landmarks.landmark[9]
        
        ring_tip = hand_landmarks.landmark[16]
        ring_mcp = hand_landmarks.landmark[13]
        
        pinky_tip = hand_landmarks.landmark[20]
        pinky_mcp = hand_landmarks.landmark[17]
        
        # Index up (tip y < mcp y, since y increases downward)
        index_up = index_tip.y < index_mcp.y - 0.05
        
        # Other fingers down (tip y > mcp y)
        middle_down = middle_tip.y > middle_mcp.y
        ring_down = ring_tip.y > ring_mcp.y
        pinky_down = pinky_tip.y > pinky_mcp.y
        
        return index_up and middle_down and ring_down and pinky_down
    
    def validate_fist(self, hand_landmarks):
        """
        Check if gesture is a fist (all fingers closed).
        
        Args:
            hand_landmarks: MediaPipe hand landmarks
            
        Returns:
            bool: True if fist detected
        """
        # All finger tips should be close to palm (below their MCPs)
        index_tip = hand_landmarks.landmark[8]
        index_mcp = hand_landmarks.landmark[5]
        
        middle_tip = hand_landmarks.landmark[12]
        middle_mcp = hand_landmarks.landmark[9]
        
        ring_tip = hand_landmarks.landmark[16]
        ring_mcp = hand_landmarks.landmark[13]
        
        pinky_tip = hand_landmarks.landmark[20]
        pinky_mcp = hand_landmarks.landmark[17]
        
        # All fingers should be curled (tip y > mcp y or very close)
        index_curled = index_tip.y >= index_mcp.y - 0.02
        middle_curled = middle_tip.y >= middle_mcp.y - 0.02
        ring_curled = ring_tip.y >= ring_mcp.y - 0.02
        pinky_curled = pinky_tip.y >= pinky_mcp.y - 0.02
        
        return index_curled and middle_curled and ring_curled and pinky_curled
    
    def validate_open_hand(self, hand_landmarks):
        """
        Check if gesture is an open hand (all fingers extended, palm visible).
        
        Args:
            hand_landmarks: MediaPipe hand landmarks
            
        Returns:
            bool: True if open hand detected (all 5 fingers extended)
        """
        # All finger tips should be above their MCPs (fingers extended)
        index_tip = hand_landmarks.landmark[8]
        index_mcp = hand_landmarks.landmark[5]
        
        middle_tip = hand_landmarks.landmark[12]
        middle_mcp = hand_landmarks.landmark[9]
        
        ring_tip = hand_landmarks.landmark[16]
        ring_mcp = hand_landmarks.landmark[13]
        
        pinky_tip = hand_landmarks.landmark[20]
        pinky_mcp = hand_landmarks.landmark[17]
        
        # Thumb landmarks
        thumb_tip = hand_landmarks.landmark[4]
        thumb_ip = hand_landmarks.landmark[3]
        
        # All 4 fingers extended (tip y < mcp y, since y increases downward)
        index_extended = index_tip.y < index_mcp.y - 0.03
        middle_extended = middle_tip.y < middle_mcp.y - 0.03
        ring_extended = ring_tip.y < ring_mcp.y - 0.03
        pinky_extended = pinky_tip.y < pinky_mcp.y - 0.03
        
        # Thumb extended (tip x further from palm than IP joint)
        # Works for both left and right hands by checking distance from wrist
        wrist = hand_landmarks.landmark[0]
        thumb_extended = abs(thumb_tip.x - wrist.x) > abs(thumb_ip.x - wrist.x)
        
        return index_extended and middle_extended and ring_extended and pinky_extended and thumb_extended
    
    def show_instruction(self, gesture_name, instruction_text):
        """Display clear instruction with visual separation."""
        print('\n' + '='*70)
        print(f'GESTURE: {gesture_name.replace("_", " ").upper()}')
        print('='*70)
        print(f'\n{instruction_text}\n')
        print('='*70)
        input('Press ENTER when ready to capture...')
        print('Capturing... (15 seconds)\n')
    
    def capture_gesture(self, gesture_name, instruction_text, duration=15):
        """
        Capture images for one gesture with MediaPipe validation.
        
        Args:
            gesture_name: 'index_finger_up' or 'fist'
            instruction_text: Clear instruction to user
            duration: Seconds to capture
        """
        self.show_instruction(gesture_name, instruction_text)
        
        # Create output directory for this gesture
        gesture_output_dir = self.base_output_dir / gesture_name
        gesture_output_dir.mkdir(parents=True, exist_ok=True)
        
        start_time = time.time()
        gesture_saved = 0
        gesture_frames = 0
        hands_detected = 0
        valid_gestures = 0
        
        print('Capturing...', end='', flush=True)
        
        while True:
            in_frame = self.queue.get()
            frame = in_frame.getCvFrame()
            gesture_frames += 1
            
            # Convert to RGB for MediaPipe
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Process with MediaPipe Hands
            results = self.hands.process(frame_rgb)
            
            if results.multi_hand_landmarks:
                hands_detected += 1
                
                for hand_landmarks in results.multi_hand_landmarks:
                    # Validate gesture
                    is_valid = False
                    if gesture_name == 'index_finger_up':
                        is_valid = self.validate_index_finger_up(hand_landmarks)
                    elif gesture_name == 'fist':
                        is_valid = self.validate_fist(hand_landmarks)
                    elif gesture_name == 'open_hand':
                        is_valid = self.validate_open_hand(hand_landmarks)
                    
                    if is_valid:
                        valid_gestures += 1
                        
                        # Save full frame with hand
                        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                        filename = f'{timestamp}_{gesture_name}_{gesture_saved:03d}.jpg'
                        filepath = gesture_output_dir / filename
                        
                        cv2.imwrite(str(filepath), frame)
                        gesture_saved += 1
                        self.total_saved += 1
                        
                        # Throttle saves (don't save every frame)
                        time.sleep(0.1)
            
            elapsed = time.time() - start_time
            
            # Progress indicator
            if int(elapsed) % 3 == 0 and int(elapsed) > 0:
                print('.', end='', flush=True)
            
            if elapsed > duration:
                break
        
        print(f' Done!\n')
        print(f'Gesture Results:')
        print(f'  ✓ Frames captured: {gesture_frames}')
        print(f'  ✓ Hands detected: {hands_detected}')
        print(f'  ✓ Valid gestures: {valid_gestures}')
        print(f'  ✓ Images saved: {gesture_saved}')
        print(f'  ✓ Total so far: {self.total_saved}')
    
    def run(self):
        """Run the complete gesture capture sequence."""
        print('\n' + '='*70)
        print(f'GESTURE TRAINING DATA CAPTURE FOR: {self.person_name.upper()}')
        print('='*70)
        print('\nYou will be guided through 3 gesture capture stages.')
        print('Each stage: 15 seconds of video capture.')
        print('Hand detection and gesture validation are automatic.')
        print('\nAt each stage, you\'ll be asked to press ENTER before starting.')
        print('This gives you time to position yourself correctly.\n')
        
        input('Press ENTER to begin first gesture...')
        
        try:
            # Gesture 1: Index finger up
            self.capture_gesture(
                'index_finger_up',
                'Hold your INDEX FINGER UP (pointing upward).\n'
                'Keep other fingers closed or relaxed.\n'
                'Hold hand steady in front of camera (arm\'s length).\n'
                'Slowly move hand: left/right, up/down for variation.\n'
                'Keep the gesture consistent - index finger always pointing up.'
            )
            
            # Gesture 2: Fist
            self.capture_gesture(
                'fist',
                'Make a FIST (all fingers closed).\n'
                'Close all fingers tightly into palm.\n'
                'Hold hand steady in front of camera (arm\'s length).\n'
                'Slowly move hand: left/right, up/down for variation.\n'
                'Keep fist closed throughout the capture.'
            )
            
            # Gesture 3: Open hand
            self.capture_gesture(
                'open_hand',
                'Show your OPEN HAND (palm facing camera, all fingers extended).\n'
                'Spread all 5 fingers apart - like a "stop" or "high five" gesture.\n'
                'Hold hand steady in front of camera (arm\'s length).\n'
                'Slowly move hand: left/right, up/down for variation.\n'
                'Keep all fingers extended and spread throughout the capture.'
            )
            
            # Summary
            print('\n' + '='*70)
            print('GESTURE CAPTURE COMPLETE')
            print('='*70)
            print(f'\nTotal images captured: {self.total_saved}')
            print(f'Output directory: {self.base_output_dir}')
            
            # Check each gesture
            for gesture in self.gestures:
                gesture_dir = self.base_output_dir / gesture
                count = len(list(gesture_dir.glob('*.jpg')))
                print(f'  • {gesture}: {count} images')
            
            print(f'\nRecommendation:')
            min_images = min([len(list((self.base_output_dir / g).glob('*.jpg'))) for g in self.gestures])
            
            if min_images < 20:
                print(f'  ⚠️  Low image count ({min_images} minimum).')
                print(f'  → Consider running capture again (add more images)')
                print(f'  → Better results with 30-40+ images per gesture')
            elif min_images < 30:
                print(f'  ✓ {min_images} images per gesture is adequate.')
                print(f'  → You can proceed to training')
                print(f'  → Consider adding more for even better accuracy')
            else:
                print(f'  ✓ {min_images} images per gesture is excellent!')
                print(f'  → Ready for training')
            
            print(f'\nNext step: Train gesture classifier from captured images.')
            print('='*70 + '\n')
            
            return True
            
        except KeyboardInterrupt:
            print('\n\n⚠️  Capture interrupted by user.')
            return False
        finally:
            self.hands.close()
            self.device.close()


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('Usage: python3 _gesture_capture_module.py <person_name> <gesture_recognition_data_dir>')
        sys.exit(1)
    
    try:
        person_name = sys.argv[1]
        data_dir = sys.argv[2]
        
        capture = GestureCaptureModule(person_name, data_dir)
        success = capture.run()
        sys.exit(0 if success else 1)
    except Exception as e:
        print(f'\n❌ Error: {e}')
        import traceback
        traceback.print_exc()
        sys.exit(1)


