#!/usr/bin/env python3
"""
Interactive Face Recognition Training System

This script presents training tasks step-by-step with detailed instructions.
You confirm each task, the system captures images, and then moves to the next task.

Each training task varies:
- Lighting conditions (bright, medium, low)
- Distance from camera (1m, 2m, 3m)
- Head angle/direction (front, left, right, up, down)
- Expressions (neutral, smile, serious)

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


class InteractiveTraining:
    """Interactive step-by-step face training system."""
    
    # Training tasks (you can customize these)
    TRAINING_TASKS = [
        {
            "id": 1,
            "name": "Bright Light - Front View",
            "description": "Stand in BRIGHT sunlight or well-lit area",
            "instructions": [
                "☀️  BRIGHT LIGHTING (sunlight or strong lamp)",
                "",
                "📏 DISTANCE: 1 meter from camera",
                "🧑 POSITION: Face forward, center of frame",
                "",
                "HEAD MOVEMENTS:",
                "  • Start facing directly at camera",
                "  • Slow head turn: LEFT → CENTER → RIGHT",
                "  • Look slightly UP then DOWN",
                "  • Move head side-to-side 2-3 times",
                "",
                "EXPRESSIONS:",
                "  • Keep mostly neutral",
                "  • One natural smile is fine",
                "",
                "⏱️  Duration: 20 seconds total"
            ],
            "duration": 20
        },
        {
            "id": 2,
            "name": "Medium Light - 1.5m Distance",
            "description": "Step back to 1.5 meters, look left and right",
            "instructions": [
                "💡 MEDIUM LIGHTING (indoor room light, not direct sun)",
                "",
                "📏 DISTANCE: 1.5 meters from camera",
                "🧑 POSITION: Face forward, center of frame",
                "",
                "HEAD MOVEMENTS:",
                "  • Start facing camera",
                "  • Rotate head: 45° LEFT → CENTER → 45° RIGHT",
                "  • Repeat 3-4 times slowly",
                "  • Tilt head slightly forward/back",
                "",
                "EXPRESSIONS:",
                "  • Natural, neutral expression",
                "",
                "⏱️  Duration: 20 seconds total"
            ],
            "duration": 20
        },
        {
            "id": 3,
            "name": "Low Light - Front View",
            "description": "Step into low light area (no direct lighting)",
            "instructions": [
                "🌙 LOW LIGHTING (shadowed area, no direct light)",
                "",
                "📏 DISTANCE: 1.5 meters from camera",
                "🧑 POSITION: Face forward, well-centered",
                "",
                "HEAD MOVEMENTS:",
                "  • Slight head movements only",
                "  • Look slightly left, center, right",
                "  • Small up/down tilts",
                "",
                "EXPRESSIONS:",
                "  • Neutral, calm expression",
                "",
                "⏱️  Duration: 20 seconds total"
            ],
            "duration": 20
        },
        {
            "id": 4,
            "name": "Bright Light - Profile (45°)",
            "description": "Return to bright light, look 45° to the side",
            "instructions": [
                "☀️  BRIGHT LIGHTING (same as task 1)",
                "",
                "📏 DISTANCE: 1.5 meters from camera",
                "🧑 POSITION: Turn head 45° to the RIGHT",
                "",
                "HEAD MOVEMENTS:",
                "  • Turn right side to camera (45° angle)",
                "  • Hold for a moment",
                "  • Slowly turn to 45° LEFT",
                "  • Hold position",
                "  • Repeat 2-3 times",
                "",
                "EXPRESSIONS:",
                "  • Neutral expression",
                "",
                "⏱️  Duration: 20 seconds total"
            ],
            "duration": 20
        },
        {
            "id": 5,
            "name": "Medium Light - 2 Meters Distance",
            "description": "Step back to 2 meters, normal lighting",
            "instructions": [
                "💡 MEDIUM LIGHTING (indoor room light)",
                "",
                "📏 DISTANCE: 2 meters from camera",
                "🧑 POSITION: Face forward",
                "",
                "HEAD MOVEMENTS:",
                "  • Front view for first 5 seconds",
                "  • Turn slightly left/right (30° angles)",
                "  • Keep steady movements",
                "",
                "EXPRESSIONS:",
                "  • Neutral expression",
                "",
                "⏱️  Duration: 20 seconds total"
            ],
            "duration": 20
        },
        {
            "id": 6,
            "name": "Bright Light - Looking Up/Down",
            "description": "Bright light, various head angles (up/down)",
            "instructions": [
                "☀️  BRIGHT LIGHTING (strong light)",
                "",
                "📏 DISTANCE: 1.5 meters from camera",
                "🧑 POSITION: Face forward, then vary angle",
                "",
                "HEAD MOVEMENTS:",
                "  • Start facing camera normally",
                "  • Look UP (30° angle)",
                "  • Return to neutral",
                "  • Look DOWN (30° angle)",
                "  • Back to neutral",
                "  • Repeat 2-3 times",
                "",
                "EXPRESSIONS:",
                "  • Neutral throughout",
                "",
                "⏱️  Duration: 20 seconds total"
            ],
            "duration": 20
        },
        {
            "id": 7,
            "name": "Low Light - Profile Views",
            "description": "Low light area, left and right profiles",
            "instructions": [
                "🌙 LOW LIGHTING (shadowed, no direct light)",
                "",
                "📏 DISTANCE: 1.5 meters from camera",
                "🧑 POSITION: Profile view (side of face to camera)",
                "",
                "HEAD MOVEMENTS:",
                "  • Show RIGHT PROFILE for ~7 seconds",
                "  • Turn to front for ~2 seconds",
                "  • Show LEFT PROFILE for ~7 seconds",
                "  • Return to front",
                "  • Repeat once more",
                "",
                "EXPRESSIONS:",
                "  • Neutral, relax facial muscles",
                "",
                "⏱️  Duration: 20 seconds total"
            ],
            "duration": 20
        },
        {
            "id": 8,
            "name": "Medium Light - 3 Meters Distance (Final)",
            "description": "Farthest distance test with normal lighting",
            "instructions": [
                "💡 MEDIUM LIGHTING (indoor room light)",
                "",
                "📏 DISTANCE: 3 meters from camera (farthest)",
                "🧑 POSITION: Full body visible, face centered",
                "",
                "HEAD MOVEMENTS:",
                "  • Small, subtle head movements",
                "  • Keep face well-centered in frame",
                "  • Slight turns left/right",
                "  • Keep face looking toward camera",
                "",
                "EXPRESSIONS:",
                "  • Neutral expression",
                "",
                "⏱️  Duration: 20 seconds total"
            ],
            "duration": 20
        }
    ]
    
    def __init__(self, person_name, output_dir):
        """Initialize the interactive training system."""
        self.person_name = person_name
        self.output_dir = Path(output_dir) / person_name
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        self.current_task = 0
        self.total_images = 0
        
        print('\n' + '='*70)
        print('INITIALIZING INTERACTIVE FACE TRAINING SYSTEM')
        print('='*70)
        print(f'\nPerson: {person_name}')
        print(f'Output directory: {self.output_dir}')
        print(f'Total tasks: {len(self.TRAINING_TASKS)}')
        
        # Initialize camera
        print('\n[Initializing camera...]')
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
        
        # Load face cascade
        cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        if not os.path.exists(cascade_path):
            cascade_path = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'
        
        self.face_cascade = cv2.CascadeClassifier(cascade_path)
        if self.face_cascade.empty():
            raise RuntimeError('Failed to load Haar Cascade')
        
        print('✓ Camera initialized')
        print('✓ Face detection ready')
        print('\n' + '='*70)
    
    def show_task_header(self, task):
        """Display task header."""
        print('\n' + '='*70)
        print(f'TASK {task["id"]} of {len(self.TRAINING_TASKS)}')
        print(f'{task["name"].upper()}')
        print('='*70)
    
    def show_instructions(self, task):
        """Display detailed instructions."""
        print('\n📋 INSTRUCTIONS:\n')
        for line in task["instructions"]:
            print(f'  {line}')
        print()
    
    def wait_for_confirmation(self):
        """Wait for user to confirm they're ready."""
        print('='*70)
        print()
        response = input('Are you ready to START this task? Type "yes" to begin: ').strip().lower()
        print()
        
        if response == 'yes':
            return True
        else:
            print('⚠️  Skipping this task.')
            return False
    
    def capture_task(self, task):
        """Capture images for a task."""
        duration = task["duration"]
        stage_name = task["name"].lower().replace(' - ', '_').replace(' ', '_')
        
        print(f'Capturing... ({duration} seconds)')
        print('Indicator: . = 1 second of capture\n')
        print('Progress: ', end='', flush=True)
        
        start_time = time.time()
        task_images = 0
        faces_detected = 0
        frames_processed = 0
        
        while True:
            in_frame = self.queue.get()
            if in_frame is None:
                continue
            
            frame = in_frame.getCvFrame()
            frames_processed += 1
            
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
                    
                    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
                    filename = f'{timestamp}_{stage_name}_{task_images:03d}.jpg'
                    filepath = self.output_dir / filename
                    
                    cv2.imwrite(str(filepath), face_gray)
                    task_images += 1
                    self.total_images += 1
            
            elapsed = time.time() - start_time
            
            # Progress indicator (one dot per second)
            if int(elapsed) > 0 and int(elapsed) % 1 == 0:
                if int(elapsed) == int(time.time() - start_time):
                    print('.', end='', flush=True)
            
            if elapsed > duration:
                break
        
        print(' Done!\n')
        
        return {
            "task_id": task["id"],
            "task_name": task["name"],
            "frames": frames_processed,
            "faces_detected": faces_detected,
            "images_saved": task_images,
            "duration": duration
        }
    
    def show_results(self, result):
        """Show results of a single task."""
        print(f'📊 TASK {result["task_id"]} RESULTS:')
        print(f'  ✓ Frames processed: {result["frames"]}')
        print(f'  ✓ Faces detected: {result["faces_detected"]}')
        print(f'  ✓ Images saved: {result["images_saved"]}')
        print(f'  ✓ Total accumulated: {self.total_images}')
        print()
    
    def run(self):
        """Run the complete interactive training sequence."""
        print('\n' + '='*70)
        print('INTERACTIVE FACE RECOGNITION TRAINING')
        print('='*70)
        print(f'\nYou will complete {len(self.TRAINING_TASKS)} training tasks.')
        print('Each task has different lighting, distance, and angle.')
        print('\nFor each task:')
        print('  1. Read the instructions')
        print('  2. Prepare yourself in the specified position')
        print('  3. Type "yes" when ready')
        print('  4. System captures images automatically')
        print('  5. Move to next task')
        print('\nCancel anytime: Press Ctrl+C\n')
        
        input('Press ENTER to start the first task...')
        
        results = []
        skipped = []
        
        try:
            for task in self.TRAINING_TASKS:
                self.show_task_header(task)
                self.show_instructions(task)
                
                ready = self.wait_for_confirmation()
                
                if ready:
                    result = self.capture_task(task)
                    results.append(result)
                    self.show_results(result)
                else:
                    skipped.append(task["id"])
                    print(f'Task {task["id"]} skipped.')
                    print()
                
                # Ask to continue or stop
                if task["id"] < len(self.TRAINING_TASKS):
                    cont = input('Continue to next task? (yes/no): ').strip().lower()
                    if cont != 'yes':
                        print('\n⚠️  Stopping training.')
                        break
                    print()
            
            # Final summary
            self.show_summary(results, skipped)
            
            return True
        
        except KeyboardInterrupt:
            print('\n\n⚠️  Training interrupted by user.')
            print(f'Images captured so far: {self.total_images}')
            return False
        
        finally:
            self.device.close()
    
    def show_summary(self, results, skipped):
        """Show final summary of training."""
        print('\n' + '='*70)
        print('TRAINING SUMMARY')
        print('='*70)
        
        print(f'\nTasks completed: {len(results)} of {len(self.TRAINING_TASKS)}')
        
        if skipped:
            print(f'Tasks skipped: {len(skipped)} (Task IDs: {", ".join(map(str, skipped))})')
        
        print(f'\nTotal images captured: {self.total_images}')
        
        if results:
            print(f'\nDetailed results:')
            for result in results:
                print(f'  Task {result["task_id"]}: {result["images_saved"]} images')
        
        print('\n' + '='*70)
        print('RECOMMENDATIONS:')
        print('='*70)
        
        if self.total_images < 50:
            print(f'\n⚠️  You have {self.total_images} images.')
            print('   Target: 100+ images for best recognition quality')
            print('   Consider running again to capture more.')
        elif self.total_images < 100:
            print(f'\n✓ You have {self.total_images} images.')
            print('  This is a good start!')
            print('  Consider adding 20-30 more for better robustness.')
        else:
            print(f'\n✓✓ You have {self.total_images} images!')
            print('   Excellent dataset for face recognition training.')
            print('   Ready to proceed with model training.')
        
        print('\n' + '='*70)
        print('NEXT STEPS:')
        print('='*70)
        print('\n1. Train your model:')
        print(f'   python3 _train_module.py {self.person_name} ~/dev/r2d2/data/face_recognition')
        print('\n2. Test recognition:')
        print(f'   python3 _test_module.py {self.person_name} ~/dev/r2d2/data/face_recognition')
        print('\n' + '='*70 + '\n')


if __name__ == '__main__':
    try:
        person_name = 'severin'  # Default to your name, can be changed
        output_dir = Path.home() / 'dev' / 'r2d2' / 'data' / 'face_recognition'
        
        training = InteractiveTraining(person_name, output_dir)
        success = training.run()
        
        sys.exit(0 if success else 1)
    
    except Exception as e:
        print(f'\n❌ Error: {e}')
        import traceback
        traceback.print_exc()
        sys.exit(1)
