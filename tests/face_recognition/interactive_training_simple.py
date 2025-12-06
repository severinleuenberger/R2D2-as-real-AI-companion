#!/usr/bin/env python3
"""
Simplified Interactive Face Training - 4 Tasks Only

Tasks:
1. Bright Light - 1m (head: right, left, up, down)
2. Bright Light - 2m (head: right, left, up, down)
3. Low Light - 3m (head: right, left, up, down)
4. Low Light - 5m (head: right, left, up, down)

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


class SimpleInteractiveTraining:
    """Simplified interactive face training system."""
    
    TRAINING_TASKS = [
        {
            "id": 1,
            "name": "Bright Light - 1 Meter",
            "lighting": "bright",
            "distance": 1,
            "instructions": [
                "☀️  BRIGHT LIGHTING (sunlight or strong lamp)",
                "📏 DISTANCE: 1 meter from camera",
                "",
                "HEAD MOVEMENTS (during 20 seconds):",
                "  • Look RIGHT (5 sec)",
                "  • Look LEFT (5 sec)",
                "  • Look UP (5 sec)",
                "  • Look DOWN (5 sec)",
                "",
                "⏱️  Duration: 20 seconds total"
            ],
            "duration": 20
        },
        {
            "id": 2,
            "name": "Bright Light - 2 Meters",
            "lighting": "bright",
            "distance": 2,
            "instructions": [
                "☀️  BRIGHT LIGHTING (sunlight or strong lamp)",
                "📏 DISTANCE: 2 meters from camera",
                "",
                "HEAD MOVEMENTS (during 20 seconds):",
                "  • Look RIGHT (5 sec)",
                "  • Look LEFT (5 sec)",
                "  • Look UP (5 sec)",
                "  • Look DOWN (5 sec)",
                "",
                "⏱️  Duration: 20 seconds total"
            ],
            "duration": 20
        },
        {
            "id": 3,
            "name": "Low Light - 3 Meters",
            "lighting": "low",
            "distance": 3,
            "instructions": [
                "🌙 LOW LIGHTING (shadowed area, no direct light)",
                "📏 DISTANCE: 3 meters from camera",
                "",
                "HEAD MOVEMENTS (during 20 seconds):",
                "  • Look RIGHT (5 sec)",
                "  • Look LEFT (5 sec)",
                "  • Look UP (5 sec)",
                "  • Look DOWN (5 sec)",
                "",
                "⏱️  Duration: 20 seconds total"
            ],
            "duration": 20
        },
        {
            "id": 4,
            "name": "Low Light - 5 Meters",
            "lighting": "low",
            "distance": 5,
            "instructions": [
                "🌙 LOW LIGHTING (shadowed area, no direct light)",
                "📏 DISTANCE: 5 meters from camera",
                "",
                "HEAD MOVEMENTS (during 20 seconds):",
                "  • Look RIGHT (5 sec)",
                "  • Look LEFT (5 sec)",
                "  • Look UP (5 sec)",
                "  • Look DOWN (5 sec)",
                "",
                "⏱️  Duration: 20 seconds total"
            ],
            "duration": 20
        }
    ]

    def __init__(self, person_name, output_dir):
        self.person_name = person_name
        self.output_dir = Path(output_dir) / person_name
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        self.pipeline = None
        self.device = None
        self.cam = None
        self.face_detector = None
        self.total_images = 0
        
    def setup_camera(self):
        """Initialize camera pipeline."""
        print("[Initializing camera...]")
        
        self.pipeline = dai.Pipeline()
        
        # Camera node
        self.cam = self.pipeline.createColorCamera()
        self.cam.setPreviewSize(1920, 1080)
        self.cam.setVideoSize(1920, 1080)
        self.cam.setFps(30)
        self.cam.setInterleaved(False)
        
        # Output queue
        self.cam_out = self.pipeline.createXLinkOut()
        self.cam_out.setStreamName("preview")
        self.cam.preview.link(self.cam_out.input)
        
        # Start device
        self.device = dai.Device(self.pipeline)
        self.queue = self.device.getOutputQueue(name="preview", maxSize=4, blocking=False)
        
        print("✓ Camera initialized")
        
        # Load face detector
        cascade_path = "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"
        if not os.path.exists(cascade_path):
            cascade_path = "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml"
        
        self.face_detector = cv2.CascadeClassifier(cascade_path)
        if self.face_detector.empty():
            raise Exception("Failed to load cascade classifier")
        
        print("✓ Face detection ready")
        
    def close(self):
        """Clean up resources."""
        if self.device:
            self.device.close()
    
    def capture_task(self, task):
        """Capture images for a single task."""
        print()
        print("=" * 73)
        print(f"TASK {task['id']} of {len(self.TRAINING_TASKS)}")
        print(task['name'].upper())
        print("=" * 73)
        print()
        print("📋 INSTRUCTIONS:")
        print()
        for line in task['instructions']:
            print(f"  {line}")
        print()
        
        # Wait for user confirmation
        response = input("Are you ready to START this task? Type 'yes' to begin: ").strip().lower()
        if response != 'yes':
            print("⏭️  Skipping this task.")
            return False
        
        print()
        print(f"Capturing... ({task['duration']} seconds)")
        print("Indicator: . = 1 second of capture")
        print()
        print("Progress: ", end="", flush=True)
        
        start_time = time.time()
        frames_processed = 0
        faces_detected = 0
        images_saved = 0
        task_start_total = self.total_images
        
        while True:
            elapsed = time.time() - start_time
            if elapsed >= task['duration']:
                break
            
            # Get frame
            in_frame = self.queue.get()
            if in_frame is None:
                continue
            
            frame = in_frame.getCvFrame()
            frames_processed += 1
            
            # Detect faces
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_detector.detectMultiScale(gray, 1.3, 5)
            
            if len(faces) > 0:
                faces_detected += 1
                
                # Save each detected face
                for (x, y, w, h) in faces:
                    # Extract face region
                    face_img = gray[y:y+h, x:x+w]
                    
                    # Resize to 100x100
                    face_resized = cv2.resize(face_img, (100, 100))
                    
                    # Save with timestamp
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    filename = f"{timestamp}_{task['lighting']}_dist{task['distance']}_{images_saved}.jpg"
                    filepath = self.output_dir / filename
                    
                    cv2.imwrite(str(filepath), face_resized)
                    images_saved += 1
                    self.total_images += 1
            
            # Progress indicator (every second)
            if frames_processed % 30 == 0:
                print(".", end="", flush=True)
        
        print(" Done!")
        print()
        print("📊 TASK RESULTS:")
        print(f"  ✓ Frames processed: {frames_processed}")
        print(f"  ✓ Faces detected: {faces_detected}")
        print(f"  ✓ Images saved: {images_saved}")
        print(f"  ✓ Total accumulated: {self.total_images}")
        print()
        
        return True
    
    def run(self):
        """Run the full training sequence."""
        print()
        print("=" * 73)
        print("SIMPLIFIED INTERACTIVE FACE TRAINING SYSTEM")
        print("=" * 73)
        print()
        print(f"Person: {self.person_name}")
        print(f"Output directory: {self.output_dir}")
        print(f"Total tasks: {len(self.TRAINING_TASKS)}")
        print()
        
        self.setup_camera()
        print()
        
        print("=" * 73)
        print("TRAINING OVERVIEW")
        print("=" * 73)
        print()
        print("You will complete 4 training tasks:")
        print("  Task 1: Bright Light - 1 meter")
        print("  Task 2: Bright Light - 2 meters")
        print("  Task 3: Low Light - 3 meters")
        print("  Task 4: Low Light - 5 meters")
        print()
        print("Each task: 20 seconds of capturing while moving your head")
        print("(RIGHT → LEFT → UP → DOWN)")
        print()
        print("Cancel anytime: Press Ctrl+C")
        print()
        
        input("Press ENTER to start the first task...")
        print()
        
        completed = 0
        try:
            for task in self.TRAINING_TASKS:
                if self.capture_task(task):
                    completed += 1
                
                # Ask to continue
                if task['id'] < len(self.TRAINING_TASKS):
                    response = input("Continue to next task? (yes/no): ").strip().lower()
                    if response != 'yes':
                        break
        
        except KeyboardInterrupt:
            print()
            print()
            print("⚠️  Training interrupted by user.")
            print(f"Images captured so far: {self.total_images}")
            print()
        
        finally:
            self.close()
        
        # Summary
        print()
        print("=" * 73)
        print("TRAINING SUMMARY")
        print("=" * 73)
        print()
        print(f"Tasks completed: {completed}/{len(self.TRAINING_TASKS)}")
        print(f"Total images captured: {self.total_images}")
        print()
        
        if self.total_images > 0:
            print("✅ Training data ready!")
            print()
            print("Next steps:")
            print("  1. From the menu, select [3] Train model")
            print("  2. Then select [4] Test model")
            print()
        else:
            print("⚠️  No images captured. Try again!")
            print()


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 interactive_training_simple.py <person_name> <output_dir>")
        sys.exit(1)
    
    person_name = sys.argv[1]
    output_dir = sys.argv[2]
    
    trainer = SimpleInteractiveTraining(person_name, output_dir)
    trainer.run()
