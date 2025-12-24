#!/usr/bin/env python3
"""
Improved Face Training Data Capture Script for R2D2

Purpose:
  Captures high-quality, diverse face images for LBPH training.
  Implements quality filtering, deduplication, and optimal image selection.

Key Improvements:
  - Quality assessment (blur, lighting, face size)
  - Deduplication (avoids redundant images)
  - Limited images per stage (10-15 max, total 40-50)
  - Minimum interval between saves (0.5s)
  - Real-time quality feedback

Usage:
  source ~/depthai_env/bin/activate
  export OPENBLAS_CORETYPE=ARMV8
  python3 1_capture_training_data_improved.py

Output:
  Saves 40-50 high-quality images to ~/dev/r2d2/data/face_recognition/severin/

Author: R2D2 Perception Pipeline
Date: December 9, 2025
"""

import depthai as dai
import cv2
import os
import time
import numpy as np
from datetime import datetime

# Try to import SSIM for better deduplication, fallback to histogram if not available
try:
    from skimage.metrics import structural_similarity as ssim
    HAS_SSIM = True
except ImportError:
    HAS_SSIM = False

# Use non-GUI backend
cv2.setNumThreads(2)  # ARM optimization


class ImprovedTrainingDataCapture:
    """Captures high-quality, diverse face training images with quality filtering."""
    
    def __init__(self):
        """Initialize camera pipeline and face cascade."""
        self.output_dir = os.path.expanduser('~/dev/r2d2/data/face_recognition/severin')
        self.frame_count = 0
        self.total_saved = 0
        self.recent_images = []  # For deduplication
        self.last_save_time = 0
        self.min_save_interval = 0.5  # Minimum 0.5 seconds between saves
        
        # Quality thresholds
        self.min_quality_score = 0.6  # Minimum quality score (0.0-1.0)
        self.max_similarity = 0.85  # Maximum similarity for deduplication (0.0-1.0)
        self.min_laplacian_var = 50.0  # Minimum blur threshold
        
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
        print(f'✓ Quality filtering enabled (min score: {self.min_quality_score})')
        print(f'✓ Deduplication enabled (max similarity: {self.max_similarity})')
        if HAS_SSIM:
            print(f'✓ SSIM available for advanced deduplication')
        else:
            print(f'⚠ Using histogram comparison for deduplication (install scikit-image for better results)')
    
    def assess_quality(self, face_image):
        """
        Assess quality of a face image.
        
        Args:
            face_image: Grayscale face image (100x100)
            
        Returns:
            tuple: (quality_score, blur_score, lighting_score, details)
        """
        # 1. Blur detection using Laplacian variance
        laplacian_var = cv2.Laplacian(face_image, cv2.CV_64F).var()
        blur_score = min(laplacian_var / 100.0, 1.0)  # Normalize to 0-1
        
        # 2. Lighting assessment (histogram analysis)
        mean_brightness = np.mean(face_image)
        # Ideal brightness is around 128 (middle of 0-255)
        brightness_diff = abs(mean_brightness - 128) / 128.0
        lighting_score = max(0.0, 1.0 - brightness_diff)
        
        # 3. Contrast assessment (standard deviation)
        contrast = np.std(face_image) / 128.0  # Normalize
        contrast_score = min(contrast, 1.0)
        
        # 4. Combined quality score (weighted)
        quality_score = (
            blur_score * 0.5 +      # Blur is most important
            lighting_score * 0.3 +   # Lighting is important
            contrast_score * 0.2     # Contrast helps
        )
        
        details = {
            'blur': blur_score,
            'lighting': lighting_score,
            'contrast': contrast_score,
            'laplacian_var': laplacian_var
        }
        
        return quality_score, blur_score, lighting_score, details
    
    def is_duplicate(self, new_image, threshold=None):
        """
        Check if new image is too similar to recently saved images.
        
        Args:
            new_image: Grayscale face image (100x100)
            threshold: Similarity threshold (default: self.max_similarity)
            
        Returns:
            bool: True if duplicate, False otherwise
        """
        if threshold is None:
            threshold = self.max_similarity
        
        if len(self.recent_images) == 0:
            return False
        
        # Compare with last 10 saved images
        for recent_image in self.recent_images[-10:]:
            if HAS_SSIM:
                # Use SSIM (Structural Similarity Index) - more accurate
                try:
                    similarity = ssim(new_image, recent_image)
                    if similarity > threshold:
                        return True
                except Exception:
                    pass
            
            # Fallback to histogram comparison
            hist1 = cv2.calcHist([new_image], [0], None, [256], [0, 256])
            hist2 = cv2.calcHist([recent_image], [0], None, [256], [0, 256])
            correlation = cv2.compareHist(hist1, hist2, cv2.HISTCMP_CORREL)
            if correlation > threshold:
                return True
        
        return False
    
    def should_save_image(self, face_image, current_time):
        """
        Determine if image should be saved based on quality and timing.
        
        Args:
            face_image: Grayscale face image (100x100)
            current_time: Current timestamp
            
        Returns:
            tuple: (should_save, reason, quality_score)
        """
        # Check minimum interval
        if current_time - self.last_save_time < self.min_save_interval:
            return False, "interval", 0.0
        
        # Assess quality
        quality_score, blur_score, lighting_score, details = self.assess_quality(face_image)
        
        # Check quality threshold
        if quality_score < self.min_quality_score:
            return False, "quality", quality_score
        
        # Check blur threshold
        if details['laplacian_var'] < self.min_laplacian_var:
            return False, "blur", quality_score
        
        # Check for duplicates
        if self.is_duplicate(face_image):
            return False, "duplicate", quality_score
        
        return True, "ok", quality_score
    
    def capture_stage(self, stage_name, description, duration_seconds=10, max_images=15):
        """
        Capture high-quality images for one stage.
        
        Args:
            stage_name: Identifier (e.g., "bright_direct")
            description: User-facing instruction text
            duration_seconds: How long to capture
            max_images: Maximum images to save in this stage
        """
        print(f'\n' + '='*60)
        print(f'STAGE: {stage_name.upper()}')
        print('='*60)
        print(f'{description}')
        print(f'Capturing for {duration_seconds} seconds...')
        print(f'Maximum {max_images} high-quality images will be saved.')
        print('='*60 + '\n')
        
        start_time = time.time()
        stage_saved = 0
        frame_count = 0
        quality_rejected = 0
        duplicate_rejected = 0
        interval_rejected = 0
        
        while True:
            # Get frame from camera
            in_frame = self.queue.get()
            frame = in_frame.getCvFrame()
            frame_count += 1
            current_time = time.time()
            elapsed = current_time - start_time
            
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
                # Check if we've reached max images for this stage
                if stage_saved >= max_images:
                    break
                
                # Extract face ROI and resize to 100x100 (standard size for LBPH)
                face_roi = frame[y:y+h, x:x+w]
                face_resized = cv2.resize(face_roi, (100, 100))
                face_gray = cv2.cvtColor(face_resized, cv2.COLOR_BGR2GRAY)
                
                # Check if we should save this image
                should_save, reason, quality_score = self.should_save_image(face_gray, current_time)
                
                if should_save:
                    # Save image
                    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                    filename = f'{timestamp}_{stage_name}_{stage_saved:03d}.jpg'
                    filepath = os.path.join(self.output_dir, filename)
                    
                    cv2.imwrite(filepath, face_gray)
                    
                    # Update tracking
                    stage_saved += 1
                    self.total_saved += 1
                    self.last_save_time = current_time
                    self.recent_images.append(face_gray.copy())
                    
                    # Keep only last 20 images for deduplication
                    if len(self.recent_images) > 20:
                        self.recent_images.pop(0)
                    
                    print(f'  ✓ Saved image {stage_saved}/{max_images} (quality: {quality_score:.2f})')
                else:
                    # Track rejection reasons
                    if reason == "quality":
                        quality_rejected += 1
                    elif reason == "duplicate":
                        duplicate_rejected += 1
                    elif reason == "interval":
                        interval_rejected += 1
            
            # Log progress every 2 seconds
            if frame_count % 60 == 0:  # ~2 seconds at 30 fps
                print(f'  [{elapsed:.1f}s] Saved: {stage_saved}/{max_images} | '
                      f'Rejected: Q={quality_rejected} D={duplicate_rejected} I={interval_rejected}')
            
            # Check for timeout
            if elapsed > duration_seconds:
                break
        
        print(f'\n✓ Stage complete: Saved {stage_saved} high-quality images')
        print(f'  Rejected: {quality_rejected} (quality), {duplicate_rejected} (duplicate), '
              f'{interval_rejected} (interval)')
        return True
    
    def run(self):
        """Run the complete training data capture sequence."""
        print('\n' + '='*60)
        print('R2D2 IMPROVED FACE TRAINING DATA CAPTURE')
        print('='*60)
        print('This script captures HIGH-QUALITY, DIVERSE training images.')
        print('Quality filtering ensures only the best images are saved.')
        print('Expected result: 40-50 high-quality images (not 800+!)')
        print('\nPress Ctrl+C to stop at any time.')
        print('='*60)
        
        input('\nPress ENTER to begin...')
        
        try:
            # Stage 1: Frontal, bright light (10 images)
            self.capture_stage(
                'bright_direct',
                'STAGE 1/5: Bright Direct Light\n'
                'Stand 1 meter from camera, look straight ahead.\n'
                'Move slowly left/right and nod up/down to vary angles.',
                duration_seconds=10,
                max_images=10
            )
            
            # Stage 2: Frontal, low light (10 images)
            self.capture_stage(
                'dim_indoor',
                'STAGE 2/5: Dim Indoor Light\n'
                'Move to a dimly lit area (e.g., corner without direct light).\n'
                'Stand 1 meter from camera, look straight ahead.\n'
                'Move left/right and nod to vary angles.',
                duration_seconds=10,
                max_images=10
            )
            
            # Stage 3: Side angle (10 images)
            self.capture_stage(
                'side_45deg',
                'STAGE 3/5: Side Profile (45 degrees)\n'
                'Return to bright area, stand at 45-degree angle.\n'
                'Move slowly left/right while maintaining 45-degree angle.\n'
                'Nod up/down to vary vertical angle.',
                duration_seconds=10,
                max_images=10
            )
            
            # Stage 4: Varied distances (10 images)
            self.capture_stage(
                'varied_distance',
                'STAGE 4/5: Varied Distance (1m, 2m, 3m)\n'
                'Stand 1 meter from camera for 3 seconds, then move back.\n'
                'Move to 2 meters away for 3 seconds.\n'
                'Move to 3 meters away for the rest of the stage.\n'
                'Look straight ahead and move side-to-side at each distance.',
                duration_seconds=15,
                max_images=10
            )
            
            # Stage 5: Expressions (5-10 images)
            self.capture_stage(
                'expressions',
                'STAGE 5/5: Facial Expressions\n'
                'Stand 1 meter from camera, front-facing.\n'
                'Show neutral expression, then smile, then slight variations.\n'
                'Keep face clear and well-lit.',
                duration_seconds=5,
                max_images=10
            )
            
            # Summary
            print('\n' + '='*60)
            print('TRAINING DATA CAPTURE COMPLETE')
            print('='*60)
            print(f'Total high-quality images captured: {self.total_saved}')
            print(f'Expected range: 40-50 images (optimal for LBPH)')
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
        capture = ImprovedTrainingDataCapture()
        capture.run()
    except Exception as e:
        print(f'\n❌ Error: {e}')
        import traceback
        traceback.print_exc()
        exit(1)

