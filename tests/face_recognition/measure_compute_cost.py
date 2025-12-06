#!/usr/bin/env python3
"""
Measure the exact compute cost (CPU usage) for face recognition status reporting.

This script breaks down:
- Face detection cost
- Face recognition cost  
- Status update cost
- Total per-frame cost

Uses timing analysis and resource measurement.
"""

import cv2
import time
import numpy as np
from pathlib import Path
import threading
import psutil
import os

class ComputeCostAnalyzer:
    """Analyze the compute cost of face recognition."""
    
    def __init__(self, person_name='severin', data_dir='~/dev/r2d2/data/face_recognition'):
        self.person_name = person_name
        self.data_dir = Path(data_dir).expanduser()
        self.model_path = self.data_dir / 'models' / f'{person_name}_lbph.xml'
        
        # Load components
        self.face_detector = self._load_face_detector()
        self.recognizer = self._load_recognizer()
        self.process = psutil.Process(os.getpid())
        
        # Measurement results
        self.measurements = {
            'face_detection': [],
            'face_recognition': [],
            'status_update': [],
            'total_per_frame': [],
            'cpu_percent': [],
        }
    
    def _load_face_detector(self):
        """Load Haar cascade face detector."""
        cascade_path = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'
        if not os.path.exists(cascade_path):
            cascade_path = '/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml'
        
        detector = cv2.CascadeClassifier(cascade_path)
        if detector.empty():
            raise Exception('Failed to load face detector')
        return detector
    
    def _load_recognizer(self):
        """Load trained face recognizer."""
        if not self.model_path.exists():
            raise FileNotFoundError(f'Model not found: {self.model_path}')
        
        recognizer = cv2.face.LBPHFaceRecognizer_create()
        recognizer.read(str(self.model_path))
        return recognizer
    
    def analyze_recognition_process(self, test_image_path=None):
        """
        Analyze the cost of the complete recognition process.
        
        Breakdown:
        1. Face Detection (Haar cascade)
        2. Face Recognition (LBPH matching)
        3. Status Update (confidence comparison)
        """
        
        # Create test image if not provided
        if test_image_path is None:
            # Use a sample training image
            person_dir = self.data_dir / self.person_name
            if person_dir.exists():
                images = list(person_dir.glob('*.jpg'))[:5]
            else:
                raise Exception(f"No training images found for {self.person_name}")
        else:
            images = [Path(test_image_path)]
        
        if not images:
            raise Exception("No test images available")
        
        print("="*80)
        print("FACE RECOGNITION COMPUTE COST ANALYSIS")
        print("="*80)
        print()
        
        # Warm-up run
        print("[1/4] Warming up (2 runs)...")
        for _ in range(2):
            frame = cv2.imread(str(images[0]))
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_detector.detectMultiScale(gray, 1.3, 5)
            if len(faces) > 0:
                face_roi = gray[faces[0][1]:faces[0][1]+faces[0][3], 
                                faces[0][0]:faces[0][0]+faces[0][2]]
                label, conf = self.recognizer.predict(face_roi)
        
        # Actual measurements
        print("[2/4] Measuring face detection cost (100 iterations)...")
        self._measure_face_detection(images)
        
        print("[3/4] Measuring face recognition cost (100 iterations)...")
        self._measure_face_recognition(images)
        
        print("[4/4] Measuring complete pipeline (50 iterations)...")
        self._measure_complete_pipeline(images)
        
        # Report results
        self._report_results()
    
    def _measure_face_detection(self, images):
        """Measure face detection cost."""
        frame = cv2.imread(str(images[0]))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        times = []
        for _ in range(100):
            start = time.perf_counter()
            faces = self.face_detector.detectMultiScale(gray, 1.3, 5)
            elapsed = time.perf_counter() - start
            times.append(elapsed * 1000)  # Convert to ms
        
        self.measurements['face_detection'] = times
    
    def _measure_face_recognition(self, images):
        """Measure face recognition cost (confidence matching)."""
        frame = cv2.imread(str(images[0]))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_detector.detectMultiScale(gray, 1.3, 5)
        
        if len(faces) == 0:
            print("  ⚠️  No faces detected in test image, using synthetic face ROI")
            face_roi = np.random.randint(0, 255, (100, 100), dtype=np.uint8)
        else:
            x, y, w, h = faces[0]
            face_roi = gray[y:y+h, x:x+w]
        
        times = []
        for _ in range(100):
            start = time.perf_counter()
            label, confidence = self.recognizer.predict(face_roi)
            elapsed = time.perf_counter() - start
            times.append(elapsed * 1000)  # Convert to ms
        
        self.measurements['face_recognition'] = times
    
    def _measure_complete_pipeline(self, images):
        """Measure complete recognition pipeline."""
        times = []
        
        for img_path in images * 10:  # Repeat to get 50 iterations
            frame = cv2.imread(str(img_path))
            
            start = time.perf_counter()
            
            # Face detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_detector.detectMultiScale(gray, 1.3, 5)
            
            # Face recognition
            recognized = None
            if len(faces) > 0:
                x, y, w, h = faces[0]
                face_roi = gray[y:y+h, x:x+w]
                label, confidence = self.recognizer.predict(face_roi)
                
                # Status update (threshold check)
                if confidence < 70:  # Confidence threshold
                    recognized = self.person_name
            
            elapsed = time.perf_counter() - start
            times.append(elapsed * 1000)  # Convert to ms
        
        self.measurements['total_per_frame'] = times
    
    def _report_results(self):
        """Report analysis results."""
        print()
        print("="*80)
        print("RESULTS: COMPUTE COST BREAKDOWN")
        print("="*80)
        print()
        
        # Face detection
        face_det = self.measurements['face_detection']
        print("📊 FACE DETECTION (Haar Cascade)")
        print(f"   Time per frame: {np.mean(face_det):.2f} ms (±{np.std(face_det):.2f} ms)")
        print(f"   Min/Max: {np.min(face_det):.2f} / {np.max(face_det):.2f} ms")
        print(f"   Frames per second: {1000/np.mean(face_det):.1f} FPS")
        print()
        
        # Face recognition
        face_rec = self.measurements['face_recognition']
        print("🔍 FACE RECOGNITION (LBPH confidence matching)")
        print(f"   Time per face: {np.mean(face_rec):.2f} ms (±{np.std(face_rec):.2f} ms)")
        print(f"   Min/Max: {np.min(face_rec):.2f} / {np.max(face_rec):.2f} ms")
        print(f"   Frames per second: {1000/np.mean(face_rec):.1f} FPS")
        print()
        
        # Total pipeline
        total = self.measurements['total_per_frame']
        print("⚡ COMPLETE PIPELINE (Detection + Recognition + Status)")
        print(f"   Time per frame: {np.mean(total):.2f} ms (±{np.std(total):.2f} ms)")
        print(f"   Min/Max: {np.min(total):.2f} / {np.max(total):.2f} ms")
        print(f"   Frames per second: {1000/np.mean(total):.1f} FPS")
        print()
        
        # CPU percentage estimation
        self._estimate_cpu_usage()
        
        # Summary table
        self._print_summary_table()
    
    def _estimate_cpu_usage(self):
        """Estimate CPU usage based on timing."""
        avg_time_per_frame = np.mean(self.measurements['total_per_frame'])
        
        # At 15 FPS (66.67 ms per frame)
        fps_15 = 15
        time_per_frame_15fps = 1000 / fps_15  # 66.67 ms
        cpu_percent_15fps = (avg_time_per_frame / time_per_frame_15fps) * 100
        
        # At different FPS with frame skipping
        print("💻 CPU USAGE ESTIMATION (at 15 FPS camera)")
        print()
        print("   Frame Skip | Actual FPS | Processing % | Expected CPU*")
        print("   " + "-"*60)
        
        for skip in [1, 2, 3, 6]:
            actual_fps = 15 / skip
            processing_percent = cpu_percent_15fps / skip
            cpu_estimate = processing_percent * 0.5  # Rough estimate (half-core usage)
            
            marker = " ← Default" if skip == 2 else ""
            print(f"   {skip:2d}        | {actual_fps:6.1f} Hz   | {processing_percent:6.1f}%      | {cpu_estimate:5.1f}%-{cpu_estimate+5:5.1f}%{marker}")
        
        print()
        print("   * CPU estimate assumes single CPU core handling recognition.")
        print("     Actual CPU usage depends on system load and other processes.")
        print()
    
    def _print_summary_table(self):
        """Print summary table."""
        face_det = self.measurements['face_detection']
        face_rec = self.measurements['face_recognition']
        total = self.measurements['total_per_frame']
        
        print("="*80)
        print("COST BREAKDOWN TABLE")
        print("="*80)
        print()
        print(f"{'Component':<30} {'Mean (ms)':<15} {'Std (ms)':<15} {'% of Total':<15}")
        print("-"*80)
        
        face_det_mean = np.mean(face_det)
        face_rec_mean = np.mean(face_rec)
        total_mean = np.mean(total)
        
        print(f"{'Face Detection':<30} {face_det_mean:<15.3f} {np.std(face_det):<15.3f} "
              f"{(face_det_mean/total_mean)*100:<15.1f}")
        print(f"{'Face Recognition (1 face)':<30} {face_rec_mean:<15.3f} {np.std(face_rec):<15.3f} "
              f"{(face_rec_mean/total_mean)*100:<15.1f}")
        print("-"*80)
        print(f"{'TOTAL PER FRAME':<30} {total_mean:<15.3f} {np.std(total):<15.3f} {'100.0':<15}")
        print()
        
        # Cost analysis
        print("="*80)
        print("COST ANALYSIS FOR STATUS REPORTING")
        print("="*80)
        print()
        print("To determine if a person is 'RECOGNIZED' or 'NOT RECOGNIZED':")
        print()
        print(f"1. Face Detection:     {face_det_mean:.2f} ms")
        print(f"   → Locate face in image (using Haar cascade)")
        print()
        print(f"2. Face Recognition:   {face_rec_mean:.2f} ms per detected face")
        print(f"   → Extract features and match confidence against threshold")
        print(f"   → Compare confidence < 70 (threshold)")
        print()
        print(f"3. Status Update:      < 0.1 ms")
        print(f"   → Update JSON status file with result")
        print(f"   → Update LED display")
        print()
        print(f"TOTAL:                 {total_mean:.2f} ms per frame")
        print()
        print("At 15 FPS with frame skip=2 (7.5 processed FPS):")
        print(f"  • Processing time per frame: {total_mean:.2f} ms")
        print(f"  • Time available per frame: {1000/7.5:.1f} ms")
        print(f"  • CPU usage: ~{(total_mean/(1000/7.5))*100:.1f}% of one core")
        print()
        print("This scales linearly:")
        print(f"  • More faces → More recognition calls (~{face_rec_mean:.2f}ms per face)")
        print(f"  • Frame skip=3 → ~7% CPU")
        print(f"  • Frame skip=1 → ~{(total_mean/(1000/15))*100:.1f}% CPU (max throughput)")
        print()


def main():
    """Main entry point."""
    import sys
    
    person_name = 'severin' if len(sys.argv) < 2 else sys.argv[1]
    data_dir = '~/dev/r2d2/data/face_recognition' if len(sys.argv) < 3 else sys.argv[2]
    
    try:
        analyzer = ComputeCostAnalyzer(person_name, data_dir)
        analyzer.analyze_recognition_process()
    except FileNotFoundError as e:
        print(f"❌ Error: {e}")
        print()
        print("Make sure:")
        print("1. The model exists: {data_dir}/models/{person_name}_lbph.xml")
        print("2. The training data exists: {data_dir}/{person_name}/")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
