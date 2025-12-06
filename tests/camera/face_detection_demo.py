#!/usr/bin/env python3
"""
Face Detection Demo Script for R2D2

This standalone script demonstrates face detection on the OAK-D Lite camera
using OpenCV Haar Cascade classifier. It captures frames directly from the
DepthAI SDK (not via ROS 2), performs face detection, and saves annotated
frames for visual inspection.

Purpose:
- Validate that face detection works in the current environment
- Test Haar Cascade parameters on actual R2D2 hardware
- Provide baseline performance metrics (detection rate, processing time)
- Generate sample output for documentation

Key Features:
- Grabs frames from OAK-D Lite via DepthAI SDK
- Detects faces using OpenCV Haar Cascade (haarcascade_frontalface_default.xml)
- Draws red bounding boxes around detected faces
- Logs detection results to console
- Saves first and best detection frames as annotated JPEGs
- Measures processing time per frame

How It Works:
1. Create DepthAI pipeline with ColorCamera node
2. Connect to OAK-D device and start capturing
3. For each frame:
   - Convert from ROS Image format to OpenCV BGR
   - Run Haar Cascade detection on the frame
   - Count detected faces and extract bounding boxes
   - Draw rectangles around faces
   - Save annotated frame if conditions met
   - Log results

Expected Output:
- Console logs with face count and processing time
- Annotated JPEG files: face_detection_result_001.jpg, face_detection_result_002.jpg, etc.
- Performance metrics: Average FPS, detection rate
"""

import cv2
import depthai as dai
import time
import numpy as np
from pathlib import Path
from datetime import datetime


class FaceDetectionDemo:
    """Standalone face detection demo using OAK-D and OpenCV Haar Cascade."""

    def __init__(self, output_dir: str = None):
        """
        Initialize face detection demo.

        Args:
            output_dir (str): Directory to save annotated frames. 
                             Defaults to ~/dev/r2d2/tests/camera/
        """
        # Output directory setup
        if output_dir is None:
            output_dir = "/home/severin/dev/r2d2/tests/camera"
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Load Haar Cascade classifier
        cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
        self.face_cascade = cv2.CascadeClassifier(cascade_path)
        
        if self.face_cascade.empty():
            raise RuntimeError(
                f"Failed to load Haar Cascade from {cascade_path}. "
                "Check OpenCV installation."
            )

        print(f"✓ Haar Cascade loaded: {cascade_path}")

        # Cascade detection parameters (tunable)
        self.scale_factor = 1.05  # How much image size reduced at each scale
        self.min_neighbors = 5    # How many neighbors each candidate rectangle needs
        self.min_size = (30, 30)  # Minimum face size to detect
        self.max_size = (500, 500)  # Maximum face size to detect

        # Statistics tracking
        self.frame_count = 0
        self.total_faces_detected = 0
        self.frames_with_faces = 0
        self.frame_times = []

        # Output file naming
        self.output_index = 1

        print(f"✓ Output directory: {self.output_dir}")
        print(
            f"✓ Cascade parameters: scale={self.scale_factor}, "
            f"neighbors={self.min_neighbors}, minSize={self.min_size}"
        )

    def initialize_camera(self) -> dai.Device:
        """
        Initialize OAK-D Lite camera via DepthAI SDK.

        Returns:
            dai.Device: Initialized camera device

        Raises:
            RuntimeError: If device initialization fails
        """
        print("\n[Camera] Initializing OAK-D Lite...")

        # Create pipeline
        pipeline = dai.Pipeline()

        # Define source
        cam = pipeline.create(dai.node.ColorCamera)
        cam.setBoardSocket(dai.CameraBoardSocket.RGB)
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setFps(30)

        # Define stream output
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam.preview.link(xout_rgb.input)

        # Connect to device
        try:
            device = dai.Device(pipeline)
            print("✓ OAK-D Lite connected successfully")
            print(f"✓ Device: {device.getDeviceName()}")
            print(f"✓ MX ID: {device.getMxId()}")
            return device
        except RuntimeError as e:
            raise RuntimeError(f"Failed to connect to OAK-D: {str(e)}")

    def detect_faces(self, frame: np.ndarray) -> list:
        """
        Detect faces in a frame using Haar Cascade.

        Args:
            frame (np.ndarray): Input frame (BGR format, H×W×3)

        Returns:
            list: List of (x, y, width, height) tuples for detected faces
        """
        # Convert to grayscale for detection (more efficient)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect faces
        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=self.scale_factor,
            minNeighbors=self.min_neighbors,
            minSize=self.min_size,
            maxSize=self.max_size,
            flags=cv2.CASCADE_SCALE_IMAGE,
        )

        return faces

    def draw_detections(
        self, frame: np.ndarray, faces: list, color: tuple = (0, 0, 255)
    ) -> np.ndarray:
        """
        Draw bounding boxes around detected faces.

        Args:
            frame (np.ndarray): Input frame
            faces (list): List of (x, y, w, h) tuples
            color (tuple): BGR color for rectangles (default: red)

        Returns:
            np.ndarray: Annotated frame with rectangles drawn
        """
        annotated = frame.copy()

        for i, (x, y, w, h) in enumerate(faces):
            # Draw rectangle
            cv2.rectangle(annotated, (x, y), (x + w, y + h), color, 2)

            # Draw face number and size info
            label = f"Face {i+1} ({w}x{h})"
            cv2.putText(
                annotated,
                label,
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2,
            )

        # Draw overall statistics in top-left corner
        stats_text = f"Faces: {len(faces)} | Frame: {self.frame_count}"
        cv2.putText(
            annotated,
            stats_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

        return annotated

    def save_frame(self, frame: np.ndarray, label: str = "") -> str:
        """
        Save annotated frame to disk.

        Args:
            frame (np.ndarray): Frame to save
            label (str): Optional label for the filename

        Returns:
            str: Path to saved file
        """
        filename = f"face_detection_result_{self.output_index:03d}.jpg"
        if label:
            filename = f"face_detection_result_{label}_{self.output_index:03d}.jpg"

        filepath = self.output_dir / filename
        self.output_index += 1

        success = cv2.imwrite(str(filepath), frame)

        if success:
            file_size_kb = filepath.stat().st_size / 1024
            print(f"  ✓ Saved: {filename} ({file_size_kb:.1f} KB)")
            return str(filepath)
        else:
            print(f"  ✗ Failed to save: {filename}")
            return None

    def log_frame_info(self, frame_num: int, faces: list, proc_time_ms: float):
        """
        Log frame processing information.

        Args:
            frame_num (int): Frame number
            faces (list): Detected faces
            proc_time_ms (float): Processing time in milliseconds
        """
        face_count = len(faces)
        face_info = (
            f"{face_count} face{'s' if face_count != 1 else ''}"
            if face_count > 0
            else "No faces"
        )

        avg_fps = (
            len(self.frame_times) / sum(self.frame_times)
            if self.frame_times
            else 0
        )

        print(
            f"Frame #{frame_num:4d} | {face_info:12s} | "
            f"Proc time: {proc_time_ms:6.2f}ms | Avg FPS: {avg_fps:5.1f}"
        )

    def run(self, duration_seconds: int = 20):
        """
        Run face detection demo for specified duration.

        Args:
            duration_seconds (int): How long to run (default: 20 seconds)
        """
        print(f"\n[Demo] Starting face detection for {duration_seconds} seconds...")
        print("Press Ctrl+C to stop early\n")

        try:
            device = self.initialize_camera()
        except RuntimeError as e:
            print(f"✗ Camera initialization failed: {e}")
            return

        # Get queue
        q_rgb = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)

        start_time = time.time()

        try:
            while True:
                # Check timeout
                elapsed = time.time() - start_time
                if elapsed > duration_seconds:
                    break

                # Get frame
                in_rgb = q_rgb.get()
                if in_rgb is None:
                    continue

                # Convert to OpenCV format
                frame = in_rgb.getCvFrame()
                self.frame_count += 1

                # Detect faces
                frame_start = time.time()
                faces = self.detect_faces(frame)
                proc_time_ms = (time.time() - frame_start) * 1000

                self.frame_times.append(proc_time_ms / 1000)
                self.total_faces_detected += len(faces)
                if len(faces) > 0:
                    self.frames_with_faces += 1

                # Draw and log
                annotated_frame = self.draw_detections(frame, faces)
                self.log_frame_info(self.frame_count, faces, proc_time_ms)

                # Save first frame and any frame with faces
                if self.frame_count == 1:
                    self.save_frame(annotated_frame, label="first")

                if len(faces) > 0 and self.frames_with_faces <= 3:
                    # Save up to 3 frames with detected faces
                    self.save_frame(annotated_frame, label=f"detected")

        except KeyboardInterrupt:
            print("\n[Demo] Interrupted by user")
        finally:
            device.close()
            self.print_summary()

    def print_summary(self):
        """Print final statistics and summary."""
        print("\n" + "=" * 70)
        print("FACE DETECTION DEMO SUMMARY")
        print("=" * 70)
        print(f"Total frames captured:     {self.frame_count}")
        print(f"Total faces detected:      {self.total_faces_detected}")
        print(f"Frames with ≥1 face:       {self.frames_with_faces}")

        if self.frame_count > 0:
            detection_rate = (self.frames_with_faces / self.frame_count) * 100
            print(f"Detection rate:            {detection_rate:.1f}%")

        if self.frame_times:
            avg_time = np.mean(self.frame_times) * 1000
            std_time = np.std(self.frame_times) * 1000
            avg_fps = 1.0 / np.mean(self.frame_times)
            print(f"Avg processing time:       {avg_time:.2f}ms (±{std_time:.2f}ms)")
            print(f"Avg FPS:                   {avg_fps:.1f}")

        print(f"Output directory:          {self.output_dir}")
        print(
            f"Saved frames:              {self.output_index - 1} "
            f"(index resets on restart)"
        )
        print("=" * 70)


def main():
    """Main entry point."""
    print("\n" + "=" * 70)
    print("R2D2 FACE DETECTION DEMO")
    print("OpenCV Haar Cascade on OAK-D Lite")
    print("=" * 70)

    try:
        demo = FaceDetectionDemo()
        demo.run(duration_seconds=20)
    except Exception as e:
        print(f"\n✗ Error: {str(e)}")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    main()
