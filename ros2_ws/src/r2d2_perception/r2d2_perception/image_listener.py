#!/usr/bin/env python3
"""
Image listener and perception node for R2D2 perception pipeline.
Subscribes to OAK-D RGB camera topic, performs image processing, and publishes perception metrics.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32, String, Bool
from cv_bridge import CvBridge
import cv2
import time
import os
import numpy as np
from r2d2_common.person_config import PersonConfig


class ImageListener(Node):
    """
    ROS2 perception node that processes camera frames and publishes perception metrics.
    
    Functionality:
    - Subscribes to /oak/rgb/image_raw (sensor_msgs/msg/Image)
    - Counts incoming frames and tracks FPS
    - Extracts image dimensions (width, height)
    - Performs image processing:
      * Downscales image to 640x360
      * Converts to grayscale
      * Computes mean brightness
    - Detects faces using OpenCV Haar Cascade
    - Recognizes personal faces using LBPH (if enabled and model available)
    - Publishes brightness on /r2d2/perception/brightness (std_msgs/Float32)
    - Publishes face count on /r2d2/perception/face_count (std_msgs/Int32)
    - Publishes person ID on /r2d2/perception/person_id (std_msgs/String, if recognition enabled)
    - Publishes person confidence on /r2d2/perception/face_confidence (std_msgs/Float32, if recognition enabled)
    - Saves debug frames (RGB and grayscale) on demand
    """
    
    def __init__(self):
        """Initialize the perception node with subscriptions, publishers, and parameters."""
        super().__init__('image_listener')
        
        # Declare ROS 2 parameters
        self.declare_parameter('debug_frame_path', '/home/severin/dev/r2d2/tests/camera/perception_debug.jpg')
        self.declare_parameter('save_debug_gray_frame', False)
        self.declare_parameter('debug_gray_frame_path', '/home/severin/dev/r2d2/tests/camera/perception_debug_gray.jpg')
        self.declare_parameter('log_every_n_frames', 30)  # Log brightness every N frames
        self.declare_parameter('log_face_detections', False)  # Enable verbose face detection logging
        
        # Face recognition parameters (LBPH)
        self.declare_parameter('enable_face_recognition', False)  # Enable personal face recognition
        self.declare_parameter('face_recognition_model_path', 'auto')  # Auto-resolve from PersonRegistry, or provide explicit path
        self.declare_parameter('recognition_confidence_threshold', 150.0)  # Confidence threshold for target person (lower is better, set high to accept training variations)
        self.declare_parameter('recognition_frame_skip', 2)  # Process every Nth frame to manage CPU load
        self.declare_parameter('target_person_name', 'target_person')  # Name of the person to recognize (auto-resolved from PersonRegistry)
        self.declare_parameter('face_presence_threshold', 0.3)  # Seconds face must be detected before entering "stable presence" state (fast response)
        self.declare_parameter('face_absence_threshold', 5.0)  # Seconds face must be absent before entering "stable absence" state
        
        # Gesture recognition parameters
        self.declare_parameter('enable_gesture_recognition', False)  # Enable hand gesture recognition
        self.declare_parameter('gesture_model_path', 'auto')  # Auto-resolve from PersonRegistry, or provide explicit path
        self.declare_parameter('gesture_frame_skip', 3)  # Process every Nth frame to manage CPU load
        self.declare_parameter('gesture_confidence_threshold', 0.7)  # Minimum confidence for gesture recognition
        self.declare_parameter('target_person_gesture_name', 'target_person')  # Person whose gestures to recognize (auto-resolved from PersonRegistry)
        
        # Get parameter values
        self.debug_frame_path = self.get_parameter('debug_frame_path').value
        self.save_debug_gray = self.get_parameter('save_debug_gray_frame').value
        self.debug_gray_frame_path = self.get_parameter('debug_gray_frame_path').value
        self.log_every_n = self.get_parameter('log_every_n_frames').value
        self.log_faces = self.get_parameter('log_face_detections').value
        
        # Face recognition parameters
        self.enable_recognition = self.get_parameter('enable_face_recognition').value
        self.recognition_model_path = self.get_parameter('face_recognition_model_path').value
        self.recognition_threshold = self.get_parameter('recognition_confidence_threshold').value
        self.recognition_frame_skip = self.get_parameter('recognition_frame_skip').value
        self.target_person_name = self.get_parameter('target_person_name').value
        self.face_presence_threshold = self.get_parameter('face_presence_threshold').value
        self.face_absence_threshold = self.get_parameter('face_absence_threshold').value
        
        # Gesture recognition parameters
        self.enable_gesture_recognition = self.get_parameter('enable_gesture_recognition').value
        self.gesture_model_path = self.get_parameter('gesture_model_path').value
        self.gesture_frame_skip = self.get_parameter('gesture_frame_skip').value
        self.gesture_confidence_threshold = self.get_parameter('gesture_confidence_threshold').value
        self.target_person_gesture_name = self.get_parameter('target_person_gesture_name').value
        
        # Dynamic resolution using PersonConfig
        # Resolve person names from PersonRegistry if set to 'target_person'
        if self.target_person_name == 'target_person':
            resolved_name = PersonConfig.get_person_name('target_person')
            if resolved_name and resolved_name != 'target_person':
                self.get_logger().info(f"Target person name auto-resolved from PersonRegistry: {resolved_name}")
                self.target_person_name = resolved_name
        
        if self.target_person_gesture_name == 'target_person':
            resolved_gesture_name = PersonConfig.get_person_name('target_person')
            if resolved_gesture_name and resolved_gesture_name != 'target_person':
                self.get_logger().info(f"Gesture person name auto-resolved from PersonRegistry: {resolved_gesture_name}")
                self.target_person_gesture_name = resolved_gesture_name
        
        # Resolve model paths from PersonRegistry if set to 'auto' or contain 'target_person'
        if self.enable_recognition and ('auto' in self.recognition_model_path.lower() or 'target_person' in self.recognition_model_path):
            resolved_face_path = PersonConfig.get_face_model_path(self.target_person_name)
            if resolved_face_path:
                self.get_logger().info(f"Face model path auto-resolved from PersonRegistry: {resolved_face_path}")
                self.recognition_model_path = resolved_face_path
            else:
                self.get_logger().warn(f"Could not auto-resolve face model path for '{self.target_person_name}', using provided path")
        
        if self.enable_gesture_recognition and ('auto' in self.gesture_model_path.lower() or 'target_person' in self.gesture_model_path):
            resolved_gesture_path = PersonConfig.get_gesture_model_path(self.target_person_gesture_name)
            if resolved_gesture_path:
                self.get_logger().info(f"Gesture model path auto-resolved from PersonRegistry: {resolved_gesture_path}")
                self.gesture_model_path = resolved_gesture_path
            else:
                self.get_logger().warn(f"Could not auto-resolve gesture model path for '{self.target_person_gesture_name}', using provided path")
        
        # Create subscription to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/oak/rgb/image_raw',
            self.image_callback,
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        # Create publisher for brightness metric
        self.brightness_publisher = self.create_publisher(
            Float32,
            '/r2d2/perception/brightness',
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        # Create publisher for face count metric
        self.face_count_publisher = self.create_publisher(
            Int32,
            '/r2d2/perception/face_count',
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        # Create publishers for face recognition (if enabled)
        self.person_id_publisher = self.create_publisher(
            String,
            '/r2d2/perception/person_id',
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        self.face_confidence_publisher = self.create_publisher(
            Float32,
            '/r2d2/perception/face_confidence',
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        self.is_person_publisher = self.create_publisher(
            Bool,
            '/r2d2/perception/is_target_person',
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        # Create publisher for gesture events (if enabled)
        self.gesture_event_publisher = self.create_publisher(
            String,
            '/r2d2/perception/gesture_event',
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        # Initialize frame tracking
        self.frame_count = 0
        self.last_log_time = time.time()
        self.fps_frame_count = 0
        
        # Debug frame flags - save only once for each
        self.debug_rgb_saved = False
        self.debug_gray_saved = False
        
        # CV bridge for ROS Image ↔ OpenCV conversion
        self.bridge = CvBridge()
        
        # Load face cascade classifier for face detection
        # Try multiple paths for compatibility with different OpenCV installations
        cascade_paths = [
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'  if hasattr(cv2, 'data') else '',
            '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml',
            '/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml',
        ]
        
        self.face_cascade = None
        cascade_found = False
        for cascade_path in cascade_paths:
            if not cascade_path or not os.path.exists(cascade_path):
                continue
            try:
                self.face_cascade = cv2.CascadeClassifier(cascade_path)
                if not self.face_cascade.empty():
                    cascade_found = True
                    self.get_logger().info(f'Haar Cascade loaded successfully from {cascade_path}')
                    break
            except Exception:
                continue
        
        if not cascade_found or self.face_cascade is None or self.face_cascade.empty():
            self.face_cascade = None
            self.get_logger().warn('Failed to load Haar Cascade. Face detection will be disabled.')
        
        # Face detection parameters
        self.cascade_scale_factor = 1.05
        self.cascade_min_neighbors = 5
        self.cascade_min_size = (30, 30)
        self.cascade_max_size = (500, 500)
        
        # Load LBPH face recognizer if enabled
        self.face_recognizer = None
        self.recognition_enabled = False
        if self.enable_recognition:
            if os.path.exists(self.recognition_model_path):
                try:
                    self.face_recognizer = cv2.face.LBPHFaceRecognizer_create()
                    self.face_recognizer.read(self.recognition_model_path)
                    self.recognition_enabled = True
                    self.get_logger().info(f'LBPH face recognizer loaded from {self.recognition_model_path}')
                except Exception as e:
                    self.get_logger().warn(f'Failed to load LBPH model: {e}. Recognition disabled.')
                    self.recognition_enabled = False
            else:
                self.get_logger().warn(f'LBPH model not found at {self.recognition_model_path}. Recognition disabled.')
        
        # Counter for recognition frame skip (process every Nth frame)
        self.recognition_frame_counter = 0
        
        # Hysteresis state machine for stable face detection
        self.face_stable_state = False  # True = "stable presence", False = "stable absence"
        self.face_transition_start_time = None  # When current transition started
        self.last_raw_face_count = 0  # Last raw detection from Haar Cascade
        
        # Initialize gesture recognition (MediaPipe Hands + SVM classifier)
        self.gesture_recognizer_enabled = False
        self.mp_hands = None
        self.hands = None
        self.gesture_classifier = None
        self.gesture_scaler = None
        self.label_to_gesture = None
        self.last_gesture = None
        self.gesture_frame_counter = 0
        self.last_person_id = None
        
        if self.enable_gesture_recognition:
            try:
                import mediapipe as mp
                import pickle
                
                # Initialize MediaPipe Hands
                self.mp_hands = mp.solutions.hands
                self.hands = self.mp_hands.Hands(
                    static_image_mode=False,
                    max_num_hands=1,
                    min_detection_confidence=0.7,
                    min_tracking_confidence=0.5
                )
                
                # Load gesture classifier model
                if os.path.exists(self.gesture_model_path):
                    with open(self.gesture_model_path, 'rb') as f:
                        model_data = pickle.load(f)
                    
                    self.gesture_classifier = model_data['classifier']
                    self.gesture_scaler = model_data['scaler']
                    self.label_to_gesture = model_data['label_to_gesture']
                    self.gesture_recognizer_enabled = True
                    
                    self.get_logger().info(f'Gesture recognizer loaded from {self.gesture_model_path}')
                    self.get_logger().info(f'Gestures: {", ".join(model_data["gestures"])}')
                else:
                    self.get_logger().warn(f'Gesture model not found at {self.gesture_model_path}. Gesture recognition disabled.')
            
            except ImportError as e:
                self.get_logger().warn(f'MediaPipe not available: {e}. Gesture recognition disabled.')
                self.get_logger().warn('Install with: pip install mediapipe')
            except Exception as e:
                self.get_logger().warn(f'Failed to initialize gesture recognition: {e}')
        
        self.get_logger().info('ImageListener node initialized, subscribed to /oak/rgb/image_raw')
        if self.recognition_enabled:
            self.get_logger().info(f'Face recognition enabled (threshold={self.recognition_threshold}, frame_skip={self.recognition_frame_skip})')
        else:
            self.get_logger().info('Face detection enabled with Haar Cascade classifier')
        
        if self.gesture_recognizer_enabled:
            self.get_logger().info(f'Gesture recognition enabled (threshold={self.gesture_confidence_threshold}, frame_skip={self.gesture_frame_skip})')
    
    def image_callback(self, msg: Image):
        """
        Callback function triggered when a new frame arrives.
        Processes the image and publishes perception metrics.
        
        Args:
            msg (Image): ROS2 Image message from camera
        """
        # Increment frame counter
        self.frame_count += 1
        self.fps_frame_count += 1
        
        # Extract original image dimensions from message metadata
        original_width = msg.width
        original_height = msg.height
        
        # Convert ROS Image message to OpenCV BGR format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
        
        # Downscale image to 640x360 for faster processing
        downscaled = cv2.resize(cv_image, (640, 360))
        
        # Convert downscaled image to grayscale
        gray_image = cv2.cvtColor(downscaled, cv2.COLOR_BGR2GRAY)
        
        # Compute mean brightness from grayscale image (0-255 range)
        mean_brightness = float(np.mean(gray_image))
        
        # Publish brightness value on perception topic
        brightness_msg = Float32()
        brightness_msg.data = mean_brightness
        self.brightness_publisher.publish(brightness_msg)
        
        # Detect faces using Haar Cascade on grayscale image
        face_count = 0
        faces = []
        if self.face_cascade is not None and not self.face_cascade.empty():
            try:
                faces = self.face_cascade.detectMultiScale(
                    gray_image,
                    scaleFactor=self.cascade_scale_factor,
                    minNeighbors=self.cascade_min_neighbors,
                    minSize=self.cascade_min_size,
                    maxSize=self.cascade_max_size,
                    flags=cv2.CASCADE_SCALE_IMAGE
                )
                face_count = len(faces)
            except Exception as e:
                self.get_logger().error(f'Face detection failed: {e}')
        
        # Hysteresis state machine for stable face detection
        # This filters out flickering detections by requiring sustained presence/absence
        current_time = time.time()
        raw_face_detected = (face_count > 0)
        
        # Check if raw detection changed from last frame
        if raw_face_detected != (self.last_raw_face_count > 0):
            # Transition started - reset timer
            self.face_transition_start_time = current_time
        
        self.last_raw_face_count = face_count
        
        # Hysteresis logic: require sustained signal before changing stable state
        if self.face_transition_start_time is not None:
            time_in_transition = current_time - self.face_transition_start_time
            
            if self.face_stable_state is False and raw_face_detected:
                # Currently in "stable absence", raw says "present"
                # Wait for face_presence_threshold before transitioning to "stable presence"
                if time_in_transition >= self.face_presence_threshold:
                    self.face_stable_state = True
                    self.face_transition_start_time = None
                    self.get_logger().info(f'Face detection stable: PRESENCE confirmed after {time_in_transition:.1f}s')
            
            elif self.face_stable_state is True and not raw_face_detected:
                # Currently in "stable presence", raw says "absent"
                # Wait for face_absence_threshold before transitioning to "stable absence"
                if time_in_transition >= self.face_absence_threshold:
                    self.face_stable_state = False
                    self.face_transition_start_time = None
                    self.get_logger().info(f'Face detection stable: ABSENCE confirmed after {time_in_transition:.1f}s')
        
        # Publish stable face count (smoothed via hysteresis)
        stable_face_count = 1 if self.face_stable_state else 0
        face_count_msg = Int32()
        face_count_msg.data = stable_face_count
        self.face_count_publisher.publish(face_count_msg)
        
        # Perform face recognition if enabled and face is in stable presence state
        if self.recognition_enabled and self.face_stable_state and face_count > 0:
            # Use frame skip to manage CPU load (process every Nth frame)
            self.recognition_frame_counter += 1
            if self.recognition_frame_counter >= self.recognition_frame_skip:
                self.recognition_frame_counter = 0
                
                # Process the first detected face for recognition
                if len(faces) > 0:
                    (x, y, w, h) = faces[0]  # Use first face
                    face_roi = gray_image[y:y+h, x:x+w]
                    face_resized = cv2.resize(face_roi, (100, 100))
                    
                    try:
                        label, confidence = self.face_recognizer.predict(face_resized)
                        
                        # Interpret result (label=0 is target person, lower confidence is better)
                        is_target_person = (confidence < self.recognition_threshold)
                        person_name = self.target_person_name if is_target_person else "unknown"
                        
                        # Debug log confidence
                        self.get_logger().debug(f"Face detected: label={label}, confidence={confidence:.2f}, threshold={self.recognition_threshold}, recognized={person_name}")
                        
                        # Publish person ID
                        person_id_msg = String()
                        person_id_msg.data = person_name
                        self.person_id_publisher.publish(person_id_msg)
                        
                        # Store last person ID for gesture recognition gating
                        self.last_person_id = person_name
                        
                        # Publish confidence
                        confidence_msg = Float32()
                        confidence_msg.data = float(confidence)
                        self.face_confidence_publisher.publish(confidence_msg)
                        
                        # Publish boolean for convenience
                        is_person_msg = Bool()
                        is_person_msg.data = is_target_person
                        self.is_person_publisher.publish(is_person_msg)
                        
                    except Exception as e:
                        self.get_logger().error(f'Face recognition failed: {e}')
        
        # Perform gesture recognition if enabled and ANY authorized person recognized
        if self.gesture_recognizer_enabled:
            # Use frame skip to manage CPU load
            self.gesture_frame_counter += 1
            if self.gesture_frame_counter >= self.gesture_frame_skip:
                self.gesture_frame_counter = 0
                
                # MULTI-USER: Any recognized person (not "unknown", not empty) can use gestures
                # The training itself is the authorization - if LBPH recognizes them, they're authorized
                if self.last_person_id and self.last_person_id != "unknown":
                    try:
                        gesture_name, confidence = self._predict_gesture(downscaled)
                        
                        if gesture_name and confidence > self.gesture_confidence_threshold:
                            # Publish gesture event only on state change (not continuous)
                            if gesture_name != self.last_gesture:
                                gesture_msg = String()
                                gesture_msg.data = gesture_name
                                self.gesture_event_publisher.publish(gesture_msg)
                                self.last_gesture = gesture_name
                                self.get_logger().debug(f"Gesture detected: {gesture_name} (confidence: {confidence:.2f})")
                        else:
                            # Clear last gesture if no valid gesture detected
                            self.last_gesture = None
                    except Exception as e:
                        self.get_logger().error(f'Gesture recognition failed: {e}')
        
        # Log frame data periodically based on log_every_n parameter
        if self.frame_count % self.log_every_n == 0:
            current_time = time.time()
            elapsed = current_time - self.last_log_time
            
            if elapsed >= 1.0:
                # Calculate FPS for this time window
                fps = self.fps_frame_count / elapsed if elapsed > 0 else 0
                self.get_logger().info(
                    f'Frame #{self.frame_count} | FPS: {fps:.2f} | '
                    f'Original: {original_width}x{original_height} | '
                    f'Brightness: {mean_brightness:.1f} | Faces: {face_count}'
                )
                
                # Log bounding box details if verbose face detection is enabled
                if self.log_faces and face_count > 0:
                    for i, (x, y, w, h) in enumerate(faces):
                        self.get_logger().info(
                            f'  Face {i+1}: position=({x}, {y}), size={w}x{h}'
                        )
                
                # Reset counters for next window
                self.last_log_time = current_time
                self.fps_frame_count = 0
        
        # Save RGB debug frame once (first frame)
        if not self.debug_rgb_saved:
            try:
                debug_dir = os.path.dirname(self.debug_frame_path)
                os.makedirs(debug_dir, exist_ok=True)
                cv2.imwrite(self.debug_frame_path, cv_image)
                self.get_logger().info(f'RGB debug frame saved to: {self.debug_frame_path}')
                self.debug_rgb_saved = True
            except Exception as e:
                self.get_logger().error(f'Failed to save RGB debug frame: {e}')
        
        # Save grayscale debug frame once if enabled
        if self.save_debug_gray and not self.debug_gray_saved:
            try:
                debug_dir = os.path.dirname(self.debug_gray_frame_path)
                os.makedirs(debug_dir, exist_ok=True)
                cv2.imwrite(self.debug_gray_frame_path, gray_image)
                self.get_logger().info(f'Grayscale debug frame saved to: {self.debug_gray_frame_path}')
                self.debug_gray_saved = True
            except Exception as e:
                self.get_logger().error(f'Failed to save grayscale debug frame: {e}')
    
    def _extract_hand_landmarks(self, image):
        """
        Extract hand landmarks from image using MediaPipe.
        
        Args:
            image: OpenCV image (BGR)
            
        Returns:
            numpy array of 63 features (21 landmarks × 3 coords) or None
        """
        if self.hands is None:
            return None
        
        # Convert to RGB for MediaPipe
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Process with MediaPipe
        results = self.hands.process(image_rgb)
        
        if not results.multi_hand_landmarks:
            return None
        
        # Extract first detected hand
        hand_landmarks = results.multi_hand_landmarks[0]
        
        # Convert to numpy array (21 landmarks × 3 coords = 63 features)
        landmarks = []
        for landmark in hand_landmarks.landmark:
            landmarks.extend([landmark.x, landmark.y, landmark.z])
        
        return np.array(landmarks)
    
    def _normalize_landmarks(self, landmarks):
        """
        Normalize landmarks to be scale and position invariant.
        
        Args:
            landmarks: numpy array of 63 features
            
        Returns:
            normalized numpy array
        """
        # Reshape to (21, 3)
        landmarks_reshaped = landmarks.reshape(21, 3)
        
        # Get wrist position (landmark 0)
        wrist = landmarks_reshaped[0]
        
        # Translate to origin (wrist at 0,0,0)
        landmarks_centered = landmarks_reshaped - wrist
        
        # Calculate hand size (distance from wrist to middle finger MCP)
        middle_mcp = landmarks_centered[9]  # Middle finger MCP
        hand_size = np.linalg.norm(middle_mcp)
        
        # Scale by hand size (avoid division by zero)
        if hand_size > 0.001:
            landmarks_normalized = landmarks_centered / hand_size
        else:
            landmarks_normalized = landmarks_centered
        
        # Flatten back to 63 features
        return landmarks_normalized.flatten()
    
    def _predict_gesture(self, image):
        """
        Predict gesture from image.
        
        Args:
            image: OpenCV image (BGR)
            
        Returns:
            (gesture_name, confidence) or (None, 0.0)
        """
        if self.gesture_classifier is None or self.gesture_scaler is None:
            return None, 0.0
        
        # Extract landmarks
        landmarks = self._extract_hand_landmarks(image)
        
        if landmarks is None:
            return None, 0.0
        
        # Normalize
        landmarks_normalized = self._normalize_landmarks(landmarks)
        
        # Scale features
        features_scaled = self.gesture_scaler.transform([landmarks_normalized])
        
        # Predict
        prediction = self.gesture_classifier.predict(features_scaled)[0]
        probabilities = self.gesture_classifier.predict_proba(features_scaled)[0]
        
        gesture_name = self.label_to_gesture[prediction]
        confidence = max(probabilities)
        
        return gesture_name, confidence


def main(args=None):
    """
    Main entry point for the image listener node.
    Initializes ROS2 and spins the node.
    """
    rclpy.init(args=args)
    
    # Create and spin the node
    image_listener = ImageListener()
    rclpy.spin(image_listener)
    
    # Cleanup on shutdown
    image_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
