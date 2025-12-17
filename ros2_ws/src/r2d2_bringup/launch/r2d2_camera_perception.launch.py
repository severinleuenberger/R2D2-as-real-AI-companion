#!/usr/bin/env python3
"""
Launch file for R2D2 camera and perception pipeline.
Starts both the OAK-D camera driver and the perception node with image processing.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate launch description for camera + perception pipeline.
    
    Includes:
    - OAK-D camera driver (r2d2_camera)
    - Perception node (r2d2_perception) for image processing and brightness metrics
    """
    
    # Find package shares
    r2d2_camera_share = FindPackageShare('r2d2_camera')
    r2d2_perception_share = FindPackageShare('r2d2_perception')
    
    # Declare launch arguments for perception node
    save_debug_gray_arg = DeclareLaunchArgument(
        'save_debug_gray_frame',
        default_value='false',
        description='Enable saving grayscale debug frame'
    )
    
    log_every_n_arg = DeclareLaunchArgument(
        'log_every_n_frames',
        default_value='30',
        description='Log perception metrics every N frames'
    )
    
    log_faces_arg = DeclareLaunchArgument(
        'log_face_detections',
        default_value='false',
        description='Enable verbose logging of face detections (bounding box info)'
    )
    
    enable_recognition_arg = DeclareLaunchArgument(
        'enable_face_recognition',
        default_value='false',
        description='Enable personal face recognition using LBPH'
    )
    
    recognition_model_arg = DeclareLaunchArgument(
        'face_recognition_model_path',
        default_value='/home/severin/dev/r2d2/data/face_recognition/models/severin_lbph.xml',
        description='Path to trained LBPH face recognizer model'
    )
    
    recognition_threshold_arg = DeclareLaunchArgument(
        'recognition_confidence_threshold',
        default_value='150.0',
        description='Confidence threshold for recognizing target person (lower is higher confidence)'
    )
    
    recognition_skip_arg = DeclareLaunchArgument(
        'recognition_frame_skip',
        default_value='2',
        description='Process face recognition every N frames to manage CPU load'
    )
    
    target_person_arg = DeclareLaunchArgument(
        'target_person_name',
        default_value='target_person',
        description='Name of the person to recognize (should match training data)'
    )
    
    # Declare launch arguments for gesture recognition
    enable_gesture_arg = DeclareLaunchArgument(
        'enable_gesture_recognition',
        default_value='true',
        description='Enable gesture recognition for intent triggers'
    )
    
    gesture_model_arg = DeclareLaunchArgument(
        'gesture_recognition_model_path',
        default_value='/home/severin/dev/r2d2/data/gesture_recognition/models/severin_gesture_classifier.pkl',
        description='Path to trained gesture classifier model'
    )
    
    gesture_threshold_arg = DeclareLaunchArgument(
        'gesture_confidence_threshold',
        default_value='0.7',
        description='Confidence threshold for gesture recognition (0.0-1.0)'
    )
    
    gesture_skip_arg = DeclareLaunchArgument(
        'gesture_frame_skip',
        default_value='5',
        description='Process gesture recognition every N frames to manage CPU load'
    )
    
    # Include camera launch file from r2d2_camera
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([r2d2_camera_share, 'launch', 'camera.launch.py'])
        )
    )
    
    # Include perception launch file from r2d2_perception
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([r2d2_perception_share, 'launch', 'perception.launch.py'])
        ),
        launch_arguments={
            'save_debug_gray_frame': LaunchConfiguration('save_debug_gray_frame'),
            'log_every_n_frames': LaunchConfiguration('log_every_n_frames'),
            'log_face_detections': LaunchConfiguration('log_face_detections'),
            'enable_face_recognition': LaunchConfiguration('enable_face_recognition'),
            'face_recognition_model_path': LaunchConfiguration('face_recognition_model_path'),
            'recognition_confidence_threshold': LaunchConfiguration('recognition_confidence_threshold'),
            'recognition_frame_skip': LaunchConfiguration('recognition_frame_skip'),
            'target_person_name': LaunchConfiguration('target_person_name'),
            'enable_gesture_recognition': LaunchConfiguration('enable_gesture_recognition'),
            'gesture_recognition_model_path': LaunchConfiguration('gesture_recognition_model_path'),
            'gesture_confidence_threshold': LaunchConfiguration('gesture_confidence_threshold'),
            'gesture_frame_skip': LaunchConfiguration('gesture_frame_skip'),
        }.items()
    )
    
    # Return complete launch description
    return LaunchDescription([
        save_debug_gray_arg,
        log_every_n_arg,
        log_faces_arg,
        enable_recognition_arg,
        recognition_model_arg,
        recognition_threshold_arg,
        recognition_skip_arg,
        target_person_arg,
        enable_gesture_arg,
        gesture_model_arg,
        gesture_threshold_arg,
        gesture_skip_arg,
        camera_launch,
        perception_launch,
    ])
