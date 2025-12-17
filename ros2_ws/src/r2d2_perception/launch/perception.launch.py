#!/usr/bin/env python3
"""
Launch file for the R2D2 perception pipeline.
Starts the image_listener node to perform image processing and publish perception metrics.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    """
    Generate the launch description for the perception node.
    
    Returns:
        LaunchDescription: Configuration to launch the image_listener node with parameters
    """
    
    # Declare launch arguments for RGB debug frame
    debug_frame_path_arg = DeclareLaunchArgument(
        'debug_frame_path',
        default_value='/home/severin/dev/r2d2/tests/camera/perception_debug.jpg',
        description='Path to save RGB debug frame JPEG'
    )
    
    # Declare launch argument for grayscale debug frame saving
    save_debug_gray_arg = DeclareLaunchArgument(
        'save_debug_gray_frame',
        default_value='false',
        description='Enable saving grayscale debug frame'
    )
    
    # Declare launch argument for grayscale debug frame path
    debug_gray_path_arg = DeclareLaunchArgument(
        'debug_gray_frame_path',
        default_value='/home/severin/dev/r2d2/tests/camera/perception_debug_gray.jpg',
        description='Path to save grayscale debug frame JPEG'
    )
    
    # Declare launch argument for logging frequency
    log_every_n_arg = DeclareLaunchArgument(
        'log_every_n_frames',
        default_value='30',
        description='Log perception metrics every N frames'
    )
    
    # Declare launch argument for verbose face detection logging
    log_faces_arg = DeclareLaunchArgument(
        'log_face_detections',
        default_value='false',
        description='Enable verbose logging of face detections (bounding box info)'
    )
    
    # Declare launch arguments for face recognition (LBPH)
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
        default_value='70.0',
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
    
    target_person_gesture_arg = DeclareLaunchArgument(
        'target_person_gesture_name',
        default_value='target_person',
        description='Name of the person whose gestures to recognize'
    )
    
    # Declare launch arguments for gesture recognition
    enable_gesture_arg = DeclareLaunchArgument(
        'enable_gesture_recognition',
        default_value='false',
        description='Enable gesture recognition for intent triggers'
    )
    
    gesture_model_arg = DeclareLaunchArgument(
        'gesture_model_path',
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
    
    # Create the image listener node with all parameters
    image_listener_node = Node(
        package='r2d2_perception',
        executable='image_listener',
        name='image_listener',
        parameters=[
            {'debug_frame_path': LaunchConfiguration('debug_frame_path')},
            {'save_debug_gray_frame': LaunchConfiguration('save_debug_gray_frame')},
            {'debug_gray_frame_path': LaunchConfiguration('debug_gray_frame_path')},
            {'log_every_n_frames': LaunchConfiguration('log_every_n_frames')},
            {'log_face_detections': LaunchConfiguration('log_face_detections')},
            {'enable_face_recognition': LaunchConfiguration('enable_face_recognition')},
            {'face_recognition_model_path': LaunchConfiguration('face_recognition_model_path')},
            {'recognition_confidence_threshold': LaunchConfiguration('recognition_confidence_threshold')},
            {'recognition_frame_skip': LaunchConfiguration('recognition_frame_skip')},
            {'target_person_name': LaunchConfiguration('target_person_name')},
            {'target_person_gesture_name': LaunchConfiguration('target_person_gesture_name')},
            {'enable_gesture_recognition': LaunchConfiguration('enable_gesture_recognition')},
            {'gesture_model_path': LaunchConfiguration('gesture_model_path')},
            {'gesture_confidence_threshold': LaunchConfiguration('gesture_confidence_threshold')},
            {'gesture_frame_skip': LaunchConfiguration('gesture_frame_skip')},
        ],
        output='screen'
    )
    
    # Return launch description with all arguments and node
    return LaunchDescription([
        debug_frame_path_arg,
        save_debug_gray_arg,
        debug_gray_path_arg,
        log_every_n_arg,
        log_faces_arg,
        enable_recognition_arg,
        recognition_model_arg,
        recognition_threshold_arg,
        recognition_skip_arg,
        target_person_arg,
        target_person_gesture_arg,
        enable_gesture_arg,
        gesture_model_arg,
        gesture_threshold_arg,
        gesture_skip_arg,
        image_listener_node,
    ])
