#!/usr/bin/env python3
"""
Simple OAK-D Lite RGB frame capture - takes one photo and saves it
"""
import depthai as dai
import cv2
import time
import numpy as np

print("📷 OAK-D Lite - First Photo Capture\n" + "="*50)

try:
    # Create pipeline
    pipeline = dai.Pipeline()
    
    # Create RGB camera node
    cam_rgb = pipeline.createColorCamera()
    xout_rgb = pipeline.createXLinkOut()
    
    xout_rgb.setStreamName("rgb")
    
    # Camera properties - use default resolution
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
    cam_rgb.setInterleaved(False)
    cam_rgb.setFps(30)
    
    # Link
    cam_rgb.video.link(xout_rgb.input)
    
    # Connect and get frame
    with dai.Device(pipeline) as device:
        print("✅ Connected to OAK-D")
        
        q_rgb = device.getOutputQueue(name="rgb", maxSize=1, blocking=True)
        
        print("📸 Waiting 3 seconds for sensor to stabilize...")
        time.sleep(3)
        
        print("🔄 Capturing frame...")
        in_rgb = q_rgb.get()
        
        if in_rgb is not None:
            # Get frame data as numpy array
            frame = in_rgb.getCvFrame()
            
            print(f"   Raw frame shape: {frame.shape}")
            
            # Save image
            filename = "/home/severin/dev/r2d2/oak_d_photo.jpg"
            success = cv2.imwrite(filename, frame)
            
            if success:
                print(f"✅ Photo saved: {filename}")
                print(f"   Resolution: {frame.shape[1]}x{frame.shape[0]}")
                print(f"   Channels: {frame.shape[2] if len(frame.shape) > 2 else 1}")
                
                # Display image info
                print("\n" + "="*50)
                print("🎉 FIRST PHOTO SUCCESSFULLY CAPTURED!")
                print("="*50)
                print(f"\nFile saved at: {filename}")
                print(f"File size: ", end="")
                
                import os
                if os.path.exists(filename):
                    size_bytes = os.path.getsize(filename)
                    size_kb = size_bytes / 1024
                    print(f"{size_kb:.1f} KB")
            else:
                print("❌ Failed to write image to disk")
            
        else:
            print("❌ Failed to capture frame")
            
except Exception as e:
    print(f"❌ Error: {e}")
    import traceback
    traceback.print_exc()
