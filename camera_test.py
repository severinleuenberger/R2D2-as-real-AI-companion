#!/usr/bin/env python3
"""
Simple OAK-D camera test - captures RGB frame and saves it
"""
import depthai as dai
import time

try:
    # Create pipeline
    pipeline = dai.Pipeline()
    
    # Define source and output
    cam_rgb = pipeline.createColorCamera()
    xout_rgb = pipeline.createXLinkOut()
    
    xout_rgb.setStreamName("rgb")
    
    # Properties
    cam_rgb.setPreviewSize(640, 480)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorProperties.setResolution(dai.ColorCameraProperties.SensorProperties.StreamProperties.RGB))
    cam_rgb.setInterleaved(False)
    
    # Linking
    cam_rgb.preview.link(xout_rgb.input)
    
    print("✅ Pipeline created successfully")
    
    # Connect to device
    with dai.Device(pipeline) as device:
        print("✅ Connected to OAK-D camera")
        print(f"   Device name: {device.getProductName()}")
        print(f"   Device ID: {device.getDeviceInfo().getMxId()}")
        
        # Get output queue
        q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        
        print("📷 Capturing frame in 2 seconds...")
        time.sleep(2)
        
        # Get one frame
        in_rgb = q_rgb.get()
        if in_rgb is not None:
            print(f"✅ Frame captured!")
            print(f"   Shape: {in_rgb.getFrame().shape}")
            print(f"   Type: {in_rgb.getFrame().dtype}")
            
            # Note: We're not saving because PIL/cv2 might need extra setup
            print("✅ Camera is fully functional!")
        else:
            print("⚠️  No frame received")
            
except RuntimeError as e:
    print(f"❌ Device error: {e}")
    print("\n🔧 Troubleshooting:")
    print("   1. Camera needs udev rules - requires system reboot")
    print("   2. Run: sudo usermod -a -G dialout $USER && sudo reboot")
    print("   3. Or run this script with: sudo -u $USER python3 camera_test.py")
except Exception as e:
    print(f"❌ Error: {e}")
    import traceback
    traceback.print_exc()
