import cv2
import depthai as dai

# Create pipeline
pipeline = dai.Pipeline()

# Create RGB camera node
cam_rgb = pipeline.create(dai.node.ColorCamera)
cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
cam_rgb.setFps(30)

# Create XLink output
xout_rgb = pipeline.create(dai.node.XLinkOut)
xout_rgb.setStreamName("rgb")
cam_rgb.video.link(xout_rgb.input)

# Connect and run
with dai.Device(pipeline) as device:
    q = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    
    # Wait for frame
    in_rgb = q.get()
    frame = in_rgb.getCvFrame()
    
    if frame is None:
        raise RuntimeError("Failed to capture frame from OAK-D")
    
    cv2.imwrite("r2d2_cam_test.jpg", frame)
    print(f"Saved r2d2_cam_test.jpg (shape: {frame.shape})")
