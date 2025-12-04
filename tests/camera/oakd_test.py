import depthai as dai
import cv2
import time

pipeline = dai.Pipeline()
cam = pipeline.createColorCamera()
cam.setPreviewSize(640, 480)
cam.setInterleaved(False)
xout = pipeline.createXLinkOut()
xout.setStreamName("preview")
cam.preview.link(xout.input)

with dai.Device(pipeline) as device:
    q = device.getOutputQueue("preview", maxSize=4, blocking=False)
    start = time.time()
    frames = 0

    while frames < 50:
        frame = q.get().getCvFrame()
        frames += 1
        if frames == 1:
            cv2.imwrite("oakd_test_frame.jpg", frame)
    fps = frames / (time.time() - start)
    print(f"Captured {frames} frames, approx {fps:.2f} FPS")
    print("Saved oakd_test_frame.jpg")
