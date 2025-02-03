import cv2
import numpy as np

device = "/dev/video21"
gst_pipeline = f"appsrc ! videoconvert ! video/x-raw,format=YUY2 ! v4l2sink device={device}"

# Open the GStreamer pipeline
out = cv2.VideoWriter(gst_pipeline, cv2.CAP_GSTREAMER, 0, 30, (640, 480))

if not out.isOpened():
    print("Error: Could not open video device.")
else:
    print("Pipeline opened successfully!")
    while True:
        # Create black frame in BGR format (OpenCV compatible)
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        # Write frame directly in BGR format
        out.write(frame)