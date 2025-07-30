import cv2
import torch
import numpy as np
import time
from transformers import AutoImageProcessor, SegformerForSemanticSegmentation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class SegmentationNode(Node):
    def __init__(self):
        super().__init__('segmentation_node')

        self.seg_pub = self.create_publisher(Image, 'seg_image', 10)
        self.bridge = CvBridge()

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {self.device}")

        # Load processor and model
        self.processor = AutoImageProcessor.from_pretrained("ayoubkirouane/Segments-Sidewalk-SegFormer-B0")
        self.model = SegformerForSemanticSegmentation.from_pretrained("ayoubkirouane/Segments-Sidewalk-SegFormer-B0").to(self.device)
        self.model.eval()

        self.width = 848
        self.height = 480

        self.cap = cv2.VideoCapture(
            f"v4l2src device=/dev/video9 ! video/x-raw, width={self.width}, height={self.height} ! videoconvert ! appsink",
            cv2.CAP_GSTREAMER
        )

        # GStreamer output of segmented video to virtual video device
        self.gst_pipeline = (
            "appsrc ! videoconvert ! video/x-raw,format=YUY2 ! identity drop-allocation=1 ! "
            "v4l2sink device=/dev/video20 sync=false"
        )
        self.out = cv2.VideoWriter(self.gst_pipeline, cv2.CAP_GSTREAMER, 0, 5, (self.width, self.height))
        assert self.out.isOpened(), "Cannot open /dev/video20 for writing"

        print("Starting segmentation stream.")
        print("Legend:")
        print("Green: Road (class 1)")
        print("Blue: Sidewalk (class 2)")
        print("Yellow: Crosswalk (class 3)")
        print("Red: Cycling lane (class 4)")

        self.frame_interval = 1.0 / 5
        self.create_timer(self.frame_interval, self.process_frame)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            print("Failed to read frame")
            return
        
        # Preprocess without resizing
        inputs = self.processor(images=frame, return_tensors="pt", do_resize=False).to(self.device)

        with torch.no_grad():
            outputs = self.model(**inputs)

        # Get predicted class IDs
        seg = torch.argmax(outputs.logits, dim=1)[0].cpu().numpy().astype(np.uint8)

        # Resize segmentation mask to original resolution (if needed)
        seg_resized = cv2.resize(seg, (self.width, self.height), interpolation=cv2.INTER_NEAREST)

        # Create colored overlay for specific classes
        overlay = np.zeros_like(frame)

        # Road = green
        overlay[seg_resized == 1] = (0, 255, 0)

        # Sidewalk = blue
        overlay[seg_resized == 2] = (255, 0, 0)
        
        # Crosswalk = yellow
        overlay[seg_resized == 3] = (0, 255, 255)

        # Cycling lane = red
        overlay[seg_resized == 4] = (0, 0, 255)

        # Blend with original image
        alpha = 0.5
        beta = 0.8
        output = cv2.addWeighted(overlay, alpha, frame, beta, 0)

        # Publish overlay only to ROS2 topic
        overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        self.seg_pub.publish(overlay_msg)

        # Write to virtual video device
        self.out.write(overlay)

    def destroy_node(self):
        self.cap.release()
        self.out.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    segmentation_node = SegmentationNode()
    rclpy.spin(segmentation_node)
    segmentation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
