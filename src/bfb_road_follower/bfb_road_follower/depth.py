import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthNode(Node):
    def __init__(self):
        super().__init__('depth_node')
        self.sub_depth = self.create_subscription(Image, '/depth/image_rect_raw', self.depth_callback, 10)
        self.sub_color = self.create_subscription(Image, '/color/image_raw', self.color_callback, 10)
        self.bridge = CvBridge()

        self.depth_img = None
        self.color_img = None

        # GStreamer pipeline for depth output
        self.gst_pipeline = (
            "appsrc is-live=true ! "
            "video/x-raw, format=BGR, width=848, height=480, framerate=30/1 ! "
            "videoconvert ! "
            "video/x-raw, format=YUY2 ! "
            "identity drop-allocation=1 ! v4l2sink device=/dev/video21 sync=false"
        )
        
        self.out = cv2.VideoWriter(self.gst_pipeline, cv2.CAP_GSTREAMER, 0, 30, (848, 480))

        if not self.out.isOpened():
            self.get_logger().error("Failed to open video pipeline.")

    def depth_callback(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.try_process()

    def color_callback(self, msg):
        self.color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.try_process()

    def try_process(self):
        if self.depth_img is not None and self.color_img is not None:
            # Process the depth image
            depth_processed = cv2.applyColorMap(self.depth_img, cv2.COLORMAP_JET)

            # Combine depth and color images (for visualization)
            combined = cv2.addWeighted(depth_processed, 0.5, self.color_img, 0.5, 0)

            # Write to GStreamer pipeline
            self.out.write(combined)

            # Reset images to avoid reprocessing
            self.depth_img = None
            self.color_img = None

def main(args=None):
    rclpy.init(args=args)
    depth_node = DepthNode()
    rclpy.spin(depth_node)
    depth_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()