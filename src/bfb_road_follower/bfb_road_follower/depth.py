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
        self.bridge = CvBridge()

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
        # Convert ROS Image to numpy array (depth image)
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        valid_mask = (depth_image >= 0) & (depth_image <= 5000)

        # Normalize depth data to 0-255 (uint8)
        depth_normalized = (depth_image / 5000) * 255.0
        depth_normalized = depth_normalized.astype(np.uint8)

        # Apply thermal colormap
        depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

        depth_colored[~valid_mask] = 0

        if self.out.isOpened():
            self.out.write(depth_colored)

def main(args=None):
    rclpy.init(args=args)
    depth_node = DepthNode()
    rclpy.spin(depth_node)
    depth_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()