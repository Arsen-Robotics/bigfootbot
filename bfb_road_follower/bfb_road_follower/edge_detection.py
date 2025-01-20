import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class EdgeDetectionNode(Node):
    def __init__(self):
        super().__init__('edge_detection_node')
        self.sub_rgb = self.create_subscription(Image, '/color/image_raw', self.edge_detection_callback, 10)
        self.pub_edge = self.create_publisher(Image, 'road_edge', 10)
        self.pub_image_out = self.create_publisher(Image, 'image_out', 10)  # New publisher for highlighted edge
        self.bridge = CvBridge()

    def edge_detection_callback(self, msg):
        # Convert ROS Image to OpenCV format
        rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to grayscale
        gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

        # Apply edge detection (Canny, Sobel, etc.)
        edges = cv2.Canny(gray_image, threshold1=100, threshold2=200)

        # Highlight the edges in red on the original image
        highlighted_image = rgb_image.copy()
        highlighted_image[edges == 255] = [0, 0, 255]  # Set edge pixels to red (BGR: [0, 0, 255])

        # Publish the processed edge-detected image
        edge_msg = self.bridge.cv2_to_imgmsg(edges, encoding='mono8')
        self.pub_edge.publish(edge_msg)

        # Publish the image with highlighted edges
        highlighted_edge_msg = self.bridge.cv2_to_imgmsg(highlighted_image, encoding='bgr8')
        self.pub_image_out.publish(highlighted_edge_msg)

def main(args=None):
    rclpy.init(args=args)
    edge_detection_node = EdgeDetectionNode()
    rclpy.spin(edge_detection_node)
    edge_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
