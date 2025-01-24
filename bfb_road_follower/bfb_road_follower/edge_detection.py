import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import subprocess

class EdgeDetectionNode(Node):
    def __init__(self):
        super().__init__('edge_detection_node')
        self.sub_rgb = self.create_subscription(Image, '/color/image_raw', self.edge_detection_callback, 10)
        self.pub_lines = self.create_publisher(Image, 'road_lines', 10)  # Replacing road_edge
        self.pub_image_out = self.create_publisher(Image, 'image_out', 10)  # Overlayed road lines
        self.bridge = CvBridge()

        self.gst_pipeline = f"appsrc ! videoconvert ! video/x-raw,format=YUY2 ! v4l2sink device=/dev/video21"
        self.out = cv2.VideoWriter(self.gst_pipeline, cv2.CAP_GSTREAMER, 0, 30, (1280, 720))

        # # GStreamer command for streaming the processed image to /dev/video21
        # self.gst_cmd = [
        #     'gst-launch-1.0',
        #     'appsrc', '!', 'video/x-raw, format=BGR', '!', 'videoconvert', '!', 'video/x-raw, format=YUY2', '!', 'v4l2sink', 'device=/dev/video21'
        # ]

        # self.gst_process = subprocess.Popen(self.gst_cmd, stdin=subprocess.PIPE)

        # Create a timer to control the frame rate
        # self.timer = self.create_timer(1.0 / 15, self.timer_callback)  # 30 FPS

        self.latest_overlay = None
        self.latest_edge = None

    def edge_detection_callback(self, msg):
        # Convert ROS Image to OpenCV format
        rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to grayscale
        gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur
        blur_image = cv2.GaussianBlur(gray_image, (7, 7), 10)

        # Apply edge detection (Canny)
        edges = cv2.Canny(blur_image, 75, 150)

        # Apply trapezoidal mask
        # mask = np.zeros_like(edges)
        # h, w = edges.shape
        # trapezoid = np.array([[(w * 0.1, h), (w * 0.4, h * 0.6), (w * 0.6, h * 0.6), (w * 0.9, h)]], np.int32)
        # cv2.fillPoly(mask, trapezoid, 255)
        # masked_edges = cv2.bitwise_and(edges, mask)

        # Detect lines using Hough Line Transform
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 30, maxLineGap=30)

        # Create a blank image to draw lines
        line_image = np.zeros_like(rgb_image)

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Green lines

        # Overlay road lines on the original image
        overlayed_image = cv2.addWeighted(rgb_image, 0.8, line_image, 1, 0)

        # # Store the latest image for later use in the timer callback
        self.latest_overlay = overlayed_image
        self.latest_edge = line_image

        self.publish_images()

    def publish_images(self):
        # # Generate a color gradient test image
        # height, width = 480, 640
        # gradient = np.zeros((height, width, 3), dtype=np.uint8)
        # for i in range(width):
        #     gradient[:, i] = (i * 255 // width, 255 - i * 255 // width, (i * 255 // width) // 2)
        # self.out.write(gradient)

        if self.latest_overlay is not None:
            overlay = self.latest_overlay

            # Convert the image to raw bytes (BGR format)
            # image_bytes = overlay.tobytes()

            # overlay = cv2.cvtColor(self.latest_overlay, cv2.COLOR_BGR2YUV)

            # Write the frame to the GStreamer process
            # self.out.write(overlay)

            # # Create black frame in BGR format (OpenCV compatible)
            # frame = np.zeros((480, 848, 3), dtype=np.uint8)

            # # Write frame directly in BGR format
            # self.out.write(frame)

            # Publish the image with overlaid road lines
            overlayed_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            self.pub_image_out.publish(overlayed_msg)

        if self.latest_edge is not None:
            edges = self.latest_edge

            self.out.write(edges)
        
            # Publish the road lines image
            line_msg = self.bridge.cv2_to_imgmsg(edges, encoding='bgr8')
            self.pub_lines.publish(line_msg)
            

def main(args=None):
    rclpy.init(args=args)
    edge_detection_node = EdgeDetectionNode()
    rclpy.spin(edge_detection_node)
    edge_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
