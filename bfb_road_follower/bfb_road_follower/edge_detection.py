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
        self.pub_lines = self.create_publisher(Image, 'road_lines', 10)  
        self.pub_image_out = self.create_publisher(Image, 'image_out', 10)  
        self.bridge = CvBridge()

        # Params
        self.slope_threshold = 0.8

        # GStreamer pipeline for BGR output with reduced latency (sync=false to reduce buffering)
        self.gst_pipeline = (
            "appsrc is-live=true block=false format=GST_FORMAT_TIME ! "
            "videoconvert ! video/x-raw,format=BGR,width=640,height=480 ! "
            "v4l2sink device=/dev/video21 sync=false"
        )
        self.out = cv2.VideoWriter(self.gst_pipeline, cv2.CAP_GSTREAMER, 0, 30, (640, 480))

        if not self.out.isOpened():
            self.get_logger().error("Failed to open video pipeline.")

    def edge_detection_callback(self, msg):
        # Convert ROS Image to OpenCV format (BGR)
        rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Convert to grayscale and apply edge detection
        gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 2)
        edges = cv2.Canny(blur, 100, 200)

        # Region of interest (ROI): focus on the bottom half of the image where road edges typically appear
        # height, width = edges.shape
        # roi = edges[int(height / 2):height, 0:width]

        # Detect lines using Hough Transform
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, minLineLength=100, maxLineGap=40)
        line_image = np.zeros_like(rgb_image)

        if lines is not None:
            for x1, y1, x2, y2 in lines[:, 0]:
                cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Draw all detected lines

        # if lines is not None:
        #     for x1, y1, x2, y2 in lines[:, 0]:
        #         # Calculate the slope of the line
        #         if x2 != x1:  # Prevent division by zero
        #             slope = (y2 - y1) / (x2 - x1)
        #         else:
        #             slope = float('inf')  # For vertical lines
                
        #         # Filter out lines that don't meet the slope criteria
        #         if abs(slope) > self.slope_threshold:  # Adjust this threshold to your needs
        #             cv2.line(line_image, (x1, y1 + int(height / 2)), (x2, y2 + int(height / 2)), (0, 255, 0), 2)

        # Overlay detected lines on the original image
        overlayed_image = cv2.addWeighted(rgb_image, 0.8, line_image, 1, 0)

        # Resize the output image to 640x480 to match GStreamer resolution
        resized_image_640x480 = cv2.resize(overlayed_image, (640, 480))

        # Write the resized image to GStreamer pipeline (this is the video that will appear on /dev/video21)
        self.out.write(resized_image_640x480)

        # Publish both images to ROS (RViz and other subscribers)
        self.pub_image_out.publish(self.bridge.cv2_to_imgmsg(overlayed_image, 'bgr8'))
        #self.pub_lines.publish(self.bridge.cv2_to_imgmsg(roi, 'mono8'))

    def destroy_node(self):
        self.out.release()  # Release GStreamer pipeline on shutdown
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    edge_detection_node = EdgeDetectionNode()
    rclpy.spin(edge_detection_node)
    edge_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
