import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

class RoadFollowerNode(Node):
    def __init__(self):
        super().__init__('road_follower_node')
        self.sub_image = self.create_subscription(Image, 'seg_image', self.image_callback, 10)
        self.twist_pub = self.create_publisher(Twist, 'seg_vel', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        twist_msg = Twist()
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Only keep green (road) mask: adjust color as needed
        lower_green = np.array([0, 250, 0])
        upper_green = np.array([10, 255, 10])
        mask = cv2.inRange(img, lower_green, upper_green)

        # Focus on bottom 1/3 of image
        height = mask.shape[0]
        roi = mask[int(height * 2/3):, :]

        # Find contours
        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                error = 424 - cx
                twist_msg.linear.x = 0.5
                twist_msg.angular.z = 0.01 * error
                self.twist_pub.publish(twist_msg)
            else:
                self.get_logger().info("Zero moments")
        else:
            self.get_logger().info("No road detected")

def main(args=None):
    rclpy.init(args=args)
    road_follower_node = RoadFollowerNode()
    rclpy.spin(road_follower_node)
    road_follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
