import rclpy
from rclpy.node import Node
import os
import numpy as np
import cv2
import time
from sensor_msgs.msg import Image as SensorImage
import subprocess


class RpiCameraNode(Node):
    def __init__(self):
        super().__init__('rpi_camera_node')

        self.rtmp_url = "rtp://192.168.5.226:1234/stream"
        self.width = 640
        self.height = 480
        self.fps = 30

        self.subscription = self.create_subscription(
            SensorImage,
            '/camera/image_raw',
            self.output_callback,
            10)

        # Command and params for ffmpeg
        self.command = ['ffmpeg',
                '-y',
                '-f', 'rawvideo',
                '-vcodec', 'rawvideo',
                '-pix_fmt', 'bgr24',
                '-s', "{}x{}".format(width, height),
                '-r', str(fps),
                '-i', '-',
                '-c:v', 'libx264',
                '-pix_fmt', 'yuv420p',
                '-preset', 'ultrafast',
                '-f', 'rtp',
                rtmp_url]

        # Using subprocess and pipe to fetch frame data
        self.p = subprocess.Popen(command, stdin=subprocess.PIPE)

        self.frame_count = 0
        self.frame = None

    def output_callback(data):
        global frame, frame_count, _frame, width, height, p

        if frame is not None:

            self.frame = cv2.cvtColor(cv2.resize(frame, (640,480)), cv2.COLOR_RGB2BGR)
            self.p.stdin.write(self.frame.tobytes())
            
        self.frame = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        self.frame_count += 1

def main(args=None):
    rclpy.init(args=args)
    rpi_camera_node = RpiCameraNode()
    rclpy.spin(rpi_camera_node)
    rpi_camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()