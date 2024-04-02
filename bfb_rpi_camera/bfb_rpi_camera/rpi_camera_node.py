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

        self.srt_url = "udp://192.168.5.226:1234/stream"
        self.width = 640
        self.height = 480
        self.fps = 30

        self.subscription = self.create_subscription(
            SensorImage,
            '/image_raw',
            self.output_callback,
            10)

        # Command and params for ffmpeg
        self.command = ['ffmpeg',
                '-y',
                '-f', 'rawvideo',
                '-vcodec', 'rawvideo',
                '-pix_fmt', 'bgr24',
                '-s', "{}x{}".format(self.width, self.height),
                '-r', str(self.fps),
                '-i', '-',
                '-c:v', 'libx264',
                '-pix_fmt', 'yuv420p',
                '-preset', 'ultrafast',
                '-f', 'mpegts',
                self.srt_url]

        # Using subprocess and pipe to fetch frame data
        self.p = subprocess.Popen(self.command, stdin=subprocess.PIPE)

    def output_callback(self, msg):
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        frame = cv2.cvtColor(cv2.resize(frame, (self.width, self.height)), cv2.COLOR_RGB2BGR)
        self.p.stdin.write(frame.tobytes())

def main(args=None):
    rclpy.init(args=args)
    rpi_camera_node = RpiCameraNode()
    rclpy.spin(rpi_camera_node)
    rpi_camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()