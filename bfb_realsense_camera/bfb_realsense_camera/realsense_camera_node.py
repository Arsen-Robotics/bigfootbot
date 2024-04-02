import rclpy
from rclpy.node import Node
import os
import numpy as np
import cv2
import time
from sensor_msgs.msg import Image
import subprocess


class RealsenseCameraNode(Node):
    def __init__(self):
        super().__init__('realsense_camera_node')

        self.srt_url = "udp://192.168.5.101:1234/stream"
        self.width = 640
        self.height = 480
        self.fps = 30

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.output_callback,
            10) # 10 is the queue size means it will store 10 messages in the queue before it starts to throw away old messages

        # Command and params for ffmpeg
        self.command = [
            'ffmpeg',
            '-y', # overwrite output files without asking
            '-f', 'rawvideo', # specifies the input format as raw video, indicating that there is no container format around the video data.
            '-vcodec', 'rawvideo', # specifies the video codec for input as raw video
            '-pix_fmt', 'bgr24', # pixel format of input image/frame. Pixel format refers to how 
                                 # the color information of each pixel in an image is represented
            '-s', "{}x{}".format(self.width, self.height), # resolution of the input frame. If not specified, it will be the size of
                                                           # the input frame, if specified, it will be resized to the specified resolution
            '-r', str(self.fps), # frame rate of the input video. If not specified, it will be the frame rate of the input video
                                 # if specified, it will be the specified frame rate
            '-i', '-', # input source. '-' means it will take input from standard input (stdin) typically through a pipe or redirection 
                       # from another command or process.
            '-c:v', 'libx264', # -c:v means 'codec:video' Sets the video codec for output as H.264 (using libx264). libx264 
                               # is a software library that provides an H.264/MPEG-4 AVC encoder
            '-pix_fmt', 'yuv420p', # sets the pixel format for output to YUV 4:2:0. YUV 4:2:0 is a planar format where the Y (luminance)
                                   # component is sampled at full resolution, but the U and V (chrominance) components are sampled 
                                   # at half resolution. Becasue the human eye is more sensitive to changes in brightness than color, 
                                   # this format is used to reduce the amount of data that needs to be transmitted. 
                                   # 4:4:4 is full resolution for all components, 4:2:2 is full resolution for Y and half resolution 
                                   # for U and V, and 4:2:0 is full resolution for Y and half resolution for U and V. 
                                   # 4:2:0 has the lowest data rate of the three.
            '-preset', 'ultrafast', # Sets the encoding preset to ultrafast, which prioritizes speed over compression efficiency.
                                    # Available presets are ultrafast, superfast, veryfast, faster, fast, medium, slow, slower, veryslow,
                                    # placebo. The slower the preset, the better the compression efficiency, but the slower the encoding.
                                    # Slower encoding means that the video will take longer to encode, but the output file will be smaller,
                                    # but latency will be higher.
                                    # The placebo preset is the slowest and produces the best compression efficiency means a better quality
                                    # and smallest file size. The default preset is medium.
                                    # For real-time applications, use the ultrafast preset.            
            'tune', 'zerolatency', # tune the encoding parameters for zero latency. This is useful for real-time applications where
                                   # latency is important. Zero latency means that the video will be encoded as quickly as possible, but the
                                   # output file will be larger. The default tune is film.
            #'-b', '900k', # -b means 'bitrate'. Set the target video bitrate to 900k.
            '-f', 'mpegts', # -f means 'format'. Force output file format. mpegts (MPGEG-TS) is the MPEG Transport Stream format.
                            # It is container that contains encoded video and audio streams. It is used for streaming video over the internet.
            self.srt_url] # output URL

        # Create a subprocess to run the ffmpeg command
        self.p = subprocess.Popen(self.command, stdin=subprocess.PIPE)

        #self.frame_count = 0
        #self.frame = None

    def output_callback(self, msg):                
        # Convert the data field of the ROS Image message (msg.data) into a NumPy array.
        # The data field of the ROS Image message is a byte array (uint8[]) that contains the image data.
        # It looks like [R1, G1, B1, R2, G2, B2, ...] or numbers [0, 255, 0, 255, 0, 255, ...]
        frame = np.frombuffer(msg.data, dtype=np.uint8)
        
        # Reshape the 1D NumPy array (image_data) into a 3D array representing the image.
        # -1 in the reshape function means that the size of the dimension is inferred from the length of the array 
        # and the remaining dimensions.
        # In NumPy array first dimension refers to row and second dimension refers to column.
        frame = frame.reshape((msg.height, msg.width, -1))

        # Convert the image data to BGR format becasue FFmpeg expects the input frame to be in BGR format.
        # cvt2.resize returns a resized image as a NumPy array.
        frame = cv2.cvtColor(cv2.resize(frame, (self.width, self.height)), cv2.COLOR_RGB2BGR)

        # Write the frame to the stdin of the FFmpeg process to stream the video.
        self.p.stdin.write(frame.tobytes())

        #if self.frame is not None:
        #    self.frame = cv2.cvtColor(cv2.resize(self.frame, (640,480)), cv2.COLOR_RGB2BGR)
        #    self.p.stdin.write(self.frame.tobytes())
            
        # Convert the image data to a numpy array      
        #self.frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1) 
        #self.frame_count += 1

def main(args=None):
    rclpy.init(args=args)
    realsense_camera_node = RealsenseCameraNode()
    rclpy.spin(realsense_camera_node)
    realsense_camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()