import io
import picamera
import subprocess

# Start the camera
camera = picamera.PiCamera()
camera.resolution = (640, 480)  # Adjust resolution as needed
camera.framerate = 24  # Adjust framerate as needed

# Open a pipe to ffmpeg as a subprocess
command = [
    'ffmpeg',
    '-y',
    '-f', 'rawvideo',
    '-vcodec', 'rawvideo',
    '-pix_fmt', 'bgr24',
    '-s', "{}x{}".format(camera.resolution[0], camera.resolution[1]),
    '-r', str(framerate),
    '-i', '-',
    '-c:v', 'libx264',
    '-pix_fmt', 'yuv420p',
    '-preset', 'ultrafast',
    '-f', 'rtp',
    'rtp://192.168.5.226:1234/stream']

# Using subprocess and pipe to fetch frame data
p = subprocess.Popen(command, stdin=subprocess.PIPE)

try:
    stream = io.BytesIO()
    for _ in camera.capture_continuous(stream, format='bgr', use_video_port=True):
        stream.seek(0)
        p.stdin.write(stream.read())
        stream.seek(0)
        stream.truncate()
finally:
    p.stdin.close()
    p.wait()
    camera.close()