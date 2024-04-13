ffmpeg -f v4l2 -i /dev/video5 -c:v libx264 -preset ultrafast -tune zerolatency -f flv rtmp://localhost/live/1234

# Use hardware encoder h264_v4l2m2m on Raspberry Pi
ffmpeg -f v4l2 -i /dev/video5 -c:v h264_v4l2m2m -preset ultrafast -tune zerolatency -f flv rtmp://localhost/live/1234

# Use hardware encoder h264_nvenc for Nvidia GPU
ffmpeg -f v4l2 -i /dev/video4 -c:v h264_nvenc -preset ultrafast -tune zerolatency -f flv rtmp://localhost/live/1234


ffmpeg -thread_queue_size 512 -s 640x480 -i /dev/video0 -codec:v h264_v4l2m2m -b:v 8096k -r 30 -f flv rtmp://192.168.178.30/live/mystream

-b:v is the bitrate for the video stream.

-re is the input option that tells ffmpeg to read the input at the native frame rate. It is useful for real-time output (e.g. live streaming).
