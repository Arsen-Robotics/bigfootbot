# RTMP is a protocol that is used for streaming audio, video, and data over the Internet. It is commonly used for live 
# streaming on platforms like YouTube, Facebook, and Twitch. 

# It was developed by Adobe Systems in the mid-2000s and is now an open standard maintained by the Internet Engineering Task Force (IETF).

# flv (Flash Video) is a container format that is commonly used for streaming video over RTMP. It is supported by most RTMP servers and clients.

# Use software encoder libx264
ffmpeg -i /dev/video5 -s 640x480 -c:v libx264 -preset ultrafast -tune zerolatency -f flv rtmp://localhost/live/mystream
ffmpeg -re -i /dev/video5 -c:v libx264 -f flv rtmp://localhost/live/mystream

# /mystream is the stream key for the RTMP server (it can be any string).

# Use hardware encoder h264_v4l2m2m on Raspberry Pi
ffmpeg -f v4l2 -i /dev/video5 -c:v h264_v4l2m2m -preset ultrafast -tune zerolatency -f flv rtmp://localhost/live/mystream

# Use hardware encoder h264_nvenc for Nvidia GPU
ffmpeg -f v4l2 -i /dev/video4 -c:v h264_nvenc -f flv rtmp://localhost/live/mystream


ffmpeg -thread_queue_size 512 -s 640x480 -i /dev/video0 -codec:v h264_v4l2m2m -b:v 8096k -r 30 -f flv rtmp://192.168.178.30/live/mystream

# -b:v is the target bitrate for the video stream (8096 kbps in this example).
# -re is the input option that tells ffmpeg to read the input at the native frame rate. It is useful for real-time output (e.g. live streaming).


# Use ffplay to play the video stream
ffplay -fflags nobuffer rtmp://localhost:1935/live/mystream

-fflags nobuffer option is used to disable buffering in ffplay. This is useful for real-time playback of live streams.