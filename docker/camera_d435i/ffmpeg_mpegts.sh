# MPEG-TS (MPEG transport stream) is a standard format for streaming video over UDP. It is supported by most IP cameras and video servers. 
# Initial release: 1995

ffmpeg -i /dev/video4 -s 960x540 -f mpegts udp://localhost:1234

- 1234 is the port number for the UDP stream.