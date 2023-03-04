#!/bin/bash

docker run -it \
--privileged \
--network host \
--runtime=nvidia \
--gpus all \
--rm \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \  #  read-write mode (i.e. the container can write as well as read files on the host)
aarch64_humble_realsense