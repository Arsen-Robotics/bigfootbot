#!/bin/bash

docker run -it \
--privileged \
--network host \
--runtime=nvidia \
--gpus all \
--rm \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
aarch64_humble_realsense