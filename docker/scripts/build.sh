#!/bin/bash

bold=$(tput bold)
red=$(tput setaf 1)
green=$(tput setaf 2)
yellow=$(tput setaf 3)
reset=$(tput sgr0)

function usage() {
    echo "Usage: build_base_image.sh" {target image, period delimited components, required} {target image name, optional}
    echo "Copyright (c) 2022, NVIDIA CORPORATION."
}

function main {
    if [ -z "$1" ] # -z means non-defined or empty
        then
            echo "${red}No argument supplied"
            exit 1
    fi

    local PLATFORM="$(uname -m)"
    # Check if is running on NVIDIA Jetson platform
    if [ $PLATFORM != "aarch64" ] 
        then
        echo "${red}Run this script only on ${bold}${green}NVIDIA${reset}${red} Jetson platform${reset}"
        exit 33
    fi
}

main $@
# EOF

#docker build -f Dockerfile.aarch64.humble.nav2 \
  #-t aarch64_humble_nav2 ..

#docker build -f Dockerfile.realsense \
#  -t aarch64_humble_realsense:latest \
#  --build-arg BASE_IMAGE="aarch64_humble_nav2" ..