#!/bin/bash

# Exit immediately (from script) if a command exits with a non-zero status.
set -e

# --- Source overlay ---
# Note: https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html
# NB!!!!! Before sourcing the overlay, it is very important that you open a !!! new terminal !!!
# separate from the one where you built the workspace. Sourcing an overlay in the same 
# terminal where you built, or likewise building where an overlay is sourced, may create complex issues.

# Sourcing the local_setup of the overlay will only add the packages available in the overlay to your environment. 
# setup sources the overlay as well as the underlay it was created in, allowing you to utilize both workspaces.
# So, sourcing your main ROS 2 installation’s setup and then the dev_ws overlay’s local_setup
# (source /opt/ros/foxy/setup.bash and then /ros/install/local_setup.bash
# is the same as just sourcing dev_ws’s setup, because that includes the environment of the underlay it was created in.

# Source custom workspace overlay with underlay it was created in 
# (in our case underlay is ROS2 main overlay which has no parent overlay) 
source "/ros_ws/install/setup.bash"
#source "/ros_ws/install/local_setup.bash"

# This line is from default ros_entrypoint.sh script file
#source "/opt/ros/$ROS_DISTRO/setup.bash"

# If you have an image with an entrypoint pointing to entrypoint.sh, and you run 
# your container as docker run my_image server start, that will translate 
# to running entrypoint.sh server start in the container. 
# At the exec line entrypoint.sh, the shell running as pid 1 will replace itself with the command server start.
exec "$@"