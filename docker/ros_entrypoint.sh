#!/bin/bash

# Exit immediately (from script) if a command exits with a non-zero status.
set -e

# --- Source overlay ---
# Note: https://docs.ros.org/en/humble/Tutorials/Workspace/Creating-A-Workspace.html
# NB!!!!! Before sourcing the overlay, it is very important that you open a !!! new terminal !!!
# separate from the one where you built the workspace. Sourcing an overlay in the same terminal where you built, 
# or likewise building where an overlay is sourced, may create complex issues.

# Sourcing the local_setup of the overlay will only add the packages available in the overlay to your environment. 
# setup sources the overlay as well as the underlay it was created in, allowing you to utilize both workspaces.
# So, sourcing your main ROS 2 installation’s setup and then the dev_ws overlay’s local_setup
# (source /opt/ros/$ROS_DISTRO/setup.bash and then /ros/install/local_setup.bash
# is the same as just sourcing dev_ws’s setup, because that includes the environment of the underlay it was created in.

# Source custom workspace overlay with underlay it was created in 
# (in our case underlay is ROS2 main overlay which has no parent overlay) 
#source ~/ros2_ws/install/setup.bash
echo "source ${ROS_WS}/install/setup.bash" >> ~/.bashrc
source ${ROS_WS}/install/setup.bash
#source /ros_ws/install/local_setup.bash

# This line is from default ros_entrypoint.sh script file
#source "/opt/ros/$ROS_DISTRO/setup.bash"

# Husarnet
# Set this file as default profile in your .bashrc file, so as to use this configuration every time you boot your system.
#echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_husarnet.xml" >> ~/.bashrc

# Add path to the robot model for Gazebo simulator
# NB! update directory name 'ros_ws' if it is changed in the docker file: ARG ROS_CUSTOM_WS
#export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/ros_ws/src/bigfootbot_description/models/
#export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/ros_ws/src/basic_mobile_robot/models/

# If you have an image with an entrypoint pointing to entrypoint.sh, and you run 
# your container as docker run my_image server start, that will translate 
# to running entrypoint.sh server start in the container. 
# At the exec line entrypoint.sh, the shell running as pid 1 will replace itself with the command server start.
exec "$@"