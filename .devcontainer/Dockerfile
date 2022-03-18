# Base image (note: ros:foxy = ros:foxy-base)
FROM ros:foxy

# Zero interaction while installing or upgrading the system via apt. 
# It accepts the default answer for all questions.
ARG DEBIAN_FRONTEND=noninteractive

# ROS custom workspace directory
ARG ROS_CUSTOM_WS=/ros

# Create ROS workspace directory and src directory inside it for storing source code of custom packages
#RUN mkdir -p ~/dev_ws/src 
RUN mkdir -p ${ROS_CUSTOM_WS}/src 


# Colcon build arguments:
# -- symlink-install if you change code in your source code, it will take effect, 
# you don't have to compile it again (ony with python, not c++)
# --packages-up-to builds the package you want, plus all its dependencies, but not the whole workspace (saves time)
# --event-handlers console_direct+ shows console output while building (can otherwise be found in the log directory)
#RUN cd ${ROS_CUSTOM_WS} && \
#    . /opt/ros/foxy/setup.sh $$ \
#    colcon build --symlink-install

# Change workdir
WORKDIR ${ROS_CUSTOM_WS}
