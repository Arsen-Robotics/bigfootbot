FROM osrf/ros:foxy-desktop

# Zero interaction while installing or upgrading the system via apt. 
# It accepts the default answer for all questions.
ARG DEBIAN_FRONTEND=noninteractive

# ROS custom workspace directory
ARG ROS_CUSTOM_WS=/ros_ws

# Install ROS2 packages: join-state-publisher, xacro, ros_navigation2, 
# nav2_bringup, turtlebot3 (many packages like turtlebot3, turtlebot3_bringup, ...)
# ---- TODO check add flag '--no-install-recommends' to apt-get install? ---
#RUN apt-get update && apt-get install -y \   
#    ros-foxy-joint-state-publisher-gui \
#    ros-foxy-xacro \
#    ros-foxy-navigation2 \
#    ros-foxy-nav2-bringup \
#    ros-foxy-turtlebot3* \
#    && rm -rf /var/lib/apt/lists/*

# Create ROS workspace directory and src directory inside it for storing source code of custom packages
#RUN mkdir -p ~/dev_ws/src 
RUN mkdir -p ${ROS_CUSTOM_WS}/src

# Copy contents (source) of the package basic_mobile_robot into container's 
# ROS working directory
COPY . ${ROS_CUSTOM_WS}/src/basic_mobile_robot

# We need to update APT database for the next step - rosdep install
# So rosdep install can find and install all needed packages
RUN apt-get update

# Install ROS dependencies (ROS packages that packages located in the src directory depend on)
# - y tell the package manager to default to y or fail when installing
# - r continue installing despite errors
RUN cd ${ROS_CUSTOM_WS} && \
    rosdep install --from-path src --ignore-src -y

# Clean image
RUN rm -rf /var/lib/apt/lists/*
# --- Build the ROS2 custom workspace
# Colcon build arguments:
# -- symlink-install if you change code in your source code, it will take effect, 
# you don't have to compile it again (ony with python, not c++)
# --packages-up-to my_package - builds the package you want, plus all its dependencies, but not the whole workspace (saves time)
# --packages-select my_package - to build only the my_package package
# --event-handlers console_direct+ shows console output while building (can otherwise be found in the log directory)
RUN cd ${ROS_CUSTOM_WS} && \
    . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install

