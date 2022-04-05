FROM osrf/ros:foxy-desktop

# Zero interaction while installing or upgrading the system via apt. 
# It accepts the default answer for all questions.
ARG DEBIAN_FRONTEND=noninteractive

# ROS custom workspace directory
ARG ROS_CUSTOM_WS=/ros_ws

# Install ROS2 packages: join-state-publisher, xacro, ros_navigation2, 
# nav2_bringup, turtlebot3 (many packages like turtlebot3, turtlebot3_bringup, ...)
# ---- TODO check add flag '--no-install-recommends' to apt-get install? ---
RUN apt-get update && apt-get install -y \   
    #ros-foxy-joint-state-publisher-gui \
    ros-foxy-gazebo-ros-pkgs \
    ros-foxy-xacro \
    ros-foxy-navigation2 \
    ros-foxy-nav2-bringup \
    ros-foxy-turtlebot3* \
    && rm -rf /var/lib/apt/lists/*

# Create ROS workspace directory and src directory inside it for storing source code of custom packages
#RUN mkdir -p ~/dev_ws/src 
RUN mkdir -p ${ROS_CUSTOM_WS}/src

# Copy contents (source) of the package basic_mobile_robot into container's 
# ROS working directory
#COPY ../basic_mobile_robot ${ROS_CUSTOM_WS}/src/basic_mobile_robot