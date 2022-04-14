# =============== Docker container run on RPi ============

# Base image (note: ros:foxy = ros:foxy-base)
FROM ros:foxy

# Zero interaction while installing or upgrading the system via apt. 
# It accepts the default answer for all questions.
ARG DEBIAN_FRONTEND=noninteractive

# ROS custom workspace directory
ARG ROS_CUSTOM_WS=/ros_ws

# Create ROS workspace directory and src directory inside it for storing source code of custom packages
#RUN mkdir -p ~/dev_ws/src 
RUN mkdir -p ${ROS_CUSTOM_WS}/src

################ BUILD & INSTALL ROS2 packages ####################

# Install libboost (needed to build ros2_roboclaw_driver from source (C++))
#RUN apt-get update && \
#    apt-get install -y libboost-dev \
#    python3-pip \
#    && rm -rf /var/lib/apt/lists/*

# Clone sources of teleop_twist_joy package into ROS workspace directory
# The purpose of this package is to provide a generic facility for tele-operating 
# Twist-based ROS2 robots with a standard joystick. It also converts joy messages 
# to velocity commands (sensor_msgs/msg/Joy --> geometry_msgs/msg/Twist)
# None! It is also possible to install this package from APT: apt-get install ros-<rosdistro>-teleop-twist-joy
RUN cd ${ROS_CUSTOM_WS}/src && \
    git clone https://github.com/ros2/teleop_twist_joy.git
    #cd teleop_twist_joy && \
    #git checkout foxy

# We need to update APT database for the next step - rosdep install
# So rosdep install can find and install all needed packages
RUN apt-get update

# Install ROS dependencies (ROS packages that packages located in the src directory depend on)
# - y tell the package manager to default to y or fail when installing
# - r continue installing despite errors
RUN cd ${ROS_CUSTOM_WS} && \
    rosdep install --from-path src --ignore-src -y

# Clean the image
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

# Change working directory (when container is started)
WORKDIR ${ROS_CUSTOM_WS}

################ Final enviroment setup ####################

# --- Setup entrypoint
COPY ./ros_entrypoint.sh /

# [] - is exec form
# exec form vs shell form - the difference is whether the specified command is invoked inside a shell or not
# Exec - runs the process directly (not inside a shell) (doesn't create separate process to run a command/program)
# So shell process with PID 1 is replaced by the process of running program in exec mode
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
#CMD ["ros2", "launch", "bigfoot_bringup", "bigfoot.launch.py"]

# NB! alternative to above 2 lines of code (COPY ... and ENTRYPOINT [and creating new ros_entrypoint.sh])
# can be the next script (it replaces [using command 'sed'] source of ROS2 main underlay 
# by the overlay source of the custom working space) (this code was copied from 
# https://github.com/rbonghi/isaac_ros_tutorial/blob/main/02-realsense_camera/Dockerfile):
# Source ros package from entrypoint
#RUN sed --in-place --expression \
#      '$isource "$ROS_CUSTOM_WS/install/setup.bash"' \
#      /ros_entrypoint.sh 

# ---

#  Run ros package launch file
#CMD ["ros2", "launch", "ros2_roboclaw_driver", "ros2_roboclaw_driver.launch.py"]

# install ros package
#RUN apt-get update && apt-get install -y \
 #     ros-${ROS_DISTRO}-demo-nodes-cpp \
  #    ros-${ROS_DISTRO}-demo-nodes-py && \
   # rm -rf /var/lib/apt/lists/*

# launch ros package
#CMD ["ros2", "launch", "demo_nodes_cpp", "talker_listener.launch.py"]