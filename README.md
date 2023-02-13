# Bigfootbot

## View robot model
### Normal launch
- `ros2 launch bigfootbot_description view_model.launch.py`

### If you want to see the available arguments you can pass to the launch file from the terminal window, type
- `ros2 launch -s bigfootbot_description view_model.launch.py`

### Example of using launch arguments (e.g. launch the robot without RViz)
- `ros2 launch bigfootbot_description view_model.launch.py use_rviz:='False'`


## Overview
- `bigfootbot`
- `bigfootbot_base`
- `bigfootbot_bringup`
- `bigfootbot_description`
- `bigfootbot_teleop`

### robot_localization wiki and source
http://docs.ros.org/en/noetic/api/robot_localization/html/
https://github.com/cra-ros-pkg/robot_localization/tree/ros2


## Add to .bashrc
###  ROS2
`#source /opt/ros/foxy/setup.bash`
`source "/home/jevgeni/ros_ws/install/setup.bash"`

### Workaround for URDF and RVIZ2: cylinder not showing
`export LC_NUMERIC="en_US.UTF-8"`

### Gazebo models
`export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros_ws/src/bigfootbot/bigfootbot_description/models` 
`export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros_ws/src/bigfootbot/bigfootbot_description`  
`export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros_ws/src/bigfootbot/basic_mobile_robot/models`

### Create alias 'tf2' to call view_frames.py
*(view_frames is a graphical debugging tool that creates a PDF graph of your current transform tree)*

`alias tf2='cd /var/tmp && ros2 run tf2_tools view_frames.py && evince frames.pdf &'`