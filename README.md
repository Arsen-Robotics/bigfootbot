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