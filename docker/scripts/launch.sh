#!/bin/bash

# Launch Foxglove node
#./launch_foxglove.sh &
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 &
  
# Launch Realsense node
#./launch_realsense.sh &
ros2 launch realsense2_camera rs_launch.py &
  
# Wait for any process to exit
wait -n
  
# Exit with status of process that exited first
exit $?
