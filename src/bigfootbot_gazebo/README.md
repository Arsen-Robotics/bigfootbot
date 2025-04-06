# Add to .bashrc:
## Gazebo models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros_ws/src/bigfootbot/bigfootbot_description/models

# Spawn the robot
ros2 run gazebo_ros spawn_entity.py -topic /robot_description -entity bigfootbot