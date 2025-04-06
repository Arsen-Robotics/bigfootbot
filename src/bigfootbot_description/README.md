model.config - this is a required configuration file for Gazebo to find this model in its database

To accomplish this method, you must make a new model database that contains just your single robot. This isn't the cleanest way to load your URDF into Gazebo but accomplishes the goal of not having to keep two copies of your robot URDF on your computer

Current solution for reading URDF models in Gazebo assumes the ROS package containing the model must also have a valid Gazebo model.config file in addition to the ROS manifest.xml file. And the model must be in both GAZEBO_MODEL_PATH and ROS_PACKAGE_PATH for both ROS and Gazebo to find the model.

With that assumption, package:// is simply replaced by model:// by parser_urdf.cc.

# ----- Add to .bashrc:
# Workaround for URDF and RVIZ2: cylinder not showing
export LC_NUMERIC="en_US.UTF-8"

<geometry> 
 <cylinder radius="0.0508" length="0.18"/>\
</geometry>