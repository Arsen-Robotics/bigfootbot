# Made based on the following file:
# https://github.com/joshnewans/articubot_one/blob/humble/launch/launch_sim.launch.py

import os

# This is a Python function that returns the path to the package share directory
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    #package_name='articubot_one'

    # Launch the sim with and empty world
    # Include the Gazebo launch file, provided by the ros_gz_sim package
    # IncludeLaunchDescription class is used to include a launch file in another launch file
    # PythonLaunchDescriptionSource class is used to specify the source of a launch file to be 
    # included in a launch file.
    gazebo_empty = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
            launch_arguments={'gz_args': 'empty.sdf'}.items()
        )

    # Spawn the robot in the empty world
    # Create a node that runs the ROS executable ("ros2 run ros_gz_sim create")
    # This node will terminate after spawning the robot
    # 'empty' is the name of the world (inside the empty.sdf - <world name="empty">)
    # NB! robot_state_publisher must be running before this node is started
    # becasue it will look for the robot URDF in the topic /robot_description
    create_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_custom_model',
            '-world', 'empty', 
            '-topic', '/robot_description'],
        output='screen'
    )

    # ROS2 node that bridges messages between ROS2 and Gazebo.
    # Overall, this code is defining a ROS2 node that bridges messages between ROS2 and Gazebo. 
    # It allows the cmd_vel topic to be used to control the motion of the robot in Gazebo, 
    # and allows the odometry topic to be used to track the position and orientation of the 
    # robot in ROS2.
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/barrelbot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/model/barrelbot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        parameters=[{'qos_overrides./model/barrelbot.subscriber.reliability': 'reliable',
                     'qos_overrides./model/barrelbot.subscriber.reliability': 'reliable'}],
        output='screen'
    )

    return LaunchDescription([
        gazebo_empty,
        create_node,
        bridge_node
    ])
