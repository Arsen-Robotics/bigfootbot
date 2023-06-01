import os
import yaml

#import launch_ros.actions
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # ----- Declare nodes -----   

    # Declare the 'joy_node' from 'joy_package'
    # Node that interfaces a generic joystick to ROS 2
    # This node publishes a 'sensor_msgs/msg/Joy' message, which contains the current state 
    # of a joystick buttons and axes
    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen',
        respawn=True,
        emulate_tty=True
    )

    # Declare 'teleop_node' from 'teleop_twist_joy package'
    # Node that republishes sensor_msgs/msg/Joy messages 
    # as scaled geometry_msgs/msg/Twist messages
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        output='screen',
        respawn=True,
        emulate_tty=True
    )

    # Create launch description and add nodes
    ld = LaunchDescription()
    ld.add_action(joy_node)
    ld.add_action(teleop_node)
    
    return ld
