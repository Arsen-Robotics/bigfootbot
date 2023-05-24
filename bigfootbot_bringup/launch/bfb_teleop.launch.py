import os
import yaml

#import launch_ros.actions
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Get path to config yaml file for RoboClaw
    # Config file is located in the 'config' directory of the 
    # package 'bigfootbot_base'
    my_package_name = 'bigfootbot_base'
    configFilePath = os.path.join(
        get_package_share_directory(my_package_name),
        'config',
        'roboclaw_config.yaml'
    )

    # Extract the relevant configuration parameters from the yaml file.
    # Doing it this way allows you, for example, to include the configuration
    # parameters in a larger yaml file that also provides parameters for
    # other packages. 

    with open(configFilePath, 'r') as file:
        configParams = yaml.safe_load(file)['motor_driver_node']['ros__parameters']

    # ----- Declare nodes -----   

    motor_driver_node = Node(
        package='ros2_roboclaw_driver',
        executable='ros2_roboclaw_driver_node',
        parameters=[configParams],
        output='screen',
        respawn=True, # Whether the node should be automatically restarted if it crashes or exits unexpectedly.
        emulate_tty=True # Helps colorize the log output to the console, providing a better visual experience
    )

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
    ld.add_action(motor_driver_node)
    ld.add_action(joy_node)
    ld.add_action(teleop_node)
    
    return ld
