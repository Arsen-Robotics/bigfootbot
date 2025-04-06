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
        emulate_tty=True,
        parameters=[os.path.join(get_package_share_directory('bigfootbot_teleop'), 'config', 'joy.yaml')]
    )

    # This node subscribes to the 'joy' topic containing joystick data and 
    # republishes it as a 'Twist' message
    # Source: https://github.com/ros2/teleop_twist_joy
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        respawn=True,
        emulate_tty=True,
        parameters=[os.path.join(get_package_share_directory('bigfootbot_teleop'), 'config', 'teleop_twist_joy_ps3.yaml')],
        remappings=[
            ('/cmd_vel', '/cmd_vel_unstamped'),
        ],
    )

    # This node subscribes to the 'cmd_vel_unstamped' topic, converts geometry_msgs/msg/Twist to
    # geometry_msgs/msg/TwistStamped and republishes it as a 'cmd_vel' topic
    twist_stamper_node = Node(
        package='bigfootbot_teleop',
        executable='twist_stamper.py',
        name='twist_stamper',
        output='screen',
        respawn=True, # Restart the node if it crashes
        remappings=[
            ('/cmd_vel', '/diff_cont/cmd_vel'), # diff_drive_controller subscribes to the topic '/diff_cont/cmd_vel'
        ]
    )

    # Create launch description and add all nodes
    ld = LaunchDescription()
    ld.add_action(joy_node)
    ld.add_action(teleop_twist_joy_node)
    ld.add_action(twist_stamper_node)
    
    return ld
