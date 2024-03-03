import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# This is a Python function that returns the path to the package share directory
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_control',
            executable='joy_to_twist_node',
            name='joy_to_twist_node'
        ),
        Node(
            package='motor_control',
            executable='roboclaw_control_node',
            name='roboclaw_control_node'
        ),
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml')])
    ])