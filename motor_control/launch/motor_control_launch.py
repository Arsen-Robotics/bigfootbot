from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
#from launch.launch_description_sources import PythonLaunchDescriptionSource
#from launch_yaml.launch_description_sources import YAMLLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # foxglove_bridge_launch_file = IncludeLaunchDescription(
    #     XMLLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('foxglove_bridge'),
    #                      'launch/foxglove_bridge_launch.xml')
    #     )
    # )

    joy_to_twist_node = Node(
        package = 'motor_control',
        executable = 'joy_to_twist_node'
    )

    roboclaw_control_node = Node(
        package = 'motor_control',
        executable = 'roboclaw_control_node'
    )

    # ld.add_action(foxglove_bridge_launch_file)
    ld.add_action(joy_to_twist_node)
    ld.add_action(roboclaw_control_node)

    return ld