import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Set the path to different files and folders.
    bfb_teleop_pkg_share = FindPackageShare(package='bigfootbot_teleop').find('bigfootbot_teleop')
    joy_node_params_path = os.path.join(bfb_teleop_pkg_share, 'params/joy.params.yaml')
    teleop_twist_joy_node_params_path = os.path.join(bfb_teleop_pkg_share, 'params/ps3.params.yaml')
    
    # Launch configuration variables
    params_filepath = LaunchConfiguration('params_filepath', default=teleop_twist_joy_node_params_path)


    # Specify the actions
    start_joy_cmd = Node(
        package='joy',
        executable='joy_node',
        #namespace=namespace,
        parameters=[{
            #'dev': joy_dev,
            'device_id': 0,
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }])

    start_teleop_twist_joy_cmd = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        #namespace=namespace,
        parameters=[params_filepath])
        #remappings={('/cmd_vel', launch.substitutions.LaunchConfiguration('joy_vel'))},)
    # --- END OF BLOCK 'Specify the actions'

    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add any actions
    ld.add_action(start_joy_cmd)
    ld.add_action(start_teleop_twist_joy_cmd)

    return ld