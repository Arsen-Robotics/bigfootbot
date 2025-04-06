import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Set the path to different files and folders.
    bfb_description_pkg_share = FindPackageShare(package='bigfootbot_description').find('bigfootbot_description') 
    #bfb_description_launch_dir = os.path.join(bfb_description_dir, 'launch') 

    start_bigfootbot_description_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bfb_description_pkg_share, 'launch', 'view_model.launch.py')),\
        launch_arguments = {'use_joint_state_publisher_gui' : 'True'}.items())

    #start_bigfootbot_description_cmd = IncludeLaunchDescription(
    #  PythonLaunchDescriptionSource(os.path.join(bfb_description_launch_dir, 'view_model.launch.py')))
      #launch_arguments = {'namespace': namespace,
      #                   'use_namespace': use_namespace,
      #                   'slam': slam,
        #                  'map': map_yaml_file,
        #                 'use_sim_time': use_sim_time,
          #                'params_file': params_file,
          #               'default_bt_xml_filename': default_bt_xml_filename,
            #              'autostart': autostart}.items())"""

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add any actions
    ld.add_action(start_bigfootbot_description_cmd)

    return ld