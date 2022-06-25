
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
#from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Set the path to different files and folders.
    gazebo_ros_pkg_share = FindPackageShare(package='gazebo_ros').find('gazebo_ros')  
    bigfootbot_gazebo_pkg_share = FindPackageShare(package='bigfootbot_gazebo').find('bigfootbot_gazebo') 
    world_file_name = 'smalltown.world'
    world_path = os.path.join(bigfootbot_gazebo_pkg_share, 'worlds', world_file_name)
    # --- END OF BLOCK 'Set the path to different files and folders'

    # Launch configuration variables
    world = LaunchConfiguration('world')
    # --- END OF BLOCK 'Launch configuration variables'


    # --- Declare the launch arguments 
    declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')
    # --- END OF BLOCK 'Declare the launch arguments '


    # --- Specify the actions

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg_share, 'launch', 'gzserver.launch.py')),
        #condition=IfCondition(use_simulator),
        launch_arguments={'world': world}.items())

    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg_share, 'launch', 'gzclient.launch.py')))
        #condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
    # --- END OF BLOCK 'Specify the actions' ---


    # --- Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_cmd)

    # Add any actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    return ld
    # --- END OF BLOCK 'Create the launch description and populate'