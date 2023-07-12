
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
#from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Set the path to different files and folders.
    gazebo_ros_pkg_share = FindPackageShare(package='gazebo_ros').find('gazebo_ros')  
    bigfootbot_gazebo_pkg_share = FindPackageShare(package='bigfootbot_gazebo').find('bigfootbot_gazebo') 
    world_file_name = 'smalltown.world'
    world_path = os.path.join(bigfootbot_gazebo_pkg_share, 'worlds', world_file_name)
    # --- END OF BLOCK 'Set the path to different files and folders'

    # Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'
    spawn_yaw_val = '0.00'

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

    # Spawn the robot
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', 'bigfootbot', 
                    '-topic', 'robot_description',
                        '-x', spawn_x_val,
                        '-y', spawn_y_val,
                        '-z', spawn_z_val,
                        '-Y', spawn_yaw_val],
                        output='screen')

    # --- END OF BLOCK 'Specify the actions' ---


    # --- Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_cmd)

    # Add any actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(spawn_entity_cmd)

    return ld
    # --- END OF BLOCK 'Create the launch description and populate'