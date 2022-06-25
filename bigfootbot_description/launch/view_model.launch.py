import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Set the path to different files and folders.
    bfb_description_pkg_share = FindPackageShare(package='bigfootbot_description').find('bigfootbot_description')
    default_model_path = os.path.join(bfb_description_pkg_share, 'urdf/bigfootbot.urdf')
    #robot_name_in_urdf = 'bigfootbot'
    default_rviz_config_path = os.path.join(bfb_description_pkg_share, 'rviz/urdf_config.rviz')

    # Launch configuration variables
    #model = LaunchConfiguration('model', default=default_model_path)
    model = LaunchConfiguration('model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_joint_state_publisher_gui = LaunchConfiguration('use_joint_state_publisher_gui')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- Declare the launch arguments  
    declare_model_path_cmd = DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path, 
        description='Absolute path to robot urdf file')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')

    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
        name='use_joint_state_publisher_gui',
        default_value='True',
        description='Whether to start joint_state_publisher_gui')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    # --- END OF BLOCK 'Declare the launch arguments' ---

    # --- Specify the actions ---
    # A GUI to manipulate the joint state values
    # Publish the joint state values for the non-fixed joints in the URDF file.
    start_joint_state_publisher_gui_node = Node(
        condition=IfCondition(use_joint_state_publisher_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui')

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        #namespace=namespace,
        parameters=[{'robot_description': Command(['xacro ', model]),
                     'use_sim_time': use_sim_time}]
        #arguments=[default_model_path],
        #remappings=remappings,
        )

    # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])
    # --- END OF BLOCK 'Specify the actions' ---

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_joint_state_publisher_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)  
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd) 

    # Add any actions (start Nodes)
    ld.add_action(start_joint_state_publisher_gui_node)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)

    return ld