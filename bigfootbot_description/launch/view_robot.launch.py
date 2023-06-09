# Description: Launches rviz and loads the robot model urdf file

import os;
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare 

#q: get_package_share_directory vs FindPackageShare? 
# a: get_package_share_directory is a function that returns the path to the share directory of a package
# FindPackageShare is a substitution that returns the path to the share directory of a package
# the difference is that get_package_share_directory is a function that returns a string, while FindPackageShare 
# is a substitution that returns a substitution object
# a substitution object is an object that can be used in a launch file to substitute a value at runtime

# loads robot model urdf file and launches rviz
def generate_launch_description():  
    # --- File path declarations
    # get the path to the bigfootbot_description package
    # find('bigfootbot_description') is a function that returns the path to the bigfootbot_description package as a string
    bfb_description_pkg_path = FindPackageShare(package='bigfootbot_description').find('bigfootbot_description')
    
    # get the path to the urdf file 
    default_model_path = os.path.join(bfb_description_pkg_path, 'urdf/bigfootbot_NEW.urdf.xacro')
    
    # get the path to the rviz config file
    default_rviz_config_path = os.path.join(bfb_description_pkg_path, 'rviz/urdf_config.rviz')

    # --- Launch configuration objects 
    # LaunchConfiguration is a substitution that returns a substitution object (Substitution)
    # (Substitution is an object that can be used in a launch file to substitute a value at runtime)
    # 
    # 'model' is the name of the launch configuration variable
    # model_lc is a substitution object that returns the value of the launch configuration variable 'model'
    model_lc = LaunchConfiguration('model', default=default_model_path)
    
    # whether to start the robot model publisher
    # robot model publisher is a tool that publishes the robot model to tf
    #use_robot_model_pub_lc = LaunchConfiguration('use_robot_model_pub')
    
    # whether to start the robot state publisher
    # robot state publisher is a tool that publishes the state of the robot to tf
    use_robot_state_pub_lc = LaunchConfiguration('use_robot_state_pub')
    
    # whether to start joint_state_publisher
    # joint_state_publisher is a tool that allows you to set the joint angles of the robot
    #use_joint_state_publisher_lc = LaunchConfiguration('use_joint_state_publisher') 
    
    # whether to start joint_state_publisher_gui
    # joint_state_publisher_gui is a tool that allows you to set the joint angles of the 
    # robot using a graphical interface
    #use_joint_state_publisher_gui_lc = LaunchConfiguration('use_joint_state_publisher_gui')
    
    # the rviz config file
    rviz_config_file_lc = LaunchConfiguration('rviz_config_file')
    
    # whether to start rviz
    use_rviz_lc = LaunchConfiguration('use_rviz')
   
    # --- Declare launch arguments (actions)
    # DeclareLaunchArgument is an action that declares a launch argument (it returns an action object)
    # a launch argument is a variable that can be passed to the launch file from the command line
    # (e.g 'ros2 launch bigfootbot_description view_model.launch.py model:=/home/user/my_robot.urdf')
    # 
    # 'model' is the name of the launch argument
    # 'description' is the description of the launch argument (this is the description that 
    #               is displayed when you run the launch file with the --help option)
    model_path_la = DeclareLaunchArgument(
        'model',
        default_value=default_model_path,
        description='Absolute path to robot urdf file')

    #use_robot_model_pub_la = DeclareLaunchArgument(
    #    'use_robot_model_pub',
    #    default_value='True',
    #    description='Whether to start robot_model_publisher')
    
    use_robot_state_pub_la = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start robot_state_publisher')
    
    #use_joint_state_publisher_la = DeclareLaunchArgument(
    #    'use_joint_state_publisher',
    #    default_value='True',
    #    description='Whether to start joint_state_publisher')
    
    #use_joint_state_publisher_gui_la = DeclareLaunchArgument(
    #    'use_joint_state_publisher_gui',
    #    default_value='True',
    #    description='Whether to start joint_state_publisher_gui')
    
    rviz_config_file_la = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file to load')
    
    use_rviz_la = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start rviz')
    
    
    # --- Nodes
    # Node is an action that launches a ROS node (it returns an action object)
    # a ROS node is a process that performs computation
    # condition is a substitution that returns a boolean value
    #robot_model_publisher_node = Node(
    #    package='robot_model_publisher',
    #    executable='robot_model_publisher',
    #    name='robot_model_publisher',
    #    condition=IfCondition(use_robot_model_pub_lc), # if use_robot_model_pub_lc is True, then launch the node
    #    parameters=[{'use_sim_time': False}],
    #    arguments=[model_lc]) # pass the value of the launch configuration variable 'model' to the node 

    # robot_state_publisher publishes the state of the robot to tf2
    # Once the state gets published, it is available to all components in the system 
    # that also use tf2 (tf/tf2 is a tool that allows you to keep track 
    # of the relationship between different coordinate frames)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        condition=IfCondition(use_robot_state_pub_lc),
        parameters=[{'use_sim_time': False}],
        arguments=[model_lc])
    
    #joint_state_publisher_node = Node(
    #    package='joint_state_publisher',
    #    executable='joint_state_publisher',
    #    name='joint_state_publisher',
    #    condition=IfCondition(use_joint_state_publisher_lc))
    
    #joint_state_publisher_gui_node = Node(
    #    package='joint_state_publisher_gui',
    #    executable='joint_state_publisher_gui',
    #    name='joint_state_publisher_gui',
    #    condition=IfCondition(use_joint_state_publisher_gui_lc))
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(use_rviz_lc),
        arguments=['-d', rviz_config_file_lc]) # -d is the argument that specifies the rviz config file to load
    
    # --- Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions to the launch description
    ld.add_action(model_path_la)
    #ld.add_action(use_robot_model_pub_la)
    ld.add_action(use_robot_state_pub_la)
    #ld.add_action(use_joint_state_publisher_la)
    #ld.add_action(use_joint_state_publisher_gui_la)
    ld.add_action(rviz_config_file_la)
    ld.add_action(use_rviz_la)

    #ld.add_action(robot_model_publisher_node)
    ld.add_action(robot_state_publisher_node)
    #ld.add_action(joint_state_publisher_node)
    #ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz_node)

    return ld

    
    
    






