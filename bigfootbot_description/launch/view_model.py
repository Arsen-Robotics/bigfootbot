# generate the launch file that launches rviz and the robot model,
# urdf file is located in the bigfootbot_description package in the urdf folder

import os;
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    # get the path to the bigfootbot_description package
    # find('bigfootbot_description') is a function that returns the path to the bigfootbot_description package as a string
    bfb_description_pkg_path = FindPackageShare(package='bigfootbot_description').find('bigfootbot_description')
    # get the path to the urdf file 
    default_model_path = os.path.join(bfb_description_pkg_path, 'urdf/bigfootbot_NEW.urdf.xacro')
    # get the path to the rviz config file
    default_rviz_config_path = os.path.join(bfb_description_pkg_path, 'rviz/urdf_config.rviz')

    # --- Launch configuration variables (these are the variables that can be passed to 
    # the launch file from the command line, 
    # e.g 'ros2 launch bigfootbot_description view_model.launch.py model:=/home/user/my_robot.urdf')) 
    # 
    # LaunchConfiguration is a substitution that returns a substitution object (Substitution)
    # (Substitution is an object that can be used in a launch file to substitute a value at runtime)
    # 'model' is the name of the launch configuration variable
    # model_sub is a substitution object that returns the value of the launch configuration variable 'model'
    model_sub = LaunchConfiguration('model', default=default_model_path)
    # the rviz config file
    rviz_config_file_sub = LaunchConfiguration('rviz_config_file')
    # whether to start joint_state_publisher
    # joint_state_publisher is a tool that allows you to set the joint angles of the robot
    use_joint_state_publisher_sub = LaunchConfiguration('use_joint_state_publisher') 
    # whether to start joint_state_publisher_gui
    # joint_state_publisher_gui is a tool that allows you to set the joint angles of the 
    # robot using a graphical interface
    use_joint_state_publisher_gui_sub = LaunchConfiguration('use_joint_state_publisher_gui')
    # whether to start the robot state publisher
    # robot state publisher is a tool that publishes the state of the robot to tf
    use_robot_state_pub_sub = LaunchConfiguration('use_robot_state_pub')
    # whether to start rviz
    use_rviz_sub = LaunchConfiguration('use_rviz')
    # whether to start the robot model publisher
    # robot model publisher is a tool that publishes the robot model to tf
    use_robot_model_pub_sub = LaunchConfiguration('use_robot_model_pub')

    # LaunchConfiguration vs DeclareLaunchArgument
    # LaunchConfiguration is a substitution that returns a substitution object (Substitution)
    # (Substitution is an object that can be used in a launch file to substitute a value at runtime)
    # DeclareLaunchArgument is an action that declares a launch argument
    # a launch argument is a variable that can be passed to the launch file from the command line
   
    # --- Declare launch arguments
    # DeclareLaunchArgument is an action that declares a launch argument (it returns an action object)
    # a launch argument is a variable that can be passed to the launch file from the command line
    # 'model' is the name of the launch argument
    # 'description' is the description of the launch argument (this is the description that 
    # is displayed when you run the launch file with the --help option)
    declare_model_path_act = DeclareLaunchArgument(
        'model',
        default_value=default_model_path,
        description='Absolute path to robot urdf file')
    
    declare_rviz_config_file_act = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file to load')
    
    # how is better to name an action object?
    # how action object is named in python code?
    # 





