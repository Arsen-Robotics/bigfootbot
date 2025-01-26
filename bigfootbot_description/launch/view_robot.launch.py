# --------------------- ATTENTION !!!! ----------------------
# NB! TODO!!!! Launch argument 'model' (path to URDF robot model) is not working (it is ignored)
# and hard coded path is used instead (bigfootbot_description/urdf/bigfootbot_NEW.urdf.xacro)
# check line 189


# This launch file is used to launch the robot_state_publisher and rviz2 nodes 
# for visualizing the robot model in ROS2. It also provides options to start 
# joint_state_publisher and joint_state_publisher_gui nodes. 
#
# The launch file takes in arguments such as the path to the robot model xacro file,
# rviz config file, whether to use simulation time, and whether to start the 
# robot_state_publisher and rviz2 nodes.
#
# To show all available arguments and their descriptions, run the launch file with the -s or --show-args option:
# ros2 launch bigfootbot_description view_robot.launch.py -s

import os
import xacro # xacro is a tool that allows you to process xacro files (xacro files are xml files that contain macros)

#from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command # Command is a substitution that returns the output of a command
                                                              # The Command class is used to execute a command in the shell
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare 

# get_package_share_directory is a function that returns the path to the share directory of a package
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
    default_model_path = os.path.join(bfb_description_pkg_path, 'urdf/bigfootbot_NEW.urdf.xacro') # BIGFOOTBOT 
    #default_model_path = os.path.join(bfb_description_pkg_path, 'urdf/barrelbot.xacro') # BARRELBOT
    
    # get the path to the rviz config file
    default_rviz_config_path = os.path.join(bfb_description_pkg_path, 'rviz/urdf_config.rviz')

    # --- Launch configuration objects 
    # LaunchConfiguration is a substitution that returns a substitution object (Substitution)
    # (Substitution is an object that can be used in a launch file to substitute a value at runtime)
    # 
    # 'model' is the name of the launch configuration variable (meaning that it can be passed to the launch file from the command line 
    # (e.g. 'ros2 launch bigfootbot_description view_model.launch.py model:=/home/user/my_robot.urdf')
    # model_lc is a substitution object that returns the value of the launch configuration variable 'model'
    #model_lc = LaunchConfiguration('model', default=default_model_path)
    model_lc = LaunchConfiguration('model')
    
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
    use_joint_state_pub_gui_lc = LaunchConfiguration('use_joint_state_pub_gui')
    
    # the rviz config file
    rviz_config_file_lc = LaunchConfiguration('rviz_config_file')
    
    # whether to start rviz
    use_rviz_lc = LaunchConfiguration('use_rviz')

    # whether to use sim time (sim time is the time of the simulation, as opposed to the real time)
    use_sim_time_lc = LaunchConfiguration('use_sim_time')
   
    # --- Declare launch arguments (actions)
    # DeclareLaunchArgument is an action that declares a launch argument (it returns an action object)
    # a launch argument is a variable that can be passed to the launch file from the command line
    # (e.g 'ros2 launch bigfootbot_description view_model.launch.py model:=/home/user/my_robot.urdf')
    # 
    # 'model' is the name of the launch argument
    # 'description' is the description of the launch argument (this is the description that 
    #               is displayed when you run the launch file with the --help option)

    declared_arguments = [] # list of launch arguments (actions)
    declared_arguments.append(
        DeclareLaunchArgument(
            'model',
            default_value=default_model_path,
            description='Absolute path to robot urdf file'
        )
    )

    #use_robot_model_pub_la = DeclareLaunchArgument(
    #    'use_robot_model_pub',
    #    default_value='True',
    #    description='Whether to start robot_model_publisher')
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_robot_state_pub',
            default_value='False',
            description='Whether to start robot_state_publisher'
        )
    )
    
    #use_joint_state_publisher_la = DeclareLaunchArgument(
    #    'use_joint_state_publisher',
    #    default_value='True',
    #    description='Whether to start joint_state_publisher')
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_joint_state_pub_gui',
            default_value='False',
            description='Whether to start joint_state_publisher_gui'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file to load'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='False',
            description='Whether to start rviz'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true'
            # Nodes will read time from the /clock topic if use_sim_time is true instead of the system clock
        )
    )
    
    
    # --- Nodes
    # Node is an action that launches a ROS node (it returns an action object)
    # a ROS node is a process that performs computation

    # The robot_state_publisher node publishes the 'robot_description' topic only once when it is started. 
    # The 'robot_description' topic contains the URDF description of the robot, 
    # which is typically loaded from a URDF file (other tools. e.g. rviz subscribe to this topic)
    # When the robot_state_publisher node is launched, it reads the URDF file and extracts 
    # the necessary information, such as joint names, parent-child relationships, joint limits, 
    # and initial joint positions. It then publishes this information as a static 'robot_description topic'.
    # Once the robot_state_publisher has published the robot_description topic, it primarily 
    # focuses on updating and publishing the current state of the robot using joint state 
    # information (from the joint_states topic)
    # The node subscribes to joint state messages, typically published by joint state publisher nodes or 
    # other components that provide the robot's joint values. It uses these joint state messages, 
    # along with the URDF information, to calculate and publish the transformation between different robot links.
    # So the node continuously receives joint state information makes calculations and 
    # publishes these transformations as TF (tf2_msgs/msg/TFMessage) messages.
    # Once the state gets published, it is available to all components in the system 
    # that also use tf2 (tf/tf2 is a tool that allows you to keep track 
    # of the relationship between different coordinate frames)
    # 
    # Published topics: /robot_description (std_msgs/msg/String), 
    # /tf (tf2_msgs/msg/TFMessage), /tf_static (tf2_msgs/msg/TFMessage) 
    # Subscribed topics: joint_states (sensor_msgs/msg/JointState)
    robot_state_publisher_node = Node(
        condition=IfCondition(use_robot_state_pub_lc),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen', # output='screen' will display the output of the node in the terminal
        parameters=[
            {
                'use_sim_time': use_sim_time_lc,
                'robot_description': xacro.process_file(default_model_path).toxml()     # NB! TODO! use model_lc instead of default_model_path
                #'robot_description': xacro.process_file(model_lc).toxml()
                #'robot_description': xacro.process_file(model_lc.perform(None)).toxml()
                #'robot_description': Command(['xacro ', default_model_path])
                #'robot_description': Command(['xacro', ' ', model_lc])
            }
        ]
    )
    
    #joint_state_publisher_node = Node(
    #    package='joint_state_publisher',
    #    executable='joint_state_publisher',
    #    name='joint_state_publisher',
    #    condition=IfCondition(use_joint_state_publisher_lc))
    
    # Package joint_state_publisher_gui is a GUI tool for setting and 
    # publishing joint state values [messages sensor_msgs/JointState to a topic /joint_states] 
    # for a given URDF [reads from a topic /robot_description]) 
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(use_joint_state_pub_gui_lc))
    
    rviz_node = Node(
        condition=IfCondition(use_rviz_lc),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file_lc], # arguments is a list of strings that are passed as command line arguments to the executable 
                                               # -d is the argument that specifies the rviz config file to load
        parameters=[{'use_sim_time': use_sim_time_lc}]
    )
     
    # Nodes to launch
    nodes = [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ]
    
    # `LaunchDescription` serves as a container that holds the various actions (launch arguments) and nodes that you want 
    # to launch when you run a launch file
    return LaunchDescription(declared_arguments + nodes) # `+` is used to concatenate lists

    
    
    






