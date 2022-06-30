import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Set the path to different files and folders.
    bfb_navigation_pkg_share = FindPackageShare(package='bigfootbot_navigation').find('bigfootbot_navigation')
    nav2_bringup_pkg_share = FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
    nav2_launch_dir = os.path.join(nav2_bringup_pkg_share, 'launch')

    # NB! It seems that this argument is not needed anymore in the new verson of bt_navigator CHECK! 
    nav2_bt_navigator_pkg_share = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')

    default_map_yaml_file = os.path.join(bfb_navigation_pkg_share, 'maps', 'smalltown_world.yaml')
    nav2_params_file = os.path.join(bfb_navigation_pkg_share, 'params', 'nav2_params.yaml')    
    robot_localization_file = os.path.join(bfb_navigation_pkg_share, 'params/ekf.yaml') 
    default_rviz_config_file = os.path.join(bfb_navigation_pkg_share, 'rviz/nav2_config.rviz')

    # NB! It seems that this argument is not needed anymore in the new verson of bt_navigator CHECK! 
    default_behavior_tree_xml_path = os.path.join(nav2_bt_navigator_pkg_share, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    # Launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    autostart = LaunchConfiguration('autostart') # Parameter for nav2_lifecycle_manager
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename') # NB! It seems that this argument is not needed anymore in the new verson of bt_navigator CHECK! 
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        name='use_namespace',
        default_value='False',
        description='Whether to apply a namespace to the navigation stack')

    declare_autostart_cmd = DeclareLaunchArgument(
        name='autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack')

    # NB! It seems that this argument is not needed anymore in the new verson of bt_navigator CHECK! 
    declare_bt_xml_cmd = DeclareLaunchArgument(  
        name='default_bt_xml_filename',
        default_value=default_behavior_tree_xml_path,
        description='Full path to the behavior tree xml file to use')

    declare_map_yaml_file_cmd = DeclareLaunchArgument(
        name='map',
        default_value=default_map_yaml_file,
        description='Full path to map file to load')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        name='params_file',
        default_value=nav2_params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_slam_cmd = DeclareLaunchArgument(
        name='slam',
        default_value='False',
        description='Whether to run SLAM')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_file,
        description='Full path to the RVIZ config file to use')

    # --- Specify the actions

    # Start robot localization using an Extended Kalman filter
    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file, 
        {'use_sim_time': use_sim_time}])

    # Launch the ROS 2 Navigation Stack
    start_ros2_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
            launch_arguments = {'namespace': namespace,
                                'use_namespace': use_namespace,
                                'slam': slam,
                                'map': map_yaml_file,
                                'use_sim_time': use_sim_time,
                                'params_file': params_file,
                                'default_bt_xml_filename': default_bt_xml_filename,  # NB! It seems there is no such argument. CHECK! 
                                'autostart': autostart}.items())

    # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]) 


    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_map_yaml_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_use_rviz_cmd) 
    ld.add_action(declare_rviz_config_file_cmd)
    
    # Add any actions
    ld.add_action(start_robot_localization_cmd)
    ld.add_action(start_ros2_navigation_cmd)
    ld.add_action(start_rviz_cmd)

    return ld