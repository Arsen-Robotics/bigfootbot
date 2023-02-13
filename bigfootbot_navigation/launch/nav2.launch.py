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
    robot_localization_file = os.path.join(bfb_navigation_pkg_share, 'params/ekf_with_gps.yaml') 
    default_rviz_config_file = os.path.join(bfb_navigation_pkg_share, 'rviz/nav2_config.rviz')

    # NB! It seems that this argument is not needed anymore in the new verson of bt_navigator CHECK! 
    default_behavior_tree_xml_path = os.path.join(nav2_bt_navigator_pkg_share, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    # Launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    autostart = LaunchConfiguration('autostart') # Parameter for nav2_lifecycle_manager
    bt_xml_filename = LaunchConfiguration('bt_xml_filename') # NB! It seems that this argument is not needed anymore in the new verson of bt_navigator CHECK! 
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
        default_value='True',
        description='Automatically startup the nav2 stack')

    # NB! It seems that this argument is not needed anymore in the new verson of bt_navigator CHECK! 
    declare_bt_xml_cmd = DeclareLaunchArgument(  
        name='bt_xml_filename',
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

    # Start the navsat transform node which converts GPS data into the world coordinate frame
    # Converts GPS data into Odometry data
    # Also publishes transform UTM->map (static]) - transform is optional
    # Doc: http://docs.ros.org/en/melodic/api/robot_localization/html/navsat_transform_node.html
    start_navsat_transform_cmd = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[robot_localization_file, 
        {'use_sim_time': use_sim_time}],
        remappings=[('imu', 'imu/data'),     # subscribed topic - a sensor_msgs/Imu message with orientation data (published by IMU sensor)
                                             # frame_id: imu_link
                    ('gps/fix', 'gps/fix'),  # subscribed topic - a sensor_msgs/NavSatFix message containing your robot’s GPS coordinates 
                                             # (published by GPS sensor)
                                             # frame_id: base_footprint
                    ('gps/filtered', 'gps/filtered'), # published topic (optional) - a sensor_msgs/NavSatFix 
                                                      # message containing your robot’s world frame (map) position, transformed into GPS coordinates
                                                      # If you want it to publish improved GPS data by odometry and IMU
                                                      # (GPS data after we have processed the raw GPS data through an Extended Kalman Filter)
                                                      # TODO! NOTE! ros2 topic echo /gps/filtered shows frame_id: base_footprint, not map!
                    ('odometry/gps', 'odometry/gps'), # published topic - a nav_msgs/Odometry message containing the GPS coordinates of your robot, 
                                                      # transformed into its world coordinate frame (map). This message can be directly fused into 
                                                      # robot_localization’s state estimation nodes (ekf_filter_node_map in our case).
                                                      # The pose of the robot in the map frame BEFORE data processing through an Extended Kalman Filter.
                                                      # frame_id: map, child_frame_id: '' (Twist is not published [all zeros])
                                                      # NOTE! robot's world frame (frame_id: map) is loaded from the message odometry/global
                    ('odometry/filtered', 'odometry/global')]) # subscribed topic - a nav_msgs/Odometry message of your robot’s current position. 
                                                               # This is needed in the event that your first GPS reading comes after your robot 
                                                               # has attained some non-zero pose.
                                                               # ododmetry/global is published by ekf_node 'ekf_filter_node_map'
                                                               # frame_id: map, child_frame_id: base_footprint

    # Start robot localization using an Extended Kalman filter
    """start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file, 
        {'use_sim_time': use_sim_time}],
        remappings=[('/set_pose', '/initialpose')])"""

    # Start robot localization using an Extended Kalman filter
    # Publishes transform map->odom
    start_robot_localization_global_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[robot_localization_file, 
        {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/global'), # published topic (nav_msgs/Odometry)
                                                              # The pose of the robot (frame base_footprint) in the  world_frame (map) after data processing 
                                                              # through an Extended Kalman Filter. 
                                                              # This value is a combination of wheel encoder information, IMU data, and GPS data.
                                                              # frame_id: map, child_frame_id: base_footprint NB! Twist is also published in the message
                    ('/set_pose', '/initialpose')]) # subscribed topic. By issuing a geometry_msgs/PoseWithCovarianceStamped message to the set_pose topic, 
                                                    # users can manually set the state of the filter. This is useful for resetting the filter 
                                                    # during testing, and allows for interaction with rviz.
                                                    # Alternatively, the state estimation nodes advertise a SetPose service, whose type is 
                                                    # robot_localization/SetPose.

    # Start robot localization using an Extended Kalman filter
    # Publishes transform odom->base_footprint
    start_robot_localization_local_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[robot_localization_file, 
        {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/local'), # published topic (nav_msgs/Odometry) 
                                                             # The pose of the robot (frame base_footprint) in the world_frame (odom) 
                                                             # after data processing through an Extended Kalman Filter.
                                                             # This value is a combination of wheel encoder information and IMU data 
                                                             # frame_id: odom, child_frame_id: base_footprint NB! Twist is also published in the message
                    ('/set_pose', '/initialpose')]) # subscribed topic

    # Launch the ROS 2 Navigation Stack
    start_ros2_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
            launch_arguments = {'namespace': namespace,
                                'use_namespace': use_namespace,
                                'slam': slam,
                                'map': map_yaml_file,
                                'use_sim_time': use_sim_time,
                                'params_file': params_file,
                                'default_bt_xml_filename': bt_xml_filename,  # NB! It seems there is no such argument 
                                                                              # in nav2_bringup/launch/bringup_launch.py. CHECK! 
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
    ld.add_action(declare_bt_xml_cmd) # Tt seems this is not needed anymore in the new verson of bt_navigator CHECK
    ld.add_action(declare_map_yaml_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_use_rviz_cmd) 
    ld.add_action(declare_rviz_config_file_cmd)
    
    # Add any actions
    ld.add_action(start_navsat_transform_cmd)
    #ld.add_action(start_robot_localization_cmd)
    ld.add_action(start_robot_localization_global_cmd)
    ld.add_action(start_robot_localization_local_cmd)
    #ld.add_action(start_ros2_navigation_cmd) 
    ld.add_action(start_rviz_cmd)

    return ld