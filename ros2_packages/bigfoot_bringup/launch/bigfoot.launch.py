import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from launch_ros.actions import Node

# Note from ROS2 official docs (https://docs.ros.org/en/foxy/Tutorials/Launch/Creating-Launch-Files.html)
# For packages with launch files, it is a good idea to add an exec_depend dependency on the ros2launch package in your package’s package.xml:
# <exec_depend>ros2launch</exec_depend>


def generate_launch_description():
   ############ ROBOCLAW NODE ###########
   roboclaw_ld = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('ros2_roboclaw_driver'), 'launch'),
         '/ros2_roboclaw_driver.launch.py'])
      )

   ############ TELEOP KEYBOARD NODE ###########
   teleop_twist_keyboard_ld = LaunchDescription()
   teleop_twist_keyboard_node = Node(
      package='teleop_twist_keyboard',
      executable='teleop_twist_keyboard',
      name='teleop_MY',
      output='screen')
   teleop_twist_keyboard_ld.add_action(teleop_twist_keyboard_node)

   
   return LaunchDescription([
      roboclaw_ld,
      #teleop_twist_keyboard_ld
   ])