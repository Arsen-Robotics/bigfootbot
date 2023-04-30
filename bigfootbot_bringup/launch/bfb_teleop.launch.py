import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():

    # Get path to config yaml file for RoboClaw
    # Config file is located in the package "bigfootbot_base"
    my_package_name = 'bigfootbot_base'
    configFilePath = os.path.join(
        get_package_share_directory(my_package_name),
        'config',
        'roboclaw_config.yaml'
    )

    # Extract the relevant configuration parameters from the yaml file.
    # Doing it this way allows you, for example, to include the configuration
    # parameters in a larger yaml file that also provides parameters for
    # other packages. See the example yaml file provided and the README
    # file for more information.

    with open(configFilePath, 'r') as file:
        configParams = yaml.safe_load(file)['motor_driver_node']['ros__parameters']   


    # Declare nodes
    node1 = launch_ros.actions.Node(
        package='my_package',
        node_executable='my_node',
        name='node_1',
        output='screen'
    )
    node2 = launch_ros.actions.Node(
        package='my_package',
        node_executable='my_other_node',
        name='node_2',
        output='screen',
        parameters=[{
            'my_parameter': 'my_value'
        }]
    )

    # Create launch description and add nodes
    ld = launch.LaunchDescription()
    ld.add_action(node1)
    ld.add_action(node2)
    
    return ld
