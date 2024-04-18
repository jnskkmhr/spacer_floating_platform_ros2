import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ls = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('soacer_floating_platform'),
        'config',
        'vrpn_config.yaml'
        )
    vrpn_node = Node(
            package='vrpn_mocap',
            executable='client_node',
            name='vrpn_client_node',
            output='screen',
            parameters = [config], 
        )
    ls.add_action(vrpn_node)
    return ls