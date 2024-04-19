import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ls = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('floating_platform_control'),
        'config',
        'vrpn_config.yaml'
        )
    ns = LaunchConfiguration('namespace')
    ns_arg = DeclareLaunchArgument(
        'namespace',
        default_value='spacer_floating_platform',
    )
    vrpn_node = Node(
            package='vrpn_mocap',
            namespace=ns, 
            executable='client_node',
            name='vrpn_client_node',
            output='screen',
            parameters = [config], 
        )
    ls.add_action(ns_arg)
    ls.add_action(vrpn_node)
    return ls