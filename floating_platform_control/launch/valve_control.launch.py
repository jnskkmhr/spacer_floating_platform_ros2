import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ls = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('soacer_floating_platform'),
        'config',
        'fp_config.yaml'
        )
    fp_node = Node(
            package='spacer_floating_platform',
            executable='FloatingPlatformDirectValveControl',
            name='FloatingPlatformDirectValveControl',
            output='screen',
            parameters = [config], 
        )
    ls.add_action(fp_node)
    return ls