import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    config_dir = os.path.join(get_package_share_directory('controller_package'), 'params')
    param_config = os.path.join(config_dir, "params.yaml")

    controller_node = Node(
        package = 'controller_package',
        executable = 'controller_node',
        output ='screen',
        parameters = [param_config]
    )

    ld.add_action(controller_node)

    return ld