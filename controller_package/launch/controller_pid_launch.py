import os
import pathlib
import yaml
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # params = os.path.join(
    #     get_package_share_directory('bluerov2_teleop'),
    #     'params',
    #     'params.yaml'
    # )

    config_dir = os.path.join(get_package_share_directory('controller_package'), 'params')
    param_config = os.path.join(config_dir, "params.yaml")
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)["controller_node"]["ros__parameters"]


    controller_node = Node(
        package = 'controller_package',
        executable = 'controller_pid_exe',
        output ='screen',
        parameters = [params]
    )


    ld.add_action(controller_node)

    return ld