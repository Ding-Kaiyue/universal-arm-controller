import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("whole_body_controller")
    config_path = os.path.join(pkg_share, "config", "whole_body_controller.yaml")

    return LaunchDescription(
        [
            Node(
                package="whole_body_controller",
                executable="whole_body_controller_node",
                parameters=[config_path],
                output="screen",
            )
        ]
    )
