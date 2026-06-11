from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare("chassis_controller"),
        "config",
        "chassis_controller.yaml",
    ])

    return LaunchDescription([
        Node(
            package="chassis_controller",
            executable="chassis_controller_node",
            name="chassis_controller_node",
            output="screen",
            parameters=[config],
        )
    ])
