#!/usr/bin/env python3
"""
重力补偿节点启动文件
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 声明参数
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='',
        description='Path to URDF file'
    )

    # 获取 URDF 文件路径（如果没有指定，使用默认路径）
    urdf_file = LaunchConfiguration('urdf_file')

    return LaunchDescription([
        urdf_file_arg,
        Node(
            package='robot_dynamics',
            executable='gravity_compensator',
            name='gravity_compensator',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['cat ', urdf_file]),
                    value_type=str
                )
            }]
        )
    ])
