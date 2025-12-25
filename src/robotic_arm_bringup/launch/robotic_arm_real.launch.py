#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 声明参数
    robot_model_arg = DeclareLaunchArgument(
        'robot_model_name',
        default_value='arm620',
        description='Robot model name (e.g., arm620, arm380, dual_arm620)'
    )

    # Get package directories
    trajectory_planning_bringup_pkg = get_package_share_directory('trajectory_planning_bringup')

    # 合并后的控制器节点 - 包含controller_manager和trajectory_controller
    # 所有配置都从hardware_config.yaml中的mapping配置读取，不再依赖ROS参数
    arm_controller_node = Node(
        package='arm_controller',
        executable='universial_arm_controller_node',  # 合并后的main函数
        name='arm_controller',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )

    # 包含trajectory planning launch - 只启动robot_description和MoveIt组件
    trajectory_planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(trajectory_planning_bringup_pkg, 'launch', 'trajectory_planning.launch.py')
        ]),
        launch_arguments={
            'robot_model_name': LaunchConfiguration('robot_model_name'),
            'planning_node_type': 'none',  # 不启动额外的规划节点
            'use_hardware_controller': 'false'  # 使用我们的arm_controller管理硬件
        }.items()
    )

    return LaunchDescription([
        # 参数声明
        robot_model_arg,

        # 主控制器节点（合并后的controller_manager + trajectory_controller）
        arm_controller_node,

        # MoveIt和robot_description组件
        trajectory_planning_launch,
    ])
