#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
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

    # SDK节点 - 提供示教/复现等高级功能，支持按键控制
    robot_sdk_node = Node(
        package='robot_sdk',
        executable='robot_sdk_node',
        name='robot_sdk',
        output='screen',
    )

    # 包含trajectory planning launch - 只启动robot_description和MoveIt组件
    trajectory_planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(trajectory_planning_bringup_pkg, 'launch', 'trajectory_planning.launch.py')
        ]),
        launch_arguments={
            'planning_node_type': 'none',  # 不启动额外的规划节点
            'use_hardware_controller': 'false'  # 使用我们的arm_controller管理硬件
        }.items()
    )

    return LaunchDescription([
        # 主控制器节点（合并后的controller_manager + trajectory_controller）
        arm_controller_node,

        # SDK节点（示教/复现/按键控制）
        robot_sdk_node,

        # MoveIt和robot_description组件
        trajectory_planning_launch,
    ])