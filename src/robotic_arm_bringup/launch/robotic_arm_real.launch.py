#!/usr/bin/env python3

import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_joint_limits_yaml(config_pkg):
    """Directly load joint_limits.yaml to ensure it's available"""
    config_dir = os.path.join(
        get_package_share_directory(config_pkg),
        "config"
    )
    joint_limits_path = os.path.join(config_dir, "joint_limits.yaml")

    try:
        with open(joint_limits_path, 'r') as f:
            joint_limits_data = yaml.safe_load(f)

        if joint_limits_data and "joint_limits" in joint_limits_data:
            return {"joint_limits": joint_limits_data["joint_limits"]}
        return {}
    except Exception as e:
        print(f"Failed to load joint_limits.yaml: {e}")
        return {}


def create_arm_controller_node(context, *args, **kwargs):
    """Create arm_controller node with MoveIt parameters including joint_limits"""
    robot_model_name = LaunchConfiguration('robot_model_name').perform(context)

    # 根据robot_model_name选择正确的MoveIt config包
    if robot_model_name == 'arm620':
        config_pkg = 'arm620_config'
    elif robot_model_name == 'arm380':
        config_pkg = 'arm380_config'
    elif robot_model_name == 'dual_arm620':
        config_pkg = 'dual_arm620_config'
    elif robot_model_name == 'dual_arm_with_pgc':
        config_pkg = 'dual_arm_with_pgc_config'
    elif robot_model_name == 'dual_arm_with_omnipicker':
        config_pkg = 'dual_arm_with_omnipicker_config'
    elif robot_model_name == 'dual_arm_with_omnipicker_humanoid':
        config_pkg = 'dual_arm_with_omnipicker_humanoid_config'
    else:
        raise ValueError(f'Unsupported robot model name: {robot_model_name}')

    # 构建MoveIt配置（包含robot_description_planning中的joint_limits）
    moveit_config = (
        MoveItConfigsBuilder(robot_model_name, package_name=config_pkg)
        .robot_description_semantic()
        .robot_description()
        .trajectory_execution()
        .planning_pipelines()
        .robot_description_kinematics()
        .joint_limits()  # 关键：加载joint_limits.yaml
        .to_moveit_configs()
    )

    # 手动加载joint_limits.yaml以确保参数被传入
    joint_limits_params = load_joint_limits_yaml(config_pkg)

    # 创建arm_controller_node，注入MoveIt参数（包括joint_limits）
    arm_controller_node = Node(
        package='arm_controller',
        executable='universial_arm_controller_node',
        output='screen',
        parameters=[
            moveit_config.to_dict(),  # 注入所有MoveIt参数
            joint_limits_params,      # 显式添加joint_limits参数
            {
                'use_sim_time': False,
                'arm_type': robot_model_name,  # 传入机械臂类型参数
            }
        ]
    )

    return [arm_controller_node]


def generate_launch_description():
    # 声明参数
    robot_model_arg = DeclareLaunchArgument(
        'robot_model_name',
        default_value='dual_arm620',
        description='Robot model name (e.g., arm620, arm380, ' \
        'dual_arm620, dual_arm_with_pgc, dual_arm_with_omnipicker, dual_arm_with_omnipicker_humanoid)'
    )

    # Get package directories
    trajectory_planning_bringup_pkg = get_package_share_directory('trajectory_planning_bringup')

    # 使用OpaqueFunction延迟节点创建，以便能够访问launch context获取robot_model_name
    arm_controller_node = OpaqueFunction(function=create_arm_controller_node)

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
