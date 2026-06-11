#!/usr/bin/env python3

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def config_package_for(robot_model_name):
    if robot_model_name == "simple_omni_dual_arm":
        return "whole_body_config"
    if robot_model_name == "arm620":
        return "arm620_config"
    if robot_model_name == "arm380":
        return "arm380_config"
    if robot_model_name == "dual_arm620":
        return "dual_arm620_config"
    if robot_model_name == "dual_arm_with_pgc":
        return "dual_arm_with_pgc_config"
    if robot_model_name == "dual_arm_with_omnipicker":
        return "dual_arm_with_omnipicker_config"
    if robot_model_name == "dual_arm_with_omnipicker_humanoid":
        return "dual_arm_with_omnipicker_humanoid_config"
    raise ValueError(f"Unsupported robot model name: {robot_model_name}")


def load_joint_limits_yaml(config_pkg):
    joint_limits_path = os.path.join(
        get_package_share_directory(config_pkg),
        "config",
        "joint_limits.yaml",
    )
    try:
        with open(joint_limits_path, "r", encoding="utf-8") as f:
            joint_limits_data = yaml.safe_load(f)
        if joint_limits_data and "joint_limits" in joint_limits_data:
            return {"joint_limits": joint_limits_data["joint_limits"]}
    except Exception as exc:
        print(f"Failed to load joint_limits.yaml: {exc}")
    return {}


def make_whole_body_moveit_config(robot_model_name, config_pkg):
    return (
        MoveItConfigsBuilder(
            robot_model_name,
            package_name=config_pkg,
        )
        .robot_description()
        .robot_description_semantic()
        .trajectory_execution()
        .planning_pipelines()
        .robot_description_kinematics()
        .joint_limits()
        .to_moveit_configs()
    )


def create_arm_controller_node(context, *args, **kwargs):
    robot_model_name = LaunchConfiguration("robot_model_name").perform(context)
    config_pkg = config_package_for(robot_model_name)
    moveit_config = make_whole_body_moveit_config(robot_model_name, config_pkg)
    joint_limits_params = load_joint_limits_yaml(config_pkg)

    arm_controller_node = Node(
        package="arm_controller",
        executable="universial_arm_controller_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            joint_limits_params,
            {
                "use_sim_time": True,
                "arm_type": robot_model_name,
                "hardware_mode": "gazebo",
                "reactive_task_whole_body.velocity_scaling_factor": 1.0,
                "reactive_task_whole_body.acceleration_scaling_factor": 1.0,
                "reactive_task.feedback.arm_state_source": "joint_states",
                "reactive_task.feedback.command_output": "gazebo",
                "reactive_task.feedback.joint_state_topic": "/joint_states",
                "reactive_task.feedback.left_arm_velocity_command_topic":
                    "/left_arm_velocity_controller/commands",
                "reactive_task.feedback.right_arm_velocity_command_topic":
                    "/right_arm_velocity_controller/commands",
                "reactive_task.feedback.arm_velocity_command_topic":
                    "/arm_velocity_controller/commands",
                "reactive_task.whole_body.enable_mobile_base_in_planning": True,
                "reactive_task.whole_body.enable_mobile_base_in_neo": True,
                "reactive_task.whole_body.mobile_base_type": "omnidirectional",
                "reactive_task.whole_body.mobile_base_state_source": "tf",
                "reactive_task.whole_body.mobile_base_odom_frame": "odom",
                "reactive_task.whole_body.mobile_base_frame": "base_footprint",
                "reactive_task.whole_body.cmd_vel_topic": "/cmd_vel",
            },
        ],
    )

    return [arm_controller_node]


def create_move_group_and_rviz(context, *args, **kwargs):
    robot_model_name = LaunchConfiguration("robot_model_name").perform(context)
    config_pkg = config_package_for(robot_model_name)
    moveit_config = make_whole_body_moveit_config(robot_model_name, config_pkg)

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
        "use_joint_state_topic": True,
        "use_sim_time": True,
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.15,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_configuration,
            trajectory_execution,
        ],
    )

    rviz_config = os.path.join(
        get_package_share_directory(config_pkg),
        "config",
        "moveit.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    return [move_group_node, rviz_node]


def generate_launch_description():
    robot_model_arg = DeclareLaunchArgument(
        "robot_model_name",
        default_value="simple_omni_dual_arm",
        description="Robot model name used by arm_controller/MoveIt.",
    )
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="pillar_obstacles.world",
        description="World file under robot_simulation/worlds.",
    )

    robot_simulation_pkg = get_package_share_directory("robot_simulation")
    camera_driver_pkg = get_package_share_directory("camera_driver")
    camera_driver_config = os.path.join(camera_driver_pkg, "config", "esdf_param_sim.yaml")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                robot_simulation_pkg,
                "launch",
                "simple_omni_base_gazebo.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "start_chassis_controller": "true",
            "world_name": LaunchConfiguration("world_name"),
        }.items(),
    )

    arm_controller_node = OpaqueFunction(function=create_arm_controller_node)
    move_group_and_rviz = OpaqueFunction(function=create_move_group_and_rviz)

    camera_driver_sim_node = Node(
        package="camera_driver",
        executable="camera_driver_sim_node",
        output="screen",
        arguments=[camera_driver_config],
        parameters=[
            {
                "use_sim_time": True,
                "input_pointcloud_topic": "/gazebo_depth_camera/points",
                "world_frame": "odom",
                "camera_frame": "gazebo_depth_camera_optical_frame",
                "min_z_world_m": 0.03,
                "enable_self_filter": True,
            }
        ],
    )

    return LaunchDescription(
        [
            robot_model_arg,
            world_name_arg,
            DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True),
            DeclareBooleanLaunchArg(
                "publish_monitored_planning_scene", default_value=True
            ),
            gazebo_launch,
            camera_driver_sim_node,
            move_group_and_rviz,
            arm_controller_node,
        ]
    )
