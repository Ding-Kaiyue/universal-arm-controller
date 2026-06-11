import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_controller_action(controller_name):
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            controller_name,
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "10",
        ],
        output="screen",
    )


def launch_setup(context, *args, **kwargs):
    package_name = "robot_simulation"
    entity_name = LaunchConfiguration("entity_name").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    start_chassis_controller = (
        LaunchConfiguration("start_chassis_controller").perform(context).lower() == "true"
    )
    world_name = LaunchConfiguration("world_name").perform(context)

    pkg_share = get_package_share_directory(package_name)
    robot_xacro_path = os.path.join(pkg_share, "urdf", "simple_omni_dual_arm.gazebo.urdf.xacro")
    world_path = os.path.join(pkg_share, "worlds", world_name)
    model_database_uri = "file://" + os.path.join(pkg_share, "models")
    robot_description_share = get_package_share_directory("robot_description")

    doc = xacro.process_file(robot_xacro_path)
    robot_description_xml = doc.toxml().replace(
        "package://robot_description/",
        "file://" + robot_description_share + "/",
    )
    robot_description = {"robot_description": robot_description_xml}

    gazebo = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            world_path,
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
        ],
        additional_env={
            "GAZEBO_MODEL_DATABASE_URI": model_database_uri,
            "GAZEBO_MODEL_PATH": os.path.join(pkg_share, "models"),
        },
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": use_sim_time}, robot_description],
        output="screen",
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", entity_name],
        output="screen",
    )

    load_joint_state_broadcaster = load_controller_action("joint_state_broadcaster")
    load_wheel_controller = load_controller_action("chassis_wheel_velocity_controller")
    load_left_arm_controller = load_controller_action("left_arm_velocity_controller")
    load_right_arm_controller = load_controller_action("right_arm_velocity_controller")

    bridge = Node(
        package="robot_simulation",
        executable="joint_state_velocity_bridge.py",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": "chassis_joint_commands",
                "output_topic": "/chassis_wheel_velocity_controller/commands",
            }
        ],
        output="screen",
    )

    model_tf_bridge = Node(
        package="robot_simulation",
        executable="gazebo_model_tf_bridge.py",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "model_name": entity_name,
                "world_frame": "odom",
                "base_frame": "base_footprint",
                "model_states_topic": "/gazebo/model_states",
                "project_to_planar": True,
                "base_z": 0.0,
            }
        ],
        output="screen",
    )

    actions = [
        gazebo,
        robot_state_publisher,
        spawn_entity,
        model_tf_bridge,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    TimerAction(
                        period=1.0,
                        actions=[load_joint_state_broadcaster],
                    )
                ],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[
                    load_wheel_controller,
                    load_left_arm_controller,
                    load_right_arm_controller,
                    bridge,
                ],
            )
        ),
    ]

    if start_chassis_controller:
        actions.append(
            Node(
                package="chassis_controller",
                executable="chassis_controller_node",
                parameters=[
                    os.path.join(
                        get_package_share_directory("chassis_controller"),
                        "config",
                        "chassis_controller.yaml",
                    ),
                    {"use_sim_time": use_sim_time},
                ],
                output="screen",
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "entity_name",
                default_value="simple_omni_dual_arm",
                description="Gazebo entity name.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation clock.",
            ),
            DeclareLaunchArgument(
                "start_chassis_controller",
                default_value="true",
                description="Start chassis_controller together with Gazebo.",
            ),
            DeclareLaunchArgument(
                "world_name",
                default_value="pillar_obstacles.world",
                description="World file under robot_simulation/worlds.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
