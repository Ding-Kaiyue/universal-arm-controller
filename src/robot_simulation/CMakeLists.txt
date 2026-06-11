cmake_minimum_required(VERSION 3.8)
project(robot_simulation)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY config launch models urdf worlds
    DESTINATION share/${PROJECT_NAME}
)

install(
    PROGRAMS
        scripts/gazebo_model_tf_bridge.py
        scripts/joint_state_velocity_bridge.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
