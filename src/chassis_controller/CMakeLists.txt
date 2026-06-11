cmake_minimum_required(VERSION 3.16)
project(chassis_controller LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(chassis_controller_core SHARED
    src/model/chassis_model.cpp
    src/model/chassis_joint_command.cpp
    src/adapter/chassis_velocity_command_adapter.cpp
    src/kinematics/chassis_kinematics.cpp
    src/config/chassis_controller_config.cpp
    src/controller/chassis_cmd_vel_controller.cpp
)

target_include_directories(chassis_controller_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(chassis_controller_core
    rclcpp
    geometry_msgs
    sensor_msgs
)

target_compile_options(chassis_controller_core PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

add_executable(chassis_controller_node
    src/node/chassis_controller_node.cpp
)

target_link_libraries(chassis_controller_node
    chassis_controller_core
)

ament_target_dependencies(chassis_controller_node
    rclcpp
    geometry_msgs
    sensor_msgs
)

target_compile_options(chassis_controller_node PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

install(
    DIRECTORY include/chassis_controller/
    DESTINATION include/chassis_controller
)

install(
    DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)

install(
    TARGETS chassis_controller_core
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
    TARGETS chassis_controller_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs sensor_msgs)

ament_package()
