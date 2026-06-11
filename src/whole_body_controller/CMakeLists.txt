cmake_minimum_required(VERSION 3.16)
project(whole_body_controller LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(whole_body_controller_core SHARED
    src/controller/whole_body_coordinator.cpp
)

target_include_directories(whole_body_controller_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(whole_body_controller_core
    rclcpp
    geometry_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
)

target_compile_options(whole_body_controller_core PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

add_executable(whole_body_controller_node
    src/node/whole_body_controller_node.cpp
)

target_link_libraries(whole_body_controller_node
    whole_body_controller_core
)

ament_target_dependencies(whole_body_controller_node
    rclcpp
)

target_compile_options(whole_body_controller_node PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

install(
    DIRECTORY include/whole_body_controller/
    DESTINATION include/whole_body_controller
)

install(
    DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)

install(
    TARGETS whole_body_controller_core
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
    TARGETS whole_body_controller_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    rclcpp
    geometry_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
)

ament_package()
