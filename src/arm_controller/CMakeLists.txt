cmake_minimum_required(VERSION 3.8)
project(arm_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Ensure option() honors normal variables behavior consistently
if(POLICY CMP0077)
  cmake_policy(SET CMP0077 NEW)
endif()

# Build options
option(CMAKE_INTERPROCEDURAL_OPTIMIZATION "Enable LTO for optimization" OFF)
option(BUILD_PYTHON_IPC_BINDINGS "Build Python IPC producer bindings (pybind11)" ON)
option(ARM_CONTROLLER_BUILD_MOTION_CONTROLLERS "Build motion controllers: MoveJ/MoveL/MoveC/ReactiveTask" ON)
option(ARM_CONTROLLER_BUILD_VELOCITY_CONTROLLERS "Build velocity controllers and velocity solver support" ON)
option(ARM_CONTROLLER_BUILD_TEACH_CONTROLLERS "Build teach/record/replay controllers" ON)

# Memory-efficient linker settings for low-memory systems
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-keep-memory")
set(CMAKE_EXE_LINKER_FLAGS    "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-keep-memory")

# Use lld linker if available (reduces link-time memory by 30-50%)
find_program(LLD_LINKER ld.lld)
if(LLD_LINKER)
  message(STATUS "Found lld linker: ${LLD_LINKER}")
  set(CMAKE_LINKER ${LLD_LINKER})
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fuse-ld=lld")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fuse-ld=lld")
else()
  message(STATUS "lld linker not found, using system default")
  message(STATUS "To install lld: sudo apt install lld")
endif()

# 默认禁用测试
if(NOT DEFINED BUILD_TESTING)
  set(BUILD_TESTING OFF)
endif()

enable_testing()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-subobject-linkage)
endif()

# 覆盖率编译选项（仅在构建测试时启用）
if(BUILD_TESTING)
  if(CMAKE_COMPILER_IS_GNUCXX)
    add_compile_options(--coverage -fprofile-arcs -ftest-coverage)
    link_libraries(--coverage -fprofile-arcs -ftest-coverage)
  endif()
endif()

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_planning_interfaces REQUIRED)
find_package(controller_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(urdf REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(trajectory_planning_v3 REQUIRED)
find_package(trajectory_interpolator REQUIRED)
find_package(hardware_driver REQUIRED)
find_package(moveit_core REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)
set(ARM_CONTROLLER_TESSERACT_OSQP_DIR "/home/dyk/Bags/tesseract_ws/install/osqp/lib/cmake/osqp")
if(EXISTS "${ARM_CONTROLLER_TESSERACT_OSQP_DIR}/osqp-config.cmake")
  set(osqp_DIR "${ARM_CONTROLLER_TESSERACT_OSQP_DIR}" CACHE PATH "osqp config directory" FORCE)
endif()
set(ARM_CONTROLLER_TESSERACT_OSQP_LIB "/home/dyk/Bags/tesseract_ws/install/osqp/lib")
find_package(osqp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(csaps REQUIRED)

if(ARM_CONTROLLER_BUILD_MOTION_CONTROLLERS)
  find_package(ompl REQUIRED)
endif()

if(ARM_CONTROLLER_BUILD_MOTION_CONTROLLERS)
  find_package(tesseract_common REQUIRED)
  find_package(tesseract_environment REQUIRED)
  find_package(tesseract_motion_planners REQUIRED COMPONENTS trajopt trajopt_ifopt)
  find_package(trajopt REQUIRED)
  find_package(trajopt_ifopt REQUIRED)
endif()

# 抑制 Boost.Python 找不到 Python 3.10 的警告（这是一个已知问题，不影响编译）
set(Boost_NO_WARN_NEW_VERSIONS ON)

find_package(pinocchio REQUIRED)

if(BUILD_PYTHON_IPC_BINDINGS)
  find_package(pybind11 QUIET)
  find_package(Python3 QUIET COMPONENTS Interpreter)
endif()


include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# === 1. Build module object libraries ===
set(ARM_CONTROLLER_COMMON_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
)
function(arm_controller_configure_object_target TARGET_NAME)
  target_include_directories(${TARGET_NAME}
    PUBLIC
      ${ARM_CONTROLLER_COMMON_INCLUDE_DIRS}
  )
  set_target_properties(${TARGET_NAME} PROPERTIES
    POSITION_INDEPENDENT_CODE ON
  )
  if(BUILD_TESTING)
    if(CMAKE_COMPILER_IS_GNUCXX)
      target_compile_options(${TARGET_NAME} PRIVATE --coverage -fprofile-arcs -ftest-coverage)
      target_link_options(${TARGET_NAME} PRIVATE --coverage -fprofile-arcs -ftest-coverage)
    endif()
  endif()
endfunction()

set(ARM_CONTROLLER_COMMON_AMENT_DEPS
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  visualization_msgs
  trajectory_msgs
  control_msgs
  geometry_msgs
  trajectory_planning_interfaces
  controller_interfaces
  ament_index_cpp
  urdf
  trajectory_interpolator
  trajectory_planning_v3
  hardware_driver
  moveit_core
  orocos_kdl
  kdl_parser
  Eigen3
  tf2_ros
  tf2_geometry_msgs
  csaps
  pinocchio
)

add_library(arm_controller_runtime_objects OBJECT
  src/controller_manager_section.cpp
  src/trajectory_controller_section.cpp
  src/hardware/hardware_manager.cpp
  src/hardware/motor_data_recorder.cpp
  src/hardware/recorder_manager.cpp
  src/hardware/motor_data_reloader.cpp
  src/dynamics/gravity_compensator.cpp
  src/controller/controller_registry.cpp
  src/utils/trajectory_converter.cpp
  src/arm_controller_api.cpp
  src/ipc/shm_manager.cpp
  src/ipc/command_producer.cpp
  src/ipc/controller_state_manager.cpp
  src/ipc/ipc_context.cpp
)
arm_controller_configure_object_target(arm_controller_runtime_objects)
target_compile_definitions(arm_controller_runtime_objects
  PRIVATE
    ARM_CONTROLLER_ENABLE_MOTION_CONTROLLERS=$<BOOL:${ARM_CONTROLLER_BUILD_MOTION_CONTROLLERS}>
    ARM_CONTROLLER_ENABLE_VELOCITY_CONTROLLERS=$<BOOL:${ARM_CONTROLLER_BUILD_VELOCITY_CONTROLLERS}>
    ARM_CONTROLLER_ENABLE_TEACH_CONTROLLERS=$<BOOL:${ARM_CONTROLLER_BUILD_TEACH_CONTROLLERS}>
)
ament_target_dependencies(arm_controller_runtime_objects
  ${ARM_CONTROLLER_COMMON_AMENT_DEPS}
)
target_link_libraries(arm_controller_runtime_objects
  hardware_driver::hardware_driver_canfd
  osqp::osqp
  csaps::csaps
  pinocchio::pinocchio
  yaml-cpp
)

if(ARM_CONTROLLER_BUILD_VELOCITY_CONTROLLERS OR ARM_CONTROLLER_BUILD_MOTION_CONTROLLERS)
add_library(arm_controller_kinematics_objects OBJECT
  src/kinematics/forward_kinematics.cpp
  src/kinematics/jacobian_provider.cpp
)
arm_controller_configure_object_target(arm_controller_kinematics_objects)
ament_target_dependencies(arm_controller_kinematics_objects
  rclcpp
  trajectory_planning_v3
  Eigen3
  pinocchio
)
target_link_libraries(arm_controller_kinematics_objects
  pinocchio::pinocchio
)
endif()

if(ARM_CONTROLLER_BUILD_MOTION_CONTROLLERS)
add_library(arm_controller_neo_objects OBJECT
  src/algorithm/neo/hessian_builder.cpp
  src/algorithm/neo/joint_preference_loader.cpp
  src/algorithm/neo/joint_limit_damper.cpp
  src/algorithm/neo/manipulability_gradient.cpp
  src/algorithm/neo/manipulator_hessian_tensor.cpp
  src/algorithm/neo/body_obstacle_constraint_builder.cpp
  src/algorithm/neo/obstacle_damper.cpp
  src/algorithm/neo/reactive_qp_builder.cpp
  src/algorithm/neo/reactive_qp_problem.cpp
  src/algorithm/neo/reactive_qp_solver.cpp
  src/algorithm/neo/reactive_qp_validator.cpp
  src/algorithm/neo/task_velocity_generator.cpp
)
arm_controller_configure_object_target(arm_controller_neo_objects)
target_include_directories(arm_controller_neo_objects
  PRIVATE
    ${ARM_CONTROLLER_TESSERACT_OSQP_DIR}/../../../share/osqp/codegen_files/inc/private
)
ament_target_dependencies(arm_controller_neo_objects
  rclcpp
  Eigen3
  pinocchio
  trajectory_planning_v3
)
target_link_libraries(arm_controller_neo_objects
  osqp::osqp
  pinocchio::pinocchio
)
endif()

if(ARM_CONTROLLER_BUILD_MOTION_CONTROLLERS)
add_library(arm_controller_planning_objects OBJECT
  src/algorithm/cartesian_path_planner/collision/cartesian_collision_checker.cpp
  src/algorithm/cartesian_path_planner/collision/clearance_evaluator.cpp
  src/algorithm/cartesian_path_planner/collision/whole_body_ellipsoid_collision_checker.cpp
  src/algorithm/cartesian_path_planner/core/trajectory_parameterizer.cpp
  src/algorithm/global_planner/ompl_rrt_connect_global_planner.cpp
  src/algorithm/cartesian_path_planner/map/dummy_distance_field.cpp
  src/algorithm/cartesian_path_planner/map/camera_driver_pointcloud_map_adapter.cpp
  src/algorithm/cartesian_path_planner/map/camera_driver_esdf_map_client.cpp
  src/algorithm/cartesian_path_planner/global_trajectory/global_trajectory_manager.cpp
  src/algorithm/cartesian_path_planner/smoothing/shortcut_smoother.cpp
  src/algorithm/cartesian_path_planner/sampling/reference_sampler.cpp
  src/algorithm/sphere_model/link_sphere_model.cpp
)
arm_controller_configure_object_target(arm_controller_planning_objects)
target_include_directories(arm_controller_planning_objects
  SYSTEM PRIVATE
    ${OMPL_INCLUDE_DIRS}
)
ament_target_dependencies(arm_controller_planning_objects
  rclcpp
  sensor_msgs
  controller_interfaces
  trajectory_interpolator
  trajectory_planning_v3
  Eigen3
  pinocchio
)
target_link_libraries(arm_controller_planning_objects
  pinocchio::pinocchio
  ${OMPL_LIBRARIES}
)
endif()

add_library(arm_controller_core_controller_objects OBJECT
  # 只保留测试需要的控制器
  src/controller/system_start/system_start_controller.cpp
  src/controller/hold_state/hold_state_controller.cpp
  src/controller/ros2_action_control/ros2_action_control_controller.cpp
  src/controller/basic_ops/basic_ops_ipc_interface.cpp
  src/controller/basic_ops/basic_ops_ipc_service.cpp
)
arm_controller_configure_object_target(arm_controller_core_controller_objects)
ament_target_dependencies(arm_controller_core_controller_objects
  ${ARM_CONTROLLER_COMMON_AMENT_DEPS}
)
target_link_libraries(arm_controller_core_controller_objects
  hardware_driver::hardware_driver_canfd
  csaps::csaps
  pinocchio::pinocchio
  yaml-cpp
)

if(ARM_CONTROLLER_BUILD_MOTION_CONTROLLERS)
add_library(arm_controller_motion_controller_objects OBJECT
  src/controller/movec/movec_controller.cpp
  src/controller/movec/movec_interface.cpp
  src/controller/movec/movec_ipc_interface.cpp
  src/controller/movej/movej_controller.cpp
  src/controller/movej/movej_interface.cpp
  src/controller/movej/movej_ipc_interface.cpp
  src/controller/movel/movel_controller.cpp
  src/controller/movel/movel_interface.cpp
  src/controller/movel/movel_ipc_interface.cpp
)
arm_controller_configure_object_target(arm_controller_motion_controller_objects)
ament_target_dependencies(arm_controller_motion_controller_objects
  ${ARM_CONTROLLER_COMMON_AMENT_DEPS}
)
target_link_libraries(arm_controller_motion_controller_objects
  hardware_driver::hardware_driver_canfd
  csaps::csaps
  yaml-cpp
)
endif()

if(ARM_CONTROLLER_BUILD_VELOCITY_CONTROLLERS)
add_library(arm_controller_velocity_controller_objects OBJECT
  src/utils/velocity_qp_solver.cpp
  src/utils/velocity_abb_solver.cpp
  src/utils/velocity_strict_solver.cpp
  src/controller/joint_velocity/joint_velocity_controller.cpp
  src/controller/joint_velocity/joint_velocity_interface.cpp
  src/controller/joint_velocity/joint_velocity_ipc_interface.cpp
  src/controller/command_streaming/command_streaming_controller.cpp
  src/controller/command_streaming/command_streaming_interface.cpp
  src/controller/command_streaming/command_streaming_ipc_interface.cpp
  src/controller/cartesian_velocity/cartesian_velocity_controller.cpp
  src/controller/cartesian_velocity/cartesian_velocity_interface.cpp
  src/controller/cartesian_velocity/cartesian_velocity_ipc_interface.cpp
  src/controller/mink_servo/mink_servo_controller.cpp
  src/controller/mink_servo/mink_servo_interface.cpp
  src/controller/mink_servo/mink_servo_ipc_interface.cpp
)
arm_controller_configure_object_target(arm_controller_velocity_controller_objects)
target_include_directories(arm_controller_velocity_controller_objects
  BEFORE PRIVATE
    /usr/local/include/osqp
    /usr/local/share/osqp/codegen_files/inc/private
)
target_compile_definitions(arm_controller_velocity_controller_objects
  PRIVATE
    OSQP_EIGEN_OSQP_IS_V1
    OSQP_EIGEN_OSQP_IS_V1_FINAL
)
ament_target_dependencies(arm_controller_velocity_controller_objects
  ${ARM_CONTROLLER_COMMON_AMENT_DEPS}
)
target_link_libraries(arm_controller_velocity_controller_objects
  hardware_driver::hardware_driver_canfd
  osqp::osqp
  pinocchio::pinocchio
  yaml-cpp
)
endif()

if(ARM_CONTROLLER_BUILD_TEACH_CONTROLLERS)
add_library(arm_controller_teach_controller_objects OBJECT
  src/controller/trajectory_record/trajectory_record_controller.cpp
  src/controller/trajectory_record/trajectory_record_interface.cpp
  src/controller/trajectory_record/trajectory_smoother.cpp
  src/controller/trajectory_record/trajectory_record_ipc_interface.cpp
  src/controller/trajectory_replay/trajectory_replay_controller.cpp
  src/controller/trajectory_replay/trajectory_replay_command_section.cpp
  src/controller/trajectory_replay/trajectory_replay_interface.cpp
  src/controller/trajectory_replay/trajectory_segmenter.cpp
  src/controller/trajectory_replay/trajectory_replay_ipc_interface.cpp
)
arm_controller_configure_object_target(arm_controller_teach_controller_objects)
ament_target_dependencies(arm_controller_teach_controller_objects
  ${ARM_CONTROLLER_COMMON_AMENT_DEPS}
)
target_link_libraries(arm_controller_teach_controller_objects
  hardware_driver::hardware_driver_canfd
  csaps::csaps
  yaml-cpp
)
endif()

if(ARM_CONTROLLER_BUILD_MOTION_CONTROLLERS)
add_library(arm_controller_reactive_task_core_objects OBJECT
  src/controller/reactive_task/reactive_task_controller.cpp
  src/controller/reactive_task/controller/reactive_task_execution_state_machine.cpp
  src/controller/reactive_task/reactive_task_mapping_context.cpp
  src/controller/reactive_task/reactive_task_runtime.cpp
  src/controller/reactive_task/global_planner/reactive_task_planning_execution.cpp
  src/controller/reactive_task/global_planner/reactive_task_planning_session.cpp
  src/controller/reactive_task/global_planner/reactive_task_planning_runtime.cpp
  src/controller/reactive_task/controller/reactive_task_execution_loop.cpp
  src/controller/reactive_task/config/reactive_task_runtime_config.cpp
  src/controller/reactive_task/local_planner/reactive_task_async_local_planner_runner.cpp
  src/controller/reactive_task/controller/reactive_task_control_target_builder.cpp
  src/controller/reactive_task/diagnostics/reactive_task_diagnostics_publisher.cpp
  src/controller/reactive_task/obstacle/reactive_task_environment_probe.cpp
  src/controller/reactive_task/local_planner/reactive_task_local_planner_runtime.cpp
  src/controller/reactive_task/local_planner/reactive_task_local_reference_manager.cpp
  src/controller/reactive_task/obstacle/reactive_task_obstacle_selector.cpp
  src/controller/reactive_task/controller/reactive_task_neo_pipeline.cpp
  src/controller/reactive_task/controller/reactive_task_terminal_policy.cpp
  src/controller/reactive_task/controller/reactive_task_safety_policy.cpp
  src/controller/reactive_task/controller/reactive_task_watchdog.cpp
  src/controller/reactive_task/reactive_task_interface.cpp
  src/controller/reactive_task/reactive_task_ipc_interface.cpp
)
arm_controller_configure_object_target(arm_controller_reactive_task_core_objects)
ament_target_dependencies(arm_controller_reactive_task_core_objects
  ${ARM_CONTROLLER_COMMON_AMENT_DEPS}
)
target_link_libraries(arm_controller_reactive_task_core_objects
  hardware_driver::hardware_driver_canfd
  osqp::osqp
  csaps::csaps
  pinocchio::pinocchio
  yaml-cpp
  ${OMPL_LIBRARIES}
)
endif()

if(ARM_CONTROLLER_BUILD_MOTION_CONTROLLERS)
add_library(arm_controller_motion_local_planner_objects OBJECT
  src/controller/reactive_task/local_planner/reactive_task_local_planner.cpp
)
arm_controller_configure_object_target(arm_controller_motion_local_planner_objects)
ament_target_dependencies(arm_controller_motion_local_planner_objects
  Eigen3
)
target_link_libraries(arm_controller_motion_local_planner_objects
  tesseract::tesseract_motion_planners_trajopt
  tesseract::tesseract_motion_planners_trajopt_ifopt
  tesseract::tesseract_environment
  trajopt::trajopt
  trajopt::trajopt_ifopt
)
endif()

set(ARM_CONTROLLER_OBJECT_TARGETS
  $<TARGET_OBJECTS:arm_controller_runtime_objects>
  $<TARGET_OBJECTS:arm_controller_core_controller_objects>
)
if(ARM_CONTROLLER_BUILD_VELOCITY_CONTROLLERS OR ARM_CONTROLLER_BUILD_MOTION_CONTROLLERS)
  list(APPEND ARM_CONTROLLER_OBJECT_TARGETS
    $<TARGET_OBJECTS:arm_controller_kinematics_objects>)
endif()
if(ARM_CONTROLLER_BUILD_MOTION_CONTROLLERS)
  list(APPEND ARM_CONTROLLER_OBJECT_TARGETS
    $<TARGET_OBJECTS:arm_controller_motion_controller_objects>)
endif()
if(ARM_CONTROLLER_BUILD_VELOCITY_CONTROLLERS)
  list(APPEND ARM_CONTROLLER_OBJECT_TARGETS
    $<TARGET_OBJECTS:arm_controller_velocity_controller_objects>)
endif()
if(ARM_CONTROLLER_BUILD_TEACH_CONTROLLERS)
  list(APPEND ARM_CONTROLLER_OBJECT_TARGETS
    $<TARGET_OBJECTS:arm_controller_teach_controller_objects>)
endif()
if(ARM_CONTROLLER_BUILD_MOTION_CONTROLLERS)
  list(APPEND ARM_CONTROLLER_OBJECT_TARGETS
    $<TARGET_OBJECTS:arm_controller_neo_objects>
    $<TARGET_OBJECTS:arm_controller_planning_objects>
    $<TARGET_OBJECTS:arm_controller_reactive_task_core_objects>)
endif()
if(ARM_CONTROLLER_BUILD_MOTION_CONTROLLERS)
  list(APPEND ARM_CONTROLLER_OBJECT_TARGETS
    $<TARGET_OBJECTS:arm_controller_motion_local_planner_objects>)
endif()

# === 2. Build aggregate shared library ===
add_library(arm_controller_lib ${ARM_CONTROLLER_OBJECT_TARGETS})

# arm_controller_ipc (pybind11 MODULE) links against arm_controller_lib.
# Ensure this static library is built with -fPIC so it can be linked into a shared object.
set_target_properties(arm_controller_lib PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

ament_target_dependencies(arm_controller_lib
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  visualization_msgs
  trajectory_msgs
  control_msgs
  geometry_msgs
  trajectory_planning_interfaces
  controller_interfaces
  ament_index_cpp
  urdf
  trajectory_interpolator
  trajectory_planning_v3
  hardware_driver
  moveit_core
  orocos_kdl
  kdl_parser
  Eigen3
  tf2_ros
  tf2_geometry_msgs
  csaps
  pinocchio
)

# 链接硬件驱动库、osqp、csaps 和 pinocchio 到库
target_link_libraries(arm_controller_lib
  hardware_driver::hardware_driver_canfd
  osqp::osqp
  csaps::csaps
  pinocchio::pinocchio
  yaml-cpp
)

if(ARM_CONTROLLER_BUILD_MOTION_CONTROLLERS)
target_link_libraries(arm_controller_lib
  ${OMPL_LIBRARIES}
)
endif()

if(ARM_CONTROLLER_BUILD_MOTION_CONTROLLERS)
  target_link_libraries(arm_controller_lib
    tesseract::tesseract_motion_planners_trajopt
    tesseract::tesseract_motion_planners_trajopt_ifopt
    tesseract::tesseract_environment
    trajopt::trajopt
    trajopt::trajopt_ifopt
  )
endif()

# 为库本身添加覆盖率编译选项（在构建测试时）
if(BUILD_TESTING)
  if(CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options(arm_controller_lib PRIVATE --coverage -fprofile-arcs -ftest-coverage)
    target_link_options(arm_controller_lib PRIVATE --coverage -fprofile-arcs -ftest-coverage)
  endif()
endif()

# Node
add_executable(universial_arm_controller_node
  src/main.cpp
  src/trajectory_controller_section.cpp
  src/controller_manager_section.cpp
)
target_link_libraries(universial_arm_controller_node
  arm_controller_lib
  yaml-cpp
  hardware_driver::hardware_driver_canfd
)
target_compile_definitions(universial_arm_controller_node
  PRIVATE
    ARM_CONTROLLER_ENABLE_MOTION_CONTROLLERS=$<BOOL:${ARM_CONTROLLER_BUILD_MOTION_CONTROLLERS}>
    ARM_CONTROLLER_ENABLE_VELOCITY_CONTROLLERS=$<BOOL:${ARM_CONTROLLER_BUILD_VELOCITY_CONTROLLERS}>
    ARM_CONTROLLER_ENABLE_TEACH_CONTROLLERS=$<BOOL:${ARM_CONTROLLER_BUILD_TEACH_CONTROLLERS}>
)
target_link_options(universial_arm_controller_node PRIVATE
  -Wl,--disable-new-dtags
)
set_target_properties(universial_arm_controller_node PROPERTIES
  BUILD_RPATH "${ARM_CONTROLLER_TESSERACT_OSQP_LIB}"
  INSTALL_RPATH "${ARM_CONTROLLER_TESSERACT_OSQP_LIB}"
)
ament_target_dependencies(universial_arm_controller_node
  rclcpp
  std_msgs
  rclcpp_action
  control_msgs
  trajectory_msgs
  trajectory_interpolator
  hardware_driver
  controller_interfaces
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  universial_arm_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# === Build HTTP Server ===
add_executable(arm_controller_http_server
  src/http_server.cpp
)

target_link_libraries(arm_controller_http_server
  arm_controller_lib
)
target_compile_definitions(arm_controller_http_server
  PRIVATE
    ARM_CONTROLLER_ENABLE_MOTION_CONTROLLERS=$<BOOL:${ARM_CONTROLLER_BUILD_MOTION_CONTROLLERS}>
    ARM_CONTROLLER_ENABLE_VELOCITY_CONTROLLERS=$<BOOL:${ARM_CONTROLLER_BUILD_VELOCITY_CONTROLLERS}>
    ARM_CONTROLLER_ENABLE_TEACH_CONTROLLERS=$<BOOL:${ARM_CONTROLLER_BUILD_TEACH_CONTROLLERS}>
)

ament_target_dependencies(arm_controller_http_server
  rclcpp
  std_msgs
)

install(TARGETS arm_controller_http_server
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_PYTHON_IPC_BINDINGS)
  if(pybind11_FOUND)
    set(ARM_CONTROLLER_PYTHON_MODULE_INSTALL_DIR "lib/${PROJECT_NAME}/python")
    if(Python3_FOUND)
      set(ARM_CONTROLLER_PYTHON_MODULE_INSTALL_DIR
        "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/dist-packages")
    else()
      message(WARNING "Python3 interpreter not found. Falling back to non-standard python module install dir: ${ARM_CONTROLLER_PYTHON_MODULE_INSTALL_DIR}")
    endif()

    pybind11_add_module(arm_controller_ipc
      src/python_arm_controller_ipc_pybind.cpp
    )

    target_link_libraries(arm_controller_ipc
      PRIVATE
      arm_controller_lib
    )
    target_compile_definitions(arm_controller_ipc
      PRIVATE
        ARM_CONTROLLER_ENABLE_MOTION_CONTROLLERS=$<BOOL:${ARM_CONTROLLER_BUILD_MOTION_CONTROLLERS}>
        ARM_CONTROLLER_ENABLE_VELOCITY_CONTROLLERS=$<BOOL:${ARM_CONTROLLER_BUILD_VELOCITY_CONTROLLERS}>
        ARM_CONTROLLER_ENABLE_TEACH_CONTROLLERS=$<BOOL:${ARM_CONTROLLER_BUILD_TEACH_CONTROLLERS}>
    )

    target_include_directories(arm_controller_ipc
      PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

    install(TARGETS arm_controller_ipc
      LIBRARY DESTINATION ${ARM_CONTROLLER_PYTHON_MODULE_INSTALL_DIR}
    )

    configure_file(
      ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}_pythonpath.dsv.in
      ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_pythonpath.dsv
      @ONLY
    )
    ament_environment_hooks(
      ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_pythonpath.dsv
    )
  else()
    message(WARNING "BUILD_PYTHON_IPC_BINDINGS=ON but pybind11 not found. Python IPC module will not be built.")
  endif()
endif()

# === Build Examples ===
file(GLOB EXAMPLE_SOURCES example/example_*.cpp)
if(EXAMPLE_SOURCES)
  foreach(EXAMPLE_FILE ${EXAMPLE_SOURCES})
    get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)
    set(BUILD_THIS_EXAMPLE ON)
    if((EXAMPLE_NAME STREQUAL "example_velocity_control" OR EXAMPLE_NAME STREQUAL "example_mink_servo")
       AND NOT ARM_CONTROLLER_BUILD_VELOCITY_CONTROLLERS)
      set(BUILD_THIS_EXAMPLE OFF)
    endif()
    if(EXAMPLE_NAME STREQUAL "example_teach" AND NOT ARM_CONTROLLER_BUILD_TEACH_CONTROLLERS)
      set(BUILD_THIS_EXAMPLE OFF)
    endif()
    if((EXAMPLE_NAME STREQUAL "example_reactive_task_consumer"
        OR EXAMPLE_NAME STREQUAL "example_reactive_qp_neo"
        OR EXAMPLE_NAME STREQUAL "example_visualize_link_spheres")
       AND NOT ARM_CONTROLLER_BUILD_MOTION_CONTROLLERS)
      set(BUILD_THIS_EXAMPLE OFF)
    endif()
    if(NOT BUILD_THIS_EXAMPLE)
      continue()
    endif()
    add_executable(${EXAMPLE_NAME} ${EXAMPLE_FILE})

    # When BUILD_TESTING=ON, library has coverage flags, so examples must also have them
    if(BUILD_TESTING)
      if(CMAKE_COMPILER_IS_GNUCXX)
        target_compile_options(${EXAMPLE_NAME} PRIVATE --coverage -fprofile-arcs -ftest-coverage)
        target_link_options(${EXAMPLE_NAME} PRIVATE --coverage -fprofile-arcs -ftest-coverage)
      endif()
    endif()

    target_link_libraries(${EXAMPLE_NAME}
      arm_controller_lib
    )
    target_link_options(${EXAMPLE_NAME} PRIVATE
      -Wl,--disable-new-dtags
    )
    set_target_properties(${EXAMPLE_NAME} PROPERTIES
      BUILD_RPATH "${ARM_CONTROLLER_TESSERACT_OSQP_LIB}"
      INSTALL_RPATH "${ARM_CONTROLLER_TESSERACT_OSQP_LIB}"
    )
    ament_target_dependencies(${EXAMPLE_NAME}
      rclcpp
      std_msgs
      sensor_msgs
      geometry_msgs
    )
    install(TARGETS ${EXAMPLE_NAME}
      DESTINATION bin
    )
  endforeach()
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # 自动发现并编译所有测试文件
  file(GLOB TEST_SOURCES test/test_*.cpp)
  if(TEST_SOURCES)
    foreach(TEST_FILE ${TEST_SOURCES})
      get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

      ament_add_gtest(${TEST_NAME} ${TEST_FILE})

      target_link_libraries(${TEST_NAME}
        arm_controller_lib
        pinocchio::pinocchio
      )

      ament_target_dependencies(${TEST_NAME}
        rclcpp
        std_msgs
        sensor_msgs
        geometry_msgs
        trajectory_msgs
        trajectory_planning_interfaces
        controller_interfaces
        ament_index_cpp
        pinocchio
      )

      # 为测试目标添加覆盖率编译选项
      if(CMAKE_COMPILER_IS_GNUCXX)
        target_compile_options(${TEST_NAME} PRIVATE --coverage -fprofile-arcs -ftest-coverage)
        target_link_options(${TEST_NAME} PRIVATE --coverage -fprofile-arcs -ftest-coverage)
      endif()
    endforeach()
  else()
    message(WARNING "No test files found in test/ directory")
  endif()

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
