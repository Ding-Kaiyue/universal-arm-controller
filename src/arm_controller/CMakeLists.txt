cmake_minimum_required(VERSION 3.8)
project(arm_controller)

# Ensure option() honors normal variables behavior consistently
if(POLICY CMP0077)
  cmake_policy(SET CMP0077 NEW)
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
find_package(OsqpEigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(csaps REQUIRED)


include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# === 1. Build shared library ===
add_library(arm_controller_lib
  src/controller_manager_section.cpp
  src/trajectory_controller_section.cpp
  src/hardware/hardware_manager.cpp
  src/hardware/motor_data_recorder.cpp
  src/controller/controller_registry.cpp
  src/utils/trajectory_converter.cpp
  src/utils/joint_recorder.cpp
  src/utils/velocity_qp_solver.cpp
  src/utils/velocity_abb_solver.cpp
  src/utils/velocity_strict_solver.cpp
  # 只保留测试需要的控制器
  src/controller/system_start/system_start_controller.cpp
  src/controller/joint_velocity/joint_velocity_controller.cpp
  src/controller/joint_velocity/joint_velocity_interface.cpp
  src/controller/hold_state/hold_state_controller.cpp
  # 其他控制器暂时屏蔽用于测试
  src/controller/cartesian_velocity/cartesian_velocity_controller.cpp
  src/controller/cartesian_velocity/cartesian_velocity_interface.cpp
  src/controller/move2initial/move2initial_controller.cpp
  src/controller/move2initial/move2initial_interface.cpp
  src/controller/move2start/move2start_controller.cpp
  src/controller/move2start/move2start_interface.cpp
  src/controller/ros2_action_control/ros2_action_control_controller.cpp
  src/controller/movec/movec_controller.cpp
  src/controller/movec/movec_interface.cpp
  src/controller/movej/movej_controller.cpp
  src/controller/movej/movej_interface.cpp
  src/controller/movel/movel_controller.cpp
  src/controller/movel/movel_interface.cpp
  src/controller/point_record/point_record_controller.cpp
  src/controller/point_record/point_record_interface.cpp
  # src/controller/point_replay/point_replay_controller.cpp
  # src/controller/point_replay/point_replay_interface.cpp
  src/controller/trajectory_record/trajectory_record_controller.cpp
  src/controller/trajectory_record/trajectory_record_interface.cpp
  src/controller/trajectory_replay/trajectory_replay_controller.cpp
  src/controller/trajectory_replay/trajectory_replay_interface.cpp
)

# target_link_libraries(robot_controller_lib
#   yaml-cpp
# )

ament_target_dependencies(arm_controller_lib
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
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
  OsqpEigen
  tf2_ros
  tf2_geometry_msgs
  csaps
)

# 链接硬件驱动库、osqp 和 csaps 到库
target_link_libraries(arm_controller_lib
  hardware_driver::hardware_driver_canfd
  OsqpEigen::OsqpEigen
  csaps::csaps
)

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
      )

      ament_target_dependencies(${TEST_NAME}
        rclcpp
        std_msgs
        sensor_msgs
        geometry_msgs
        trajectory_msgs
        trajectory_planning_interfaces
        controller_interfaces
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
