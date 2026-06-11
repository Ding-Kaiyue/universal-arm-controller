# Test targets.

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  file(GLOB TEST_SOURCES test/test_*.cpp)
  if(TEST_SOURCES)
    foreach(TEST_FILE ${TEST_SOURCES})
      get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)
      set(BUILD_THIS_TEST ON)

      if((TEST_NAME STREQUAL "test_global_rrt_connect_integration"
          OR TEST_NAME STREQUAL "test_reactive_qp"
          OR TEST_NAME STREQUAL "test_trajopt_to_neo_reference_integration")
         AND NOT ARM_CONTROLLER_BUILD_MOTION_CONTROLLERS)
        set(BUILD_THIS_TEST OFF)
      endif()
      if(NOT BUILD_THIS_TEST)
        continue()
      endif()

      ament_add_gtest(${TEST_NAME} ${TEST_FILE})
      if(TEST_NAME STREQUAL "test_whole_body_goal_generator")
        target_sources(${TEST_NAME} PRIVATE
          src/controller/reactive_task/goal/whole_body_goal_generator.cpp
        )
      elseif(TEST_NAME STREQUAL "test_whole_body_local_planner")
        target_sources(${TEST_NAME} PRIVATE
          src/controller/reactive_task/local_planner/reactive_task_whole_body_local_planner.cpp
          src/controller/reactive_task/local_planner/whole_body_frontend_initializer.cpp
          src/controller/reactive_task/local_planner/whole_body_lbfgs_optimizer.cpp
          src/controller/reactive_task/local_planner/whole_body_local_target_selector.cpp
          src/controller/reactive_task/local_planner/whole_body_polynomial_trajectory.cpp
          src/algorithm/global_planner/layer_gap_rrt_connector.cpp
        )
      elseif(TEST_NAME STREQUAL "test_base_guided_whole_body_planner")
        target_sources(${TEST_NAME} PRIVATE
          src/algorithm/cartesian_path_planner/base/kino_astar_base_planner.cpp
          src/algorithm/global_planner/base_guided_whole_body_planner.cpp
          src/algorithm/global_planner/layer_gap_rrt_connector.cpp
        )
      elseif(TEST_NAME STREQUAL "test_layer_gap_rrt_connector")
        target_sources(${TEST_NAME} PRIVATE
          src/algorithm/global_planner/layer_gap_rrt_connector.cpp
        )
      elseif(TEST_NAME STREQUAL "test_base_footprint_collision_checker")
        target_sources(${TEST_NAME} PRIVATE
          src/algorithm/cartesian_path_planner/base/kino_astar_base_planner.cpp
          src/algorithm/cartesian_path_planner/collision/base_footprint_collision_checker.cpp
        )
      elseif(TEST_NAME STREQUAL "test_kino_astar_base_planner")
        target_sources(${TEST_NAME} PRIVATE
          src/algorithm/cartesian_path_planner/base/kino_astar_base_planner.cpp
        )
      else()
        target_link_libraries(${TEST_NAME}
          arm_controller_lib
          pinocchio::pinocchio
        )
      endif()
      ament_target_dependencies(${TEST_NAME}
        rclcpp
        std_msgs
        sensor_msgs
        geometry_msgs
        trajectory_msgs
        trajectory_planning_interfaces
        controller_interfaces
        ament_index_cpp
        Eigen3
        pinocchio
      )
      if(ARM_CONTROLLER_BUILD_MOTION_CONTROLLERS)
        target_include_directories(${TEST_NAME}
          SYSTEM PRIVATE
            ${OMPL_INCLUDE_DIRS}
        )
        target_link_libraries(${TEST_NAME}
          ${OMPL_LIBRARIES}
        )
      endif()
      arm_controller_apply_coverage(${TEST_NAME})
    endforeach()
  else()
    message(WARNING "No test files found in test/ directory")
  endif()

  ament_lint_auto_find_test_dependencies()
endif()
