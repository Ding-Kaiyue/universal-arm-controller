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
      arm_controller_apply_coverage(${TEST_NAME})
    endforeach()
  else()
    message(WARNING "No test files found in test/ directory")
  endif()

  ament_lint_auto_find_test_dependencies()
endif()
