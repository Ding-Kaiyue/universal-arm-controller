# Example binaries.

file(GLOB EXAMPLE_SOURCES example/example_*.cpp)
if(EXAMPLE_SOURCES)
  foreach(EXAMPLE_FILE ${EXAMPLE_SOURCES})
    get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)
    set(BUILD_THIS_EXAMPLE ON)

    if((EXAMPLE_NAME STREQUAL "example_velocity_control")
       AND NOT ARM_CONTROLLER_BUILD_VELOCITY_CONTROLLERS)
      set(BUILD_THIS_EXAMPLE OFF)
    endif()
    if(EXAMPLE_NAME STREQUAL "example_teach" AND NOT ARM_CONTROLLER_BUILD_TEACH_CONTROLLERS)
      set(BUILD_THIS_EXAMPLE OFF)
    endif()
    if((EXAMPLE_NAME STREQUAL "example_dual_arm"
        OR EXAMPLE_NAME STREQUAL "example_reactive_task_consumer"
        OR EXAMPLE_NAME STREQUAL "example_reactive_qp_neo"
        OR EXAMPLE_NAME STREQUAL "example_visualize_link_spheres")
       AND NOT ARM_CONTROLLER_BUILD_MOTION_CONTROLLERS)
      set(BUILD_THIS_EXAMPLE OFF)
    endif()
    if(NOT BUILD_THIS_EXAMPLE)
      continue()
    endif()

    add_executable(${EXAMPLE_NAME} ${EXAMPLE_FILE})
    arm_controller_apply_coverage(${EXAMPLE_NAME})
    target_link_libraries(${EXAMPLE_NAME}
      arm_controller_lib
    )
    target_link_options(${EXAMPLE_NAME} PRIVATE
      -Wl,--disable-new-dtags
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
