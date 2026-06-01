# Aggregate library assembled from the layered object libraries.

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
    $<TARGET_OBJECTS:arm_controller_motion_controller_objects>
    $<TARGET_OBJECTS:arm_controller_neo_objects>
    $<TARGET_OBJECTS:arm_controller_planning_objects>
    $<TARGET_OBJECTS:arm_controller_reactive_task_core_objects>
    $<TARGET_OBJECTS:arm_controller_motion_local_planner_objects>)
endif()

if(ARM_CONTROLLER_BUILD_VELOCITY_CONTROLLERS)
  list(APPEND ARM_CONTROLLER_OBJECT_TARGETS
    $<TARGET_OBJECTS:arm_controller_velocity_controller_objects>)
endif()

if(ARM_CONTROLLER_BUILD_TEACH_CONTROLLERS)
  list(APPEND ARM_CONTROLLER_OBJECT_TARGETS
    $<TARGET_OBJECTS:arm_controller_teach_controller_objects>)
endif()

add_library(arm_controller_lib ${ARM_CONTROLLER_OBJECT_TARGETS})
set_target_properties(arm_controller_lib PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

ament_target_dependencies(arm_controller_lib
  ${ARM_CONTROLLER_COMMON_AMENT_DEPS}
)

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
    tesseract::tesseract_motion_planners_trajopt
    tesseract::tesseract_motion_planners_trajopt_ifopt
    tesseract::tesseract_environment
    trajopt::trajopt
    trajopt::trajopt_ifopt
  )
endif()

arm_controller_apply_coverage(arm_controller_lib)
