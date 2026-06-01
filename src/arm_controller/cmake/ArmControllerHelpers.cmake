# Shared target configuration helpers.

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
  arm_controller_apply_coverage(${TARGET_NAME})
endfunction()

function(arm_controller_apply_feature_definitions TARGET_NAME)
  target_compile_definitions(${TARGET_NAME}
    PRIVATE
      ARM_CONTROLLER_ENABLE_MOTION_CONTROLLERS=$<BOOL:${ARM_CONTROLLER_BUILD_MOTION_CONTROLLERS}>
      ARM_CONTROLLER_ENABLE_VELOCITY_CONTROLLERS=$<BOOL:${ARM_CONTROLLER_BUILD_VELOCITY_CONTROLLERS}>
      ARM_CONTROLLER_ENABLE_TEACH_CONTROLLERS=$<BOOL:${ARM_CONTROLLER_BUILD_TEACH_CONTROLLERS}>
  )
endfunction()

function(arm_controller_apply_coverage TARGET_NAME)
  if(ARM_CONTROLLER_ENABLE_COVERAGE)
    if(CMAKE_COMPILER_IS_GNUCXX)
      target_compile_options(${TARGET_NAME} PRIVATE --coverage -fprofile-arcs -ftest-coverage)
      target_link_options(${TARGET_NAME} PRIVATE --coverage -fprofile-arcs -ftest-coverage)
    endif()
  endif()
endfunction()
