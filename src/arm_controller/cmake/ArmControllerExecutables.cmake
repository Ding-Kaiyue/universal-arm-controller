# Installed runtime executables and optional Python IPC module.

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
arm_controller_apply_feature_definitions(universial_arm_controller_node)
arm_controller_apply_coverage(universial_arm_controller_node)
target_link_options(universial_arm_controller_node PRIVATE
  -Wl,--disable-new-dtags
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

add_executable(arm_controller_http_server
  src/http_server.cpp
)
target_link_libraries(arm_controller_http_server
  arm_controller_lib
)
arm_controller_apply_feature_definitions(arm_controller_http_server)
arm_controller_apply_coverage(arm_controller_http_server)
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
    arm_controller_apply_feature_definitions(arm_controller_ipc)
    arm_controller_apply_coverage(arm_controller_ipc)
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
