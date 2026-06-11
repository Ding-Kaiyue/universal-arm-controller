cmake_minimum_required(VERSION 3.8)
project(arm_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(ArmControllerOptions)
include(ArmControllerDependencies)
include(ArmControllerHelpers)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${camera_driver_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../camera_driver/include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

include(ArmControllerObjectTargets)
include(ArmControllerLibrary)
include(ArmControllerExecutables)
include(ArmControllerExamples)
include(ArmControllerTests)

ament_package()
