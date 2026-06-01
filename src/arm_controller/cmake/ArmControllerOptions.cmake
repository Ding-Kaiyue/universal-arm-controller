# Build options and project-wide compiler/linker settings.

if(POLICY CMP0077)
  cmake_policy(SET CMP0077 NEW)
endif()

option(CMAKE_INTERPROCEDURAL_OPTIMIZATION "Enable LTO for optimization" OFF)
option(BUILD_PYTHON_IPC_BINDINGS "Build Python IPC producer bindings (pybind11)" ON)
option(ARM_CONTROLLER_ENABLE_COVERAGE "Enable gcov coverage instrumentation" OFF)
option(ARM_CONTROLLER_BUILD_MOTION_CONTROLLERS "Build motion controllers: MoveJ/MoveL/MoveC/ReactiveTask" ON)
option(ARM_CONTROLLER_BUILD_VELOCITY_CONTROLLERS "Build velocity controllers and velocity solver support" ON)
option(ARM_CONTROLLER_BUILD_TEACH_CONTROLLERS "Build teach/record/replay controllers" ON)

set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-keep-memory")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-keep-memory")

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

if(NOT DEFINED BUILD_TESTING)
  set(BUILD_TESTING OFF)
endif()

enable_testing()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-subobject-linkage)
endif()

if(ARM_CONTROLLER_ENABLE_COVERAGE)
  if(CMAKE_COMPILER_IS_GNUCXX)
    add_compile_options(--coverage -fprofile-arcs -ftest-coverage)
    link_libraries(--coverage -fprofile-arcs -ftest-coverage)
  endif()
endif()
