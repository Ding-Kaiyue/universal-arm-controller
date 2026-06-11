# External package discovery.

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_planning_interfaces REQUIRED)
find_package(controller_interfaces REQUIRED)
find_package(camera_driver REQUIRED)
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
find_package(osqp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(csaps REQUIRED)

if(ARM_CONTROLLER_BUILD_MOTION_CONTROLLERS)
  find_package(ompl REQUIRED)
  find_package(tesseract_common REQUIRED)
  find_package(tesseract_environment REQUIRED)
  find_package(tesseract_motion_planners REQUIRED COMPONENTS trajopt trajopt_ifopt)
  find_package(trajopt REQUIRED)
  find_package(trajopt_ifopt REQUIRED)
endif()

set(Boost_NO_WARN_NEW_VERSIONS ON)
find_package(pinocchio REQUIRED)

if(BUILD_PYTHON_IPC_BINDINGS)
  find_package(pybind11 QUIET)
  find_package(Python3 QUIET COMPONENTS Interpreter)
endif()

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
  tf2
  tf2_ros
  tf2_geometry_msgs
  csaps
  pinocchio
)
