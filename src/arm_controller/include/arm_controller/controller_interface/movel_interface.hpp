#ifndef __MOVEL_INTERFACE_CPP__
#define __MOVEL_INTERFACE_CPP__

#include "controller_base/trajectory_controller_base.hpp"
#include <memory>

std::unique_ptr<TrajectoryControllerBase> createMoveLController(const rclcpp::Node::SharedPtr& node);

#endif   // __MOVEL_INTERFACE_CPP__