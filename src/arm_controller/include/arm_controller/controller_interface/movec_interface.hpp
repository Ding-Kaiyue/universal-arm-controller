#ifndef __MOVEC_INTERFACE_HPP__
#define __MOVEC_INTERFACE_HPP__

#include <controller_base/trajectory_controller_base.hpp>
#include <memory>

std::unique_ptr<TrajectoryControllerBase> createMoveCController(const rclcpp::Node::SharedPtr& node);

#endif   // __MOVEC_INTERFACE_HPP__

