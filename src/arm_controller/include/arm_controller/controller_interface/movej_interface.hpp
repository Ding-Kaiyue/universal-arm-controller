#ifndef __MOVEJ_INTERFACE_HPP__
#define __MOVEJ_INTERFACE_HPP__

#include <controller_base/trajectory_controller_base.hpp>
#include <memory>

std::unique_ptr<TrajectoryControllerBase> createMoveJController(const rclcpp::Node::SharedPtr& node);

#endif   // __MOVEJ_INTERFACE_HPP__
