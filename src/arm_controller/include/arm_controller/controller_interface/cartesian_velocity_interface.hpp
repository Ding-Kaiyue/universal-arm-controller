#ifndef __CARTESIAN_VELOCITY_INTERFACE_HPP__
#define __CARTESIAN_VELOCITY_INTERFACE_HPP__

#include <controller_base/velocity_controller_base.hpp>
#include <memory>

std::unique_ptr<VelocityControllerBase> createCartesianVelocityController(const rclcpp::Node::SharedPtr& node);

#endif  // __CARTESIAN_VELOCITY_INTERFACE_HPP__
