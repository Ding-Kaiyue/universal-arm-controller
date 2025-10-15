#ifndef __MOVE2INITIAL_INTERFACE_HPP__
#define __MOVE2INITIAL_INTERFACE_HPP__

#include <controller_base/utility_controller_base.hpp>
#include <memory>

std::unique_ptr<UtilityControllerBase> createMove2InitialController(const rclcpp::Node::SharedPtr& node);

#endif      // __MOVE2INITIAL_INTERFACE_HPP__