#ifndef __MOVE2START_INTERFACE_HPP__
#define __MOVE2START_INTERFACE_HPP__

#include <controller_base/utility_controller_base.hpp>
#include <memory>

std::unique_ptr<UtilityControllerBase> createMove2StartController(const rclcpp::Node::SharedPtr& node);

#endif      // __MOVE2START_INTERFACE_HPP__