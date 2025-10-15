#ifndef __DISABLE_INTERFACE_HPP__
#define __DISABLE_INTERFACE_HPP__

#include <controller_base/utility_controller_base.hpp>
#include <memory>

std::unique_ptr<UtilityControllerBase> createDisableController(const rclcpp::Node::SharedPtr & node);

#endif // __DISABLE_INTERFACE_HPP__