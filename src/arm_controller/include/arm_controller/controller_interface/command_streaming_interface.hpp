#ifndef __COMMAND_STREAMING_INTERFACE_HPP__
#define __COMMAND_STREAMING_INTERFACE_HPP__

#include <controller_base/velocity_controller_base.hpp>
#include <memory>

std::unique_ptr<VelocityControllerBase> createCommandStreamingController(const rclcpp::Node::SharedPtr& node);

#endif  // __COMMAND_STREAMING_INTERFACE_HPP__
