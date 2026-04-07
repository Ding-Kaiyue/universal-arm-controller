#include "command_streaming_controller.hpp"
#include "controller_interface/command_streaming_interface.hpp"

std::unique_ptr<VelocityControllerBase> createCommandStreamingController(const rclcpp::Node::SharedPtr &node) {
    return std::make_unique<CommandStreamingController>(node);
}
