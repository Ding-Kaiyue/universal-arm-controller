#include "move2start_controller.hpp"
#include "controller_interface/move2start_interface.hpp"

std::unique_ptr<UtilityControllerBase> createMove2StartController(const rclcpp::Node::SharedPtr &node) {
    return std::make_unique<Move2StartController>(node);
}
