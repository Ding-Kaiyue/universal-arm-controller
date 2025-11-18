#include "move2initial_controller.hpp"
#include "controller_interface/move2initial_interface.hpp"

std::unique_ptr<UtilityControllerBase> createMove2InitialController(const rclcpp::Node::SharedPtr &node) {
    return std::make_unique<Move2InitialController>(node);
}
