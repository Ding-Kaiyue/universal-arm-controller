#include "disable_controller.hpp"
#include "controller_interface/disable_interface.hpp"

std::unique_ptr<UtilityControllerBase> createDisableController(const rclcpp::Node::SharedPtr &node) {
    return std::make_unique<DisableController>(node);
}
