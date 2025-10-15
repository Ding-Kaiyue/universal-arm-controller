#include "cartesian_velocity_controller.hpp"
#include "controller_interface/cartesian_velocity_interface.hpp"

std::unique_ptr<VelocityControllerBase> createCartesianVelocityController(const rclcpp::Node::SharedPtr &node) {
    return std::make_unique<CartesianVelocityController>(node);
}

