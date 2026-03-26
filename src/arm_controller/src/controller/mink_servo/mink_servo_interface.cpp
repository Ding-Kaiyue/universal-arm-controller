#include "mink_servo_controller.hpp"
#include "controller_interface/mink_servo_interface.hpp"

std::unique_ptr<VelocityControllerBase> createMinkServoController(const rclcpp::Node::SharedPtr& node) {
    return std::make_unique<MinkServoController>(node);
}

