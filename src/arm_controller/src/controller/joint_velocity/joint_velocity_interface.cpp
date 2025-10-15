#include "joint_velocity_controller.hpp"
#include "controller_interface/joint_velocity_interface.hpp"

std::unique_ptr<VelocityControllerBase> createJointVelocityController(const rclcpp::Node::SharedPtr &node) {
    return std::make_unique<JointVelocityController>(node);
}
