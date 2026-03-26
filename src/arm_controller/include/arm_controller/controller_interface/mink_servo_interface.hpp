#ifndef __MINK_SERVO_INTERFACE_HPP__
#define __MINK_SERVO_INTERFACE_HPP__

#include <controller_base/velocity_controller_base.hpp>
#include <memory>

std::unique_ptr<VelocityControllerBase> createMinkServoController(const rclcpp::Node::SharedPtr& node);

#endif  // __MINK_SERVO_INTERFACE_HPP__

