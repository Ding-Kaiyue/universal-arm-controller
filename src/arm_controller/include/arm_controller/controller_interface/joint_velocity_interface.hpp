#ifndef __JOINT_VELOCITY_INTERFACE_HPP__
#define __JOINT_VELOCITY_INTERFACE_HPP__

#include <controller_base/velocity_controller_base.hpp>
#include <memory>

std::unique_ptr<VelocityControllerBase> createJointVelocityController(const rclcpp::Node::SharedPtr& node);

#endif   // __JOINT_VELOCITY_INTERFACE_HPP__
